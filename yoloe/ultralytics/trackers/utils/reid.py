# Ultralytics YOLO 🚀, AGPL-3.0 license

from pathlib import Path
import sys
import time

import cv2
import numpy as np
import torch
import torch.nn.functional as F


YOLOE_ROOT = Path(__file__).resolve().parents[3]


def _resolve_yoloe_path(path_value: str | Path, *, field_name: str) -> Path:
    """解析 ReID 配置路径，支持绝对路径和相对 yoloe 根目录的路径。"""
    if path_value is None or str(path_value).strip() == "":
        raise ValueError(f"{field_name} is required when BoT-SORT with_reid=True")

    path = Path(str(path_value)).expanduser()
    if not path.is_absolute():
        path = YOLOE_ROOT / path
    path = path.resolve()
    if not path.exists():
        raise FileNotFoundError(f"{field_name} not found: {path}")
    return path


class FastReIDEncoder:
    """FastReID 外观特征提取器，用于 BoT-SORT 的 ReID 分支。"""

    def __init__(
        self,
        config_path: str | Path,
        weights_path: str | Path,
        device: str = "cuda:0",
        root_path: str | Path = "fast-reid",
    ):
        self.root_path = _resolve_yoloe_path(root_path, field_name="fast_reid_root")
        self.config_path = _resolve_yoloe_path(config_path, field_name="fast_reid_config")
        self.weights_path = _resolve_yoloe_path(weights_path, field_name="fast_reid_weights")
        self.device = torch.device(device if torch.cuda.is_available() or not str(device).startswith("cuda") else "cpu")

        get_cfg, build_model, checkpointer_cls = self._import_fastreid(self.root_path)

        cfg = get_cfg()
        cfg.merge_from_file(str(self.config_path))
        cfg.MODEL.WEIGHTS = str(self.weights_path)
        cfg.MODEL.DEVICE = str(self.device)
        if hasattr(cfg.MODEL, "BACKBONE") and hasattr(cfg.MODEL.BACKBONE, "PRETRAIN"):
            cfg.MODEL.BACKBONE.PRETRAIN = False
        cfg.freeze()

        self.cfg = cfg
        self.model = build_model(cfg)
        checkpointer_cls(self.model).load(str(self.weights_path))
        self.model.to(self.device)
        self.model.eval()
        self.input_size = tuple(int(v) for v in cfg.INPUT.SIZE_TEST)
        self.backend = "fastreid"
        self.last_stats = {
            "backend": self.backend,
            "gpu": str(self.device).startswith("cuda"),
            "gpu_idx": int(str(self.device).split(":", 1)[1]) if str(self.device).startswith("cuda:") else -1,
            "feature_count": 0,
            "feature_dim": 0,
            "inference_ms": 0.0,
        }

    @staticmethod
    def _import_fastreid(root_path: Path):
        """兼容 pip fastreid 和原始 BoT-SORT fast_reid 子目录两种导入结构。"""
        root_text = str(root_path)
        if root_text not in sys.path:
            # fast-reid 源码仓库位于 yoloe/fast-reid，运行时需加入 sys.path 才能 import fastreid。
            sys.path.insert(0, root_text)

        try:
            from fastreid.config import get_cfg
            from fastreid.modeling import build_model
            from fastreid.utils.checkpoint import Checkpointer

            return get_cfg, build_model, Checkpointer
        except ImportError:
            pass

        try:
            from fast_reid.fastreid.config import get_cfg
            from fast_reid.fastreid.modeling.meta_arch import build_model
            from fast_reid.fastreid.utils.checkpoint import Checkpointer

            return get_cfg, build_model, Checkpointer
        except ImportError as exc:
            raise ImportError(
                "FastReID is required when BoT-SORT with_reid=True. "
                "Install fastreid in the YOLOE runtime environment or set with_reid=False."
            ) from exc

    def inference(self, img: np.ndarray, dets: np.ndarray) -> np.ndarray:
        """对检测框裁剪图像并返回 L2 归一化后的 ReID embedding。"""
        infer_t0 = time.perf_counter()
        if img is None:
            raise ValueError("FastReID inference requires the original image")
        if len(dets) == 0:
            self._record_stats(np.empty((0, 0), dtype=np.float32), infer_t0)
            return np.empty((0, 0), dtype=np.float32)

        crops = [self._crop_detection(img, det) for det in dets]
        valid_crops = [crop for crop in crops if crop is not None]
        if len(valid_crops) != len(crops):
            raise ValueError("FastReID received invalid detection boxes for image cropping")

        batch = torch.stack([self._preprocess(crop) for crop in valid_crops], dim=0).to(self.device)
        with torch.no_grad():
            features = self.model({"images": batch})
            if isinstance(features, dict):
                features = features.get("features", next(iter(features.values())))
            features = F.normalize(features, dim=1)
        result = features.detach().cpu().numpy().astype(np.float32)
        self._record_stats(result, infer_t0)
        return result

    def _record_stats(self, features: np.ndarray, start_time: float) -> None:
        """记录最近一次 ReID 推理统计，供 API 日志读取。"""
        self.last_stats = {
            "backend": self.backend,
            "gpu": str(self.device).startswith("cuda"),
            "gpu_idx": int(str(self.device).split(":", 1)[1]) if str(self.device).startswith("cuda:") else -1,
            "feature_count": int(features.shape[0]) if features.ndim >= 1 else 0,
            "feature_dim": int(features.shape[1]) if features.ndim >= 2 else 0,
            "inference_ms": round((time.perf_counter() - start_time) * 1000.0, 3),
        }

    def _preprocess(self, crop: np.ndarray) -> torch.Tensor:
        """按 FastReID 测试输入尺寸 resize，并转为 CHW tensor。"""
        crop_rgb = crop
        crop_rgb = cv2.resize(crop_rgb, tuple(self.input_size[::-1]), interpolation=cv2.INTER_LINEAR)
        return torch.as_tensor(crop_rgb.astype("float32").transpose(2, 0, 1)).float()

    @staticmethod
    def _crop_detection(img: np.ndarray, det: np.ndarray) -> np.ndarray | None:
        """将 Ultralytics 的 xywh(+angle)+idx 检测结果转为图像裁剪区域。"""
        if img is None or img.size == 0:
            return None
        det_xywh = np.asarray(det[:4], dtype=np.float32)
        if det_xywh.shape[0] < 4 or not np.isfinite(det_xywh).all():
            return None
        height, width = img.shape[:2]
        cx, cy, box_w, box_h = [float(v) for v in det_xywh]
        if box_w <= 1.0 or box_h <= 1.0:
            return None
        x1 = max(0, min(width - 1, int(round(cx - box_w / 2.0))))
        y1 = max(0, min(height - 1, int(round(cy - box_h / 2.0))))
        x2 = max(0, min(width, int(round(cx + box_w / 2.0))))
        y2 = max(0, min(height, int(round(cy + box_h / 2.0))))
        if x2 <= x1 or y2 <= y1:
            return None
        return img[y1:y2, x1:x2]


class TensorRTReIDEncoder:
    """TensorRT ReID 外观特征提取器，用于 BoT-SORT 的低延迟 ReID 分支。"""

    def __init__(
        self,
        engine_path: str | Path,
        *,
        batch_size: int = 16,
        input_size: tuple[int, int] = (256, 128),
        device: str = "cuda:0",
    ):
        self.engine_path = _resolve_yoloe_path(engine_path, field_name="fast_reid_engine")
        self.batch_size = max(1, int(batch_size))
        self.input_size = (int(input_size[0]), int(input_size[1]))
        self.gpu_idx = self._parse_gpu_idx(device)

        try:
            import pycuda.driver as cuda
            import tensorrt as trt
        except ImportError as exc:
            raise ImportError(
                "TensorRT ReID backend requires pycuda and tensorrt. "
                "Install them in the YOLOE runtime environment or set reid_backend=fastreid."
            ) from exc

        self.cuda = cuda
        self.trt = trt
        self.trt_logger = trt.Logger(trt.Logger.ERROR)
        self.cuda.init()
        self.device_ctx = self.cuda.Device(self.gpu_idx).retain_primary_context()
        self.device_ctx.push()
        try:
            self.engine = self._load_engine()
            self.context = self.engine.create_execution_context()
            self.inputs, self.outputs, self.bindings, self.stream = self._allocate_buffers()
        finally:
            self.device_ctx.pop()
        self.input_shape = tuple(int(v) for v in self.inputs[0]["shape"])
        self.engine_batch_size = int(self.input_shape[0])
        self.batch_size = min(self.batch_size, self.engine_batch_size)
        self.backend = "tensorrt"
        self.valid_indices = []
        self.last_stats = {
            "backend": self.backend,
            "feature_count": 0,
            "feature_dim": 0,
            "inference_ms": 0.0,
        }
        print(
            "[ReID][TensorRT] initialized",
            f"engine={self.engine_path}",
            f"gpu=True",
            f"gpu_idx={self.gpu_idx}",
            "cuda_context=primary",
            f"requested_batch={batch_size}",
            f"runtime_batch={self.batch_size}",
            f"engine_batch={self.engine_batch_size}",
            f"input_shape={self.input_shape}",
            f"output_shape={tuple(int(v) for v in self.outputs[-1]['shape'])}",
            flush=True,
        )

    @staticmethod
    def _parse_gpu_idx(device: str) -> int:
        device = str(device or "cuda:0").strip().lower()
        if device.startswith("cuda:") and device.split(":", 1)[1].isdigit():
            return int(device.split(":", 1)[1])
        return 0

    def _load_engine(self):
        with open(self.engine_path, "rb") as f, self.trt.Runtime(self.trt_logger) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())
        if engine is None:
            raise RuntimeError(f"failed to deserialize TensorRT ReID engine: {self.engine_path}")
        return engine

    def _binding_shape(self, binding):
        if isinstance(binding, int):
            return tuple(int(v) for v in self.engine.get_binding_shape(binding))
        return tuple(int(v) for v in self.engine.get_binding_shape(binding))

    def _binding_dtype(self, binding):
        return self.trt.nptype(self.engine.get_binding_dtype(binding))

    def _binding_is_input(self, binding) -> bool:
        return bool(self.engine.binding_is_input(binding))

    def _allocate_buffers(self):
        input_shapes = {}
        for binding in self.engine:
            if not self._binding_is_input(binding):
                continue
            index = self.engine.get_binding_index(binding)
            shape = self._binding_shape(binding)
            if any(dim < 0 for dim in shape):
                shape = (self.batch_size, 3, self.input_size[0], self.input_size[1])
                self.context.set_binding_shape(index, shape)
            input_shapes[index] = shape

        inputs, outputs, bindings = [], [], []
        stream = self.cuda.Stream()
        for binding in self.engine:
            index = self.engine.get_binding_index(binding)
            shape = input_shapes.get(index)
            if shape is None:
                shape = tuple(int(v) for v in self.context.get_binding_shape(index))
            if any(dim < 0 for dim in shape):
                raise RuntimeError(f"TensorRT ReID binding has unresolved dynamic shape: {binding} {shape}")
            size = int(self.trt.volume(shape))
            dtype = self._binding_dtype(binding)
            host_mem = self.cuda.pagelocked_empty(size, dtype)
            device_mem = self.cuda.mem_alloc(host_mem.nbytes)
            bindings.append(int(device_mem))
            item = {"host": host_mem, "device": device_mem, "shape": shape, "dtype": dtype, "name": str(binding)}
            if self._binding_is_input(binding):
                inputs.append(item)
            else:
                outputs.append(item)
        if len(inputs) != 1 or len(outputs) < 1:
            raise RuntimeError("TensorRT ReID engine must have one input and at least one output")
        return inputs, outputs, bindings, stream

    def inference(self, img: np.ndarray, dets: np.ndarray) -> np.ndarray:
        """对检测框裁剪图像并返回 L2 归一化后的 TensorRT ReID embedding。"""
        infer_t0 = time.perf_counter()
        if img is None:
            raise ValueError("TensorRT ReID inference requires the original image")
        if len(dets) == 0:
            self.valid_indices = []
            self._record_stats(np.empty((0, 0), dtype=np.float32), infer_t0)
            return np.empty((0, 0), dtype=np.float32)

        valid_crops = []
        self.valid_indices = []
        for index, det in enumerate(dets):
            crop = FastReIDEncoder._crop_detection(img, det)
            if crop is None:
                continue
            valid_crops.append(crop)
            self.valid_indices.append(index)
        if not valid_crops:
            self._record_stats(np.empty((0, 0), dtype=np.float32), infer_t0)
            print(
                "[ReID][TensorRT] inference",
                "gpu=True",
                f"gpu_idx={self.gpu_idx}",
                f"dets={len(dets)}",
                "valid=0",
                "features=(0, 0)",
                f"ms={self.last_stats['inference_ms']}",
                flush=True,
            )
            return np.empty((0, 0), dtype=np.float32)

        chunks = []
        for start in range(0, len(valid_crops), self.batch_size):
            batch_crops = valid_crops[start : start + self.batch_size]
            chunks.append(self._infer_batch(batch_crops))
        features = np.concatenate(chunks, axis=0)
        result = self._l2_normalize(features.astype(np.float32))
        self._record_stats(result, infer_t0)
        print(
            "[ReID][TensorRT] inference",
            "gpu=True",
            f"gpu_idx={self.gpu_idx}",
            f"dets={len(dets)}",
            f"valid={len(valid_crops)}",
            f"features={tuple(int(v) for v in result.shape)}",
            f"ms={self.last_stats['inference_ms']}",
            flush=True,
        )
        return result

    def _record_stats(self, features: np.ndarray, start_time: float) -> None:
        """记录最近一次 TensorRT ReID 推理统计，供 API 日志读取。"""
        self.last_stats = {
            "backend": self.backend,
            "feature_count": int(features.shape[0]) if features.ndim >= 1 else 0,
            "feature_dim": int(features.shape[1]) if features.ndim >= 2 else 0,
            "inference_ms": round((time.perf_counter() - start_time) * 1000.0, 3),
        }

    def _infer_batch(self, crops: list[np.ndarray]) -> np.ndarray:
        batch = np.stack([self._preprocess(crop) for crop in crops], axis=0)
        valid_bsz = batch.shape[0]
        if valid_bsz < self.engine_batch_size:
            padding = np.zeros(
                (self.engine_batch_size - valid_bsz, 3, self.input_size[0], self.input_size[1]), dtype=np.float32
            )
            batch = np.concatenate([batch, padding], axis=0)
        batch = np.ascontiguousarray(batch.astype(self.inputs[0]["dtype"]))

        self.device_ctx.push()
        try:
            np.copyto(self.inputs[0]["host"], batch.ravel())
            self.cuda.memcpy_htod_async(self.inputs[0]["device"], self.inputs[0]["host"], self.stream)
            self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
            for output in self.outputs:
                self.cuda.memcpy_dtoh_async(output["host"], output["device"], self.stream)
            self.stream.synchronize()
        finally:
            self.device_ctx.pop()

        output = self.outputs[-1]
        features = output["host"].reshape(self.engine_batch_size, -1)[:valid_bsz]
        return features.copy()

    def _preprocess(self, crop: np.ndarray) -> np.ndarray:
        """按 TensorRT engine 输入尺寸 resize，并转为 CHW float32。"""
        height, width = self.input_size
        crop_rgb = cv2.resize(crop, (width, height), interpolation=cv2.INTER_CUBIC)
        return crop_rgb.astype("float32").transpose(2, 0, 1)

    @staticmethod
    def _l2_normalize(features: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(features, ord=2, axis=1, keepdims=True)
        return features / (norm + np.finfo(np.float32).eps)

    def __del__(self):
        context = getattr(self, "device_ctx", None)
        if context is not None:
            try:
                context.detach()
            except Exception:
                pass


def build_reid_encoder(args):
    """根据 botsort.yaml 构造 ReID encoder。"""
    backend = str(getattr(args, "reid_backend", "fastreid") or "fastreid").strip().lower()
    print(f"[ReID] build backend={backend} with_reid={getattr(args, 'with_reid', None)}", flush=True)
    if backend in {"fastreid", "pytorch"}:
        return FastReIDEncoder(
            config_path=args.fast_reid_config,
            weights_path=args.fast_reid_weights,
            device=getattr(args, "reid_device", "cuda:0"),
            root_path=getattr(args, "fast_reid_root", "fast-reid"),
        )
    if backend in {"tensorrt", "trt"}:
        return TensorRTReIDEncoder(
            engine_path=args.fast_reid_engine,
            batch_size=getattr(args, "reid_batch_size", 16),
            input_size=tuple(getattr(args, "reid_input_size", [256, 128])),
            device=getattr(args, "reid_device", "cuda:0"),
        )
    raise ValueError(f"unsupported BoT-SORT ReID backend: {backend}")
