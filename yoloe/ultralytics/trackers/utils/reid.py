# Ultralytics YOLO 🚀, AGPL-3.0 license

from pathlib import Path
import sys

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
        if img is None:
            raise ValueError("FastReID inference requires the original image")
        if len(dets) == 0:
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
        return features.detach().cpu().numpy().astype(np.float32)

    def _preprocess(self, crop: np.ndarray) -> torch.Tensor:
        """按 FastReID 测试输入尺寸 resize，并转为 CHW tensor。"""
        crop_rgb = crop
        crop_rgb = cv2.resize(crop_rgb, tuple(self.input_size[::-1]), interpolation=cv2.INTER_LINEAR)
        return torch.as_tensor(crop_rgb.astype("float32").transpose(2, 0, 1)).float()

    @staticmethod
    def _crop_detection(img: np.ndarray, det: np.ndarray) -> np.ndarray | None:
        """将 Ultralytics 的 xywh(+angle)+idx 检测结果转为图像裁剪区域。"""
        height, width = img.shape[:2]
        cx, cy, box_w, box_h = [float(v) for v in det[:4]]
        x1 = max(0, min(width - 1, int(round(cx - box_w / 2.0))))
        y1 = max(0, min(height - 1, int(round(cy - box_h / 2.0))))
        x2 = max(0, min(width, int(round(cx + box_w / 2.0))))
        y2 = max(0, min(height, int(round(cy + box_h / 2.0))))
        if x2 <= x1 or y2 <= y1:
            return None
        return img[y1:y2, x1:x2]
