# encoding: utf-8
"""
@author:  xingyu liao
@contact: sherlockliao01@gmail.com
"""
import argparse
import glob
import os

import cv2
import numpy as np
import pycuda.driver as cuda
import tensorrt as trt
import tqdm

TRT_LOGGER = trt.Logger()


def get_parser():
    parser = argparse.ArgumentParser(description="trt model inference")

    parser.add_argument(
        "--model-path",
        default="outputs/trt_model/baseline.engine",
        help="trt model path"
    )
    parser.add_argument(
        "--input",
        nargs="+",
        help="A list of space separated input images; "
             "or a single glob pattern such as 'directory/*.jpg'",
    )
    parser.add_argument(
        "--output",
        default="trt_output",
        help="path to save trt model inference results"
    )
    parser.add_argument(
        '--batch-size',
        default=1,
        type=int,
        help='the maximum batch size of trt module'
    )
    parser.add_argument(
        "--height",
        type=int,
        default=256,
        help="height of image"
    )
    parser.add_argument(
        "--width",
        type=int,
        default=128,
        help="width of image"
    )
    return parser


class HostDeviceMem(object):
    """ Host and Device Memory Package """

    def __init__(self, host_mem, device_mem):
        self.host = host_mem
        self.device = device_mem

    def __str__(self):
        return "Host:\n" + str(self.host) + "\nDevice:\n" + str(self.device)

    def __repr__(self):
        return self.__str__()


class TrtEngine:

    def __init__(self, trt_file=None, gpu_idx=0, batch_size=1):
        print("[DEBUG] LOADED PATCHED TrtEngine from fast-reid/tools/deploy/trt_inference.py")
        cuda.init()
        self._batch_size = batch_size
        self._device_ctx = cuda.Device(gpu_idx).make_context()
        self._engine = self._load_engine(trt_file)
        self._context = self._engine.create_execution_context()

        self._is_trt10 = hasattr(self._engine, "num_io_tensors")

        self._input, self._output, self._bindings, self._stream, self._tensor_names = \
            self._allocate_buffers(self._context)

    def _load_engine(self, trt_file):
        with open(trt_file, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())
        if engine is None:
            raise RuntimeError("Failed to deserialize TensorRT engine: {}".format(trt_file))
        return engine

    def _get_io_tensors(self):
        """
        Return list of (index, name, is_input, dtype, shape)
        compatible with TensorRT 8.x and TensorRT 10.x.
        """
        tensors = []

        if hasattr(self._engine, "num_io_tensors"):
            # TensorRT 10.x
            for i in range(self._engine.num_io_tensors):
                name = self._engine.get_tensor_name(i)
                mode = self._engine.get_tensor_mode(name)
                is_input = mode == trt.TensorIOMode.INPUT
                dtype = trt.nptype(self._engine.get_tensor_dtype(name))
                shape = tuple(self._engine.get_tensor_shape(name))
                tensors.append((i, name, is_input, dtype, shape))
        else:
            # TensorRT 8.x
            for i in range(self._engine.num_bindings):
                name = self._engine.get_binding_name(i)
                is_input = self._engine.binding_is_input(i)
                dtype = trt.nptype(self._engine.get_binding_dtype(i))
                shape = tuple(self._engine.get_binding_shape(i))
                tensors.append((i, name, is_input, dtype, shape))

        return tensors

    def _fix_shape(self, shape):
        """
        Replace dynamic dims like -1 with current batch size.
        For ReID model, expected input is usually [B, 3, 256, 128].
        """
        fixed = []
        for dim in shape:
            if dim < 0:
                fixed.append(self._batch_size)
            else:
                fixed.append(dim)
        return tuple(fixed)

    def _allocate_buffers(self, context):
        inputs = []
        outputs = []
        bindings = []
        tensor_names = []
        stream = cuda.Stream()

        io_tensors = self._get_io_tensors()

        # TensorRT 10: if input shape is dynamic, set it before allocating.
        if self._is_trt10:
            for _, name, is_input, _, shape in io_tensors:
                if is_input and any(d < 0 for d in shape):
                    fixed_shape = self._fix_shape(shape)
                    context.set_input_shape(name, fixed_shape)

        for idx, name, is_input, dtype, shape in io_tensors:
            if self._is_trt10:
                # After setting input shape, query context shape when possible.
                try:
                    real_shape = tuple(context.get_tensor_shape(name))
                except Exception:
                    real_shape = shape
            else:
                real_shape = shape

            real_shape = self._fix_shape(real_shape)

            size = trt.volume(real_shape)
            if size <= 0:
                raise RuntimeError(
                    "Invalid TensorRT tensor shape: name={}, shape={}, real_shape={}".format(
                        name, shape, real_shape
                    )
                )

            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            bindings.append(int(device_mem))
            tensor_names.append(name)

            if is_input:
                inputs.append(HostDeviceMem(host_mem, device_mem))
            else:
                outputs.append(HostDeviceMem(host_mem, device_mem))

            print("[TRT_IO] name={}, is_input={}, dtype={}, shape={}, size={}".format(
                name, is_input, dtype, real_shape, size
            ))

        return inputs, outputs, bindings, stream, tensor_names

    def infer(self, data):
        [np.copyto(_inp.host, data.ravel()) for _inp in self._input]

        self._device_ctx.push()

        try:
            [cuda.memcpy_htod_async(inp.device, inp.host, self._stream) for inp in self._input]

            if self._is_trt10:
                # TensorRT 10.x: bind tensor addresses by name.
                for name, ptr in zip(self._tensor_names, self._bindings):
                    self._context.set_tensor_address(name, int(ptr))

                self._context.execute_async_v3(stream_handle=self._stream.handle)
            else:
                # TensorRT 8.x
                self._context.execute_async_v2(
                    bindings=self._bindings,
                    stream_handle=self._stream.handle
                )

            [cuda.memcpy_dtoh_async(out.host, out.device, self._stream) for out in self._output]
            self._stream.synchronize()

        finally:
            self._device_ctx.pop()

        return [out.host.reshape(self._batch_size, -1) for out in self._output[::-1]]

    def inference_on_images(self, imgs, new_size=(256, 128)):
        trt_inputs = []
        for img in imgs:
            input_ndarray = self.preprocess(img, *new_size)
            trt_inputs.append(input_ndarray)
        trt_inputs = np.vstack(trt_inputs)

        valid_bsz = trt_inputs.shape[0]
        if valid_bsz < self._batch_size:
            trt_inputs = np.vstack([
                trt_inputs,
                np.zeros(
                    (self._batch_size - valid_bsz, 3, *new_size),
                    dtype=np.float32
                )
            ])

        result, = self.infer(trt_inputs)
        result = result[:valid_bsz]
        feat = self.postprocess(result, axis=1)
        return feat

    @classmethod
    def preprocess(cls, img, img_height, img_width):
        resize_img = cv2.resize(img, (img_width, img_height), interpolation=cv2.INTER_CUBIC)
        type_img = resize_img.astype("float32").transpose(2, 0, 1)[np.newaxis]
        return type_img

    @classmethod
    def postprocess(cls, nparray, order=2, axis=-1):
        norm = np.linalg.norm(nparray, ord=order, axis=axis, keepdims=True)
        return nparray / (norm + np.finfo(np.float32).eps)

    def __del__(self):
        try:
            del self._input
            del self._output
            del self._stream
            self._device_ctx.detach()
        except Exception:
            pass


if __name__ == "__main__":
    args = get_parser().parse_args()

    trt = TrtEngine(args.model_path, batch_size=args.batch_size)

    if not os.path.exists(args.output): os.makedirs(args.output)

    if args.input:
        if os.path.isdir(args.input[0]):
            args.input = glob.glob(os.path.expanduser(args.input[0]))
            assert args.input, "The input path(s) was not found"
        inputs = []
        for img_path in tqdm.tqdm(args.input):
            img = cv2.imread(img_path)
            # the model expects RGB inputs
            cvt_img = img[:, :, ::-1]
            feat = trt.inference_on_images([cvt_img])
            np.save(os.path.join(args.output, os.path.basename(img_path).split('.')[0] + '.npy'), feat)
