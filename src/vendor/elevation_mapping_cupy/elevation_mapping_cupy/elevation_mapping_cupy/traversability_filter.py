#
# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under the MIT license. See LICENSE file in the project root for details.
#
import cupy as cp


def get_filter_cupy(w1, w2, w3, w_out):
    """Pure-cupy implementation of the ETH 3-branch dilated 3x3 CNN.

    Identical math to get_filter_torch / get_filter_chainer (4 conv2d, then
    a 1x1 12->1 conv, then exp(-|.|)) but built only from cupy element-wise
    ops + cp.concatenate. Used on Blackwell sm_120 where torch 2.7.1 has no
    sm_120 cubin and torch.cat fails with CUDA_ERROR_NO_BINARY_FOR_GPU.

    Args:
      w1, w2, w3: (4, 1, 3, 3) np.ndarray — conv weights for dilation 1/2/3
      w_out:      (1, 12, 1, 1) np.ndarray — 1x1 conv output layer
    """

    w1g = cp.asarray(w1, dtype=cp.float32)
    w2g = cp.asarray(w2, dtype=cp.float32)
    w3g = cp.asarray(w3, dtype=cp.float32)
    # Flatten 1x1 output conv to (12,) for a single dot-product.
    w_out_g = cp.asarray(w_out, dtype=cp.float32).reshape(12)

    def _conv3x3_dilated(x_HW, w_OIHW, d):
        # Valid (no padding) 3x3 conv with given dilation.
        # x_HW: (H, W) float32 cupy
        # w_OIHW: (OC, 1, 3, 3) cupy float32
        # returns: (OC, H-2d, W-2d) float32
        H, W = x_HW.shape
        oH, oW = H - 2 * d, W - 2 * d
        OC = w_OIHW.shape[0]
        out = cp.zeros((OC, oH, oW), dtype=cp.float32)
        for i in range(3):
            for j in range(3):
                patch = x_HW[i * d : i * d + oH, j * d : j * d + oW]
                wij = w_OIHW[:, 0, i, j].reshape(OC, 1, 1)
                out = out + wij * patch
        return out

    class TraversabilityFilterCupy:
        def __call__(self, elevation_cupy):
            x = elevation_cupy.astype(cp.float32, copy=False)
            out1 = _conv3x3_dilated(x, w1g, 1)  # (4, H-2, W-2)
            out2 = _conv3x3_dilated(x, w2g, 2)  # (4, H-4, W-4)
            out3 = _conv3x3_dilated(x, w3g, 3)  # (4, H-6, W-6)
            # Crop to a common (H-6, W-6).
            out1c = out1[:, 2:-2, 2:-2]
            out2c = out2[:, 1:-1, 1:-1]
            cat = cp.concatenate([out1c, out2c, out3], axis=0)  # (12, H-6, W-6)
            # 1x1 conv on absolute values, then exp(-.) bounded to (0, 1].
            conv_final = (w_out_g[:, None, None] * cp.abs(cat)).sum(axis=0)
            result = cp.exp(-conv_final)  # (H-6, W-6)
            # Match torch backend's (1, 1, H-6, W-6) output shape.
            return result.reshape(1, 1, result.shape[0], result.shape[1])

    return TraversabilityFilterCupy()


def get_filter_torch(*args, **kwargs):
    import torch
    import torch.nn as nn

    class TraversabilityFilter(nn.Module):
        def __init__(self, w1, w2, w3, w_out, device="cuda", use_bias=False):
            super(TraversabilityFilter, self).__init__()
            self.conv1 = nn.Conv2d(1, 4, 3, dilation=1, padding=0, bias=use_bias)
            self.conv2 = nn.Conv2d(1, 4, 3, dilation=2, padding=0, bias=use_bias)
            self.conv3 = nn.Conv2d(1, 4, 3, dilation=3, padding=0, bias=use_bias)
            self.conv_out = nn.Conv2d(12, 1, 1, bias=use_bias)

            # Set weights.
            self.conv1.weight = nn.Parameter(torch.from_numpy(w1).float())
            self.conv2.weight = nn.Parameter(torch.from_numpy(w2).float())
            self.conv3.weight = nn.Parameter(torch.from_numpy(w3).float())
            self.conv_out.weight = nn.Parameter(torch.from_numpy(w_out).float())

        def __call__(self, elevation_cupy):
            # Convert cupy tensor to pytorch.
            elevation_cupy = elevation_cupy.astype(cp.float32, copy=False)
            elevation = torch.as_tensor(elevation_cupy, device=self.conv1.weight.device)

            with torch.no_grad():
                out1 = self.conv1(elevation.view(-1, 1, elevation.shape[0], elevation.shape[1]))
                out2 = self.conv2(elevation.view(-1, 1, elevation.shape[0], elevation.shape[1]))
                out3 = self.conv3(elevation.view(-1, 1, elevation.shape[0], elevation.shape[1]))

                out1 = out1[:, :, 2:-2, 2:-2]
                out2 = out2[:, :, 1:-1, 1:-1]
                out = torch.cat((out1, out2, out3), dim=1)
                # out = F.concat((out1, out2, out3), axis=1)
                out = self.conv_out(out.abs())
                out = torch.exp(-out)
                out_cupy = cp.asarray(out)

            return out_cupy

    traversability_filter = TraversabilityFilter(*args, **kwargs).cuda().eval()
    return traversability_filter


def get_filter_chainer(*args, **kwargs):
    import os

    os.environ["CHAINER_WARN_VERSION_MISMATCH"] = "0"
    import chainer
    import chainer.links as L
    import chainer.functions as F

    class TraversabilityFilter(chainer.Chain):
        def __init__(self, w1, w2, w3, w_out, use_cupy=True):
            super(TraversabilityFilter, self).__init__()
            self.conv1 = L.Convolution2D(1, 4, ksize=3, pad=0, dilate=1, nobias=True, initialW=w1)
            self.conv2 = L.Convolution2D(1, 4, ksize=3, pad=0, dilate=2, nobias=True, initialW=w2)
            self.conv3 = L.Convolution2D(1, 4, ksize=3, pad=0, dilate=3, nobias=True, initialW=w3)
            self.conv_out = L.Convolution2D(12, 1, ksize=1, nobias=True, initialW=w_out)

            if use_cupy:
                self.conv1.to_gpu()
                self.conv2.to_gpu()
                self.conv3.to_gpu()
                self.conv_out.to_gpu()
            chainer.config.train = False
            chainer.config.enable_backprop = False

        def __call__(self, elevation):
            out1 = self.conv1(elevation.reshape(-1, 1, elevation.shape[0], elevation.shape[1]))
            out2 = self.conv2(elevation.reshape(-1, 1, elevation.shape[0], elevation.shape[1]))
            out3 = self.conv3(elevation.reshape(-1, 1, elevation.shape[0], elevation.shape[1]))

            out1 = out1[:, :, 2:-2, 2:-2]
            out2 = out2[:, :, 1:-1, 1:-1]
            out = F.concat((out1, out2, out3), axis=1)
            out = self.conv_out(F.absolute(out))
            return F.exp(-out).array

    traversability_filter = TraversabilityFilter(*args, **kwargs)
    return traversability_filter


if __name__ == "__main__":
    import cupy as cp
    from parameter import Parameter

    elevation = cp.random.randn(202, 202, dtype=cp.float32)
    print("elevation ", elevation.shape)
    param = Parameter()
    fc = get_filter_chainer(param.w1, param.w2, param.w3, param.w_out)
    print("chainer ", fc(elevation))

    ft = get_filter_torch(param.w1, param.w2, param.w3, param.w_out)
    print("torch ", ft(elevation))
