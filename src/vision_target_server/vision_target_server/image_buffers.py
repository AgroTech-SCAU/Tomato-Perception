"""Decode ROS image buffers with explicit stride and byte order handling."""
import cv2
import numpy as np


def image_array(message):
    formats = {
        'rgb8': ('u1', 3), 'bgr8': ('u1', 3), 'rgba8': ('u1', 4),
        'bgra8': ('u1', 4), 'mono8': ('u1', 1),
        '16UC1': ('u2', 1), 'mono16': ('u2', 1), '32FC1': ('f4', 1), '64FC1': ('f8', 1),
    }
    if message.encoding not in formats:
        raise ValueError(f'Unsupported image encoding: {message.encoding}')
    kind, channels = formats[message.encoding]
    dtype = np.dtype(kind).newbyteorder('>' if message.is_bigendian else '<')
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step < width * channels * dtype.itemsize:
        raise ValueError('Invalid image dimensions or row stride')
    data = memoryview(message.data)
    if data.nbytes < height * step:
        raise ValueError('Truncated image buffer')
    shape = (height, width) if channels == 1 else (height, width, channels)
    strides = (step, dtype.itemsize) if channels == 1 else (step, channels * dtype.itemsize, dtype.itemsize)
    return np.ndarray(shape, dtype=dtype, buffer=data, strides=strides).astype(np.dtype(kind), copy=True)


def color_bgr(message):
    if message.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'):
        raise ValueError('Expected an 8-bit color image')
    image = image_array(message)
    conversions = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                   'bgra8': cv2.COLOR_BGRA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}
    return cv2.cvtColor(image, conversions[message.encoding]) if message.encoding in conversions else image


def depth_array(message):
    if message.encoding not in ('16UC1', 'mono16', '32FC1', '64FC1'):
        raise ValueError('Expected a single-channel metric depth image')
    return image_array(message)
