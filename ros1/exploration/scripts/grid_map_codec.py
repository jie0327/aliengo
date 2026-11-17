#!/usr/bin/env python3
"""Official GridMap column-major layout, with matrix index zero at +x,+y.

ROS imports are delayed so the wire layout and coordinates can be tested on a
host without ROS. Circular-buffer start indices are always zero in this mapper.
"""
import numpy as np


def encode_layer(values):
    values = np.asarray(values, dtype=np.float32)
    if values.ndim != 2 or min(values.shape) < 1:
        raise ValueError('GridMap layer must be a nonempty matrix')
    nx, ny = values.shape
    return values.ravel(order='F').tolist(), [('column_index', ny, nx*ny), ('row_index', nx, nx)]


def decode_layer(data, dimensions):
    if len(dimensions) != 2 or dimensions[0][0] != 'column_index' or dimensions[1][0] != 'row_index':
        raise ValueError('expected official column-major GridMap layout')
    ny, nx = int(dimensions[0][1]), int(dimensions[1][1])
    if nx <= 0 or ny <= 0 or dimensions[0][2] != nx*ny or dimensions[1][2] != nx:
        raise ValueError('invalid GridMap strides')
    return np.asarray(data, dtype=np.float32).reshape((nx, ny), order='F').copy()


def grid_position(index, shape, resolution, center):
    return np.asarray(center) + (np.asarray(shape)/2. - np.asarray(index) - .5)*resolution


def to_grid_message(values, resolution, center, stamp, frame_id='lio_map'):
    from grid_map_msgs.msg import GridMap
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension
    values = np.asarray(values)
    data, dimensions = encode_layer(values)
    message = GridMap()
    message.info.header.stamp = stamp
    message.info.header.frame_id = frame_id
    message.info.resolution = resolution
    message.info.length_x = values.shape[0]*resolution
    message.info.length_y = values.shape[1]*resolution
    message.info.pose.position.x, message.info.pose.position.y = center
    message.info.pose.orientation.w = 1.
    message.layers = ['elevation']
    message.basic_layers = ['elevation']
    layer = Float32MultiArray()
    layer.layout.dim = [MultiArrayDimension(label=label, size=size, stride=stride)
                        for label, size, stride in dimensions]
    layer.layout.data_offset = 0
    layer.data = data
    message.data = [layer]
    message.outer_start_index = message.inner_start_index = 0
    return message
