import numpy as np

def depth_buffer_to_meters(depth_buffer, near, far):
    return far * near / (far - (farnear) * depth_buffer)