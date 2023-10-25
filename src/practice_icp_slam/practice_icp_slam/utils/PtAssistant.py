import random 
import numpy as np

def random_sampling(orig_points, num_points):
    assert orig_points.shape[0] > num_points

    points_down_idx = random.sample(range(orig_points.shape[0]), num_points)
    down_points = orig_points[points_down_idx, :]

    return down_points
