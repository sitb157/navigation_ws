"""
ref: https://github.com/gisbi-kim/PyICP-SLAM'
"""

import numpy as np
from sklearn.neighbors import NearestNeighbors

def find_nearest_neighbor(src, dst):

    assert src.shape == dst.shape

    neighbors = NearestNeighbors(n_neighbors=1)
    neighbors.fit(dst)
    distances, indices = neighbors.kneighbors(src, return_distance=True)

    return distances.ravel(), indices.ravel()

def calculate_transform(A, B):

    assert A.shape == B.shape

    m = A.shape[1]
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    if np.linalg.det(R) < 0:
        Vt[m-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    t = centroid_B.T - np.dot(R, centroid_A.T)

    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t

def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001):
    assert A.shape == B.shape

    m = A.shape[1]

    src = np.ones((m+1, A.shape[0]))
    dst = np.ones((m+1, B.shape[0]))
    src[:m, :] = np.copy(A.T)
    dst[:m, :] = np.copy(B.T)

    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    for i in range(max_iterations):
        distances, indices = find_nearest_neighbor(src[:m, :].T, dst[:m, :].T)
        T, _, _ = calculate_transform(src[:m,:].T, dst[:m, indices].T)

        src = np.dot(T, src)

        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    T, _, _ = calculate_transform(A, src[:m, :].T)
    return T, distances, i
