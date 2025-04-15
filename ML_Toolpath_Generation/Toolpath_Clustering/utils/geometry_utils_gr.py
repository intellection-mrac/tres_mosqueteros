import numpy as np

def compute_local_frame(points):
    """
    Computes a local reference frame (U, V, N) from a list of 3D points.
    Returns the transformation matrix T (world to UVN) and its inverse (UVN to world).
    """
    points = np.array(points)
    origin = np.mean(points, axis=0)
    u = points[1] - points[0]
    u = u / np.linalg.norm(u)
    normal = np.cross(u, points[2] - points[0])
    normal = normal / np.linalg.norm(normal)
    v = np.cross(normal, u)

    T = np.eye(4)
    T[:3, 0] = u
    T[:3, 1] = v
    T[:3, 2] = normal
    T[:3, 3] = origin

    T_inv = np.linalg.inv(T)
    return T, T_inv

def project_to_2d(points_3d, T_inv):
    """
    Projects 3D points onto a 2D UV plane using the inverse transformation matrix.
    Returns Nx2 array of (u, v) points.
    """
    points_3d = np.array(points_3d)
    pts_hom = np.hstack([points_3d, np.ones((len(points_3d), 1))])
    projected = (T_inv @ pts_hom.T).T
    return projected[:, :2]

def reproject_to_3d(points_2d, T):
    """
    Reprojects 2D (u, v) points back into 3D world space using the original transform.
    Assumes z = 0 in local UVN coordinates.
    """
    points_2d = np.array(points_2d)
    pts_uvn = np.hstack([points_2d, np.zeros((len(points_2d), 1)), np.ones((len(points_2d), 1))])
    reprojected = (T @ pts_uvn.T).T
    return reprojected[:, :3]
