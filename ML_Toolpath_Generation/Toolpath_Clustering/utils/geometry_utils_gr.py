import numpy as np

def compute_local_frame(points_3d):
    """
    Create a local coordinate frame using PCA on 3D points.
    Returns T (3x3) and T_inv transformation matrices.
    """
    centroid = np.mean(points_3d, axis=0)
    centered = points_3d - centroid
    cov = np.cov(centered.T)

    eigvals, eigvecs = np.linalg.eigh(cov)
    sorted_indices = np.argsort(eigvals)[::-1]
    eigvecs = eigvecs[:, sorted_indices]

    x_axis = eigvecs[:, 0]
    y_axis = eigvecs[:, 1]
    z_axis = np.cross(x_axis, y_axis)
    if np.linalg.norm(z_axis) == 0:
        # fallback: use default Z if collinear
        z_axis = np.array([0, 0, 1])
    else:
        z_axis /= np.linalg.norm(z_axis)

    R = np.vstack([x_axis, y_axis, z_axis]).T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = centroid

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
