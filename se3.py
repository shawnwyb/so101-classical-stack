import numpy as np

class SE3:
    """Represent a rigid transform from a source frame to a target frame.

    R's columns are the source axes expressed in the target frame; t is
    the source origin expressed in the target frame. self.T stores the
    4-by-4 homogeneous transform that combines R and t.

    transform_point() converts a point, inverse() returns the reverse
    transform, and compose(other) applies other first, then this transform.
    """

    def __init__(self, R, t):
        R = np.asarray(R, dtype=float)
        t = np.asarray(t, dtype=float)
        if R.shape != (3, 3):
            raise ValueError("R must be a 3-by-3 rotation matrix")
        if t.shape != (3,):
            raise ValueError("t must be a 3-entry translation vector")
        if not np.all(np.isfinite(R)) or not np.all(np.isfinite(t)):
            raise ValueError("R and t must contain finite numbers")
        if not np.allclose(R.T @ R, np.eye(3), rtol=0, atol=1e-8):
            raise ValueError("R must have perpendicular, unit-length columns")
        if not np.isclose(np.linalg.det(R), 1, rtol=0, atol=1e-8):
            raise ValueError("R must preserve right-handed orientation")

        self.T = np.eye(4)
        self.T[:3, :3] = R
        self.T[:3, 3] = t
    
    def transform_point(self, point):
        point_homogeneous = np.append(point, 1)
        return (self.T @ point_homogeneous)[:-1]
    
    def inverse(self):
        R_transposed = self.T[:3, :3].T
        inverse_translation = -R_transposed @ self.T[:3, 3]
        return SE3(R_transposed, inverse_translation)

    def compose(self, other):
        composed_transform = self.T @ other.T
        composed_R = composed_transform[:3, :3]
        point_t = composed_transform[:3, 3]
        return SE3(composed_R, point_t)
