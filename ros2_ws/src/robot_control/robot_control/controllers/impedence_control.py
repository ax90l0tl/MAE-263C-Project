import numpy as np

def compute_y(J, M, xdd_d, xd_d, xd_e, x_d, x_e, Kd, Kp, qd):
    xd_error = xd_d - xd_e
    x_error = x_d - x_e
    y = np.linalg.inv(J) @ np.linalg.inv(M) @ (M@xdd_d + Kd @ xd_error + Kp @ x_error - M @ J @ qd)

    return y

def compute_impedence_output(B, y, n):
    return B @ y + n