import numpy as np
from robot_control.robot_utilities.robot_kinematics import Robot
from scipy import constants

def inverse_dynamics_control(q, qd, q_d, qd_d, qdd_d, robot, Kp, Kd):
    qdd_d = np.atleast_2d(qdd_d).T
    q_error = np.atleast_2d(q_d - q).T
    qd_error = np.atleast_2d(qd_d - qd).T
    y = qdd_d + Kp @ q_error + Kd @ qd_error
    M, V, G = robot.dynamics(q, qd)
    u = M @ y + V + G
    return u


if __name__ == "__main__":
    robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
    q = np.array([0.46,  1.3156])
    qd = np.array([0.0, 0.0])
    q_d = np.array([0.0, -0.3])
    qd_d = np.array([0.0, 0.0])
    qdd_d = np.array([0.0, 0.0])
    Kp = np.diag([100.0, 10.0])
    Kd = np.diag([10.0, 1.0])

    u = inverse_dynamics_control(q, qd, q_d, qd_d, qdd_d, robot, Kp, Kd)
    print("Control output (torque):", u)