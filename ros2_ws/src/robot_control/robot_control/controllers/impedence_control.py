import numpy as np
from robot_control.robot_utilities.robot_kinematics import Robot
from scipy import constants

def compute_y(x_d, xd_d, xdd_d, x_e, q, qd, robot, Md, Kp, Kd):
    qd = np.atleast_2d(qd).T
    xdd_d = np.atleast_2d(xdd_d).T
    x_error = np.atleast_2d(x_d - x_e).T

    J = robot.jacobian(q)
    J_square = J[[0, 2], :]
    J_square_d = robot.jacobian_derivative(q, qd)[[0, 2]]
    xd_e = J_square @ qd
    xd_error = np.atleast_2d(xd_d).T - xd_e

    # print("x_error:", x_error, "xd_error:", xd_error)
    y = np.linalg.inv(J_square) @ np.linalg.inv(Md) @ (Md@xdd_d + Kd @ xd_error + Kp @ x_error - Md @ J_square_d @ qd)
    # print("y: ", y)
    return y


def compute_impedence_output(y, q, qd, robot):
    J = robot.jacobian(q)
    B, V, G = robot.dynamics(q, qd)
    T = robot.forward_kinematics2(q)
    R_0_T = T[:3, :3]
    R_T_0 = R_0_T.T
    T_e = np.vstack((np.hstack((R_T_0, np.zeros((3, 3)))), 
                    np.hstack((np.zeros((3, 3)), R_T_0))))
    # Force of gravity on base link treated as normal force applied to end-effector
    F = J.T @ T_e @ np.array([[0], [0], [robot.inertial_properties[0]['mass'] * constants.g], [0], [0], [0]])
    # print("g: ", G, F)
    return B @ y + V

if __name__ == "__main__":
    robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
    m0 = 10
    g = 9.81
    q = np.array([-1, 2.62])
    Jacobian = robot.jacobian(q)
    print(Jacobian.T)
    T = robot.forward_kinematics(q)
    print(np.round(T, 4))
    R_0_T = T[:3, :3]
    R_T_0 = R_0_T.T
    print(R_T_0)
    print(R_0_T)
    T_e = np.vstack((np.hstack((R_T_0, np.zeros((3, 3)))), 
                     np.hstack((np.zeros((3, 3)), R_T_0))))
    g = Jacobian.T @ T_e @ np.array([[0], [0], [m0 * g], [0], [0], [0]])
    print(g)
