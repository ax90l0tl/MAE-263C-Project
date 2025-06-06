import numpy as np
from robot_control.robot_utilities.robot_kinematics import Robot
from scipy import constants
import matplotlib.pyplot as plt

def active_compliance_control(x_d, x_e, q, qd, robot, Kp, Kd):

    """
    Compute the control output for active compliance control.
    
    :param x_d: 1xN desired position
    :param x_e: 1xN current position
    :param q: Joint angles of the robot.
    :param q_dot: Joint velocities of the robot.
    :param a1: Length of the first link.
    :param a2: Length of the second link.
    :param m1: Mass of the first link.
    :param m2: Mass of the second link.
    :param Jacobian: 6xN Jacobian matrix of the robot.
    :param Kp: Proportional gain 2x2.
    :param Kd: Derivative gain 2x2.
    :return: Control output (torque).
    """

    x_error = np.atleast_2d(x_d - x_e).T
    print("x_error: ", x_error)
    qd = np.atleast_2d(qd.copy()).T
    J = robot.jacobian(q)
    T_0_T = robot.forward_kinematics2(q)
    R_0_T = T_0_T[:3, :3]
    R_T_0 = R_0_T.T

    q_d = robot.inverse_kinematics(x_d)[0]
    
    T_d = robot.forward_kinematics2(q_d)
    R_d = T_d[:3, :3]
    R_d_T = R_d.T

    T_d = np.vstack((np.hstack((R_d_T, np.zeros((3, 3)))), 
                    np.hstack((np.zeros((3, 3)), R_d_T))))
    # COM in urdf is defined at the base of the link TODO need to change
    # Treat base link mass as point mass
    # Treat gravity as normal force
    # T_e = np.vstack((np.hstack((R_T_0, np.zeros((3, 3)))), 
                    # np.hstack((np.zeros((3, 3)), R_T_0))))
    # g = J.T @ np.array([[0], [0], 
                            #   [(robot.inertial_properties[0]['mass'] + robot.inertial_properties[1]['mass'] + robot.inertial_properties[2]['mass']) * constants.g], 
                            #   [0], [0], [0]])

    M, V, G = robot.dynamics(q, qd)

    
    # J_a_d = (T_d @ J )[[0, 2]]
    J_a_d = J[[0, 2]]
    # print("g", g)
    control_output = np.zeros((2, 1))
    control_output = G + J_a_d.T @ (Kp @ x_error - Kd @ (J_a_d @ qd))
    return control_output

if __name__ == "__main__":
    robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
    fig, ax = plt.subplots()

    q = np.array([0.46,  1.3156])
    x = robot.forward_kinematics(q)[[0, 2], -1]
    x_d = np.array([0.0, -0.3])
    Kp = np.diag([100.0, 10.0])
    Kd = np.diag([0.0, 0.0])
    qd = np.array([0.0, 0.0])
    q_d = robot.inverse_kinematics(x_d)[0]
    T_d = robot.forward_kinematics(q_d)
    R_d = T_d[0:3, 0:3]
    # print(np.round(R_d, 10))
    T = np.vstack((np.hstack((R_d, np.zeros((3, 3)))), np.hstack((np.zeros((3, 3)), R_d))))
    jacobian = robot.jacobian(q)
    v = jacobian @ np.array([[1],[0.1]])
    vt = T @ jacobian @ np.array([[1],[0]])
    # print(np.round(jacobian, 10))
    # print(np.round(v, 10))
    # print(np.round(T @ v, 10))
    robot.plot_robot_arm(q, x, fig, ax, "current")
    robot.plot_robot_arm(q_d, x_d, fig, ax, "desired")
    plt.quiver(x[0], x[1], v[0], v[2], angles='xy', scale_units='xy', scale=1, color='r', label='velocity vector')
    plt.quiver(x[0], x[1], vt[0], vt[2], angles='xy', scale_units='xy', scale=1, color='g', label='T velocity vector')
    
    a1 = robot.dh_params[2]['a']
    a2 = robot.dh_params[3]['a']
    m1 = robot.inertial_properties[1]['mass']
    m2 = robot.inertial_properties[2]['mass']
    m0 = robot.inertial_properties[0]['mass']
    u = np.squeeze(active_compliance_control(x_d, x, jacobian, Kp, Kd, q, qd, a1, a2, m0, m1, m2, q_d, T_d)).T
    # hip torque should be negative, knee torque should be positive
    print("u: ", u)


    plt.legend()
    plt.grid()
    plt.show()
