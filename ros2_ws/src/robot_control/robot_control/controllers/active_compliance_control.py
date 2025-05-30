import numpy as np
from robot_control.robot_utilities.robot_kinematics import Robot
from scipy import constants
import matplotlib.pyplot as plt

def active_compliance_control(x_d, x_e, Jacobian, Kp, Kd, q, qd, a1, a2, m0, m1, m2, q_d, T_d):
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
    # COM in urdf is defined at the base of the link TODO need to change
    g = np.ones((2, 1)) # * ((0.0574*np.sin(q[0]) - 0.0162*np.cos(q[0])) * m1 + m0)* constants.g
    g[0] += (0.0216*np.cos(q[0] + q[1]) - 0.1190*np.sin(q[0] + q[1]) - a1*np.sin(q[0]))*constants.g*m2 + (- 0.0574*np.cos(q[0]) - 0.0162*np.sin(q[0]))*constants.g*m1
    g[1] += constants.g*m2*(0.0216*np.cos(q[1] + q[0]) - 0.1190*np.sin(q[1] + q[0]))


    R_d = T_d[0:3, 0:3]
    T = np.vstack((np.hstack((R_d, np.zeros((3, 3)))), np.hstack((np.zeros((3, 3)), R_d))))
    J_a_d = Jacobian[[0, 2]]
    
    control_output = np.zeros((2, 1))
    control_output = J_a_d.T @ (Kp @ x_error - Kd @ (J_a_d @ qd))
    return control_output

if __name__ == "__main__":
    robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
    ax, fig = plt.subplots()

    q = np.array([-np.pi/4,  np.pi/2])
    x = robot.forward_kinematics(q)[[0, 2], -1]
    x_d = np.array([0.0, -0.3])
    Kp = np.diag([50.0, 10.0])
    Kd = np.diag([0.0, 0.0])
    qd = np.array([-0.001, 0.1])
    q_d = robot.inverse_kinematics(x_d)[0]
    T_d = robot.forward_kinematics(q_d)
    R_d = T_d[0:3, 0:3]
    print(np.round(R_d, 10))
    T = np.vstack((np.hstack((R_d, np.zeros((3, 3)))), np.hstack((np.zeros((3, 3)), R_d))))
    jacobian = robot.jacobian(q)
    v = jacobian @ np.array([[1],[0.1]])
    vt = T @ jacobian @ np.array([[1],[0]])
    print(np.round(jacobian, 10))
    print(np.round(v, 10))
    print(np.round(T @ v, 10))
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