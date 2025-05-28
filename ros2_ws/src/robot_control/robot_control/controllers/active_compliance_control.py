import numpy as np
from robot_control.robot_utilities.robot_kinematics import Robot
from scipy import constants

def active_compliance_control(x_d, x_e, Jacobian, Kp, Kd, q, qd, a1, a2, m1, m2, q_d, T_d):
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
    qd = np.atleast_2d(qd.copy()).T
    # COM in urdf is defined at the base of the link TODO need to change
    g = np.zeros((2, 1))
    g[0] = -a1*constants.g*m2*np.sin(q[0])

    R_d = T_d[0:3, 0:3]
    T = np.vstack((np.hstack((R_d, np.zeros((3, 3)))), np.hstack((np.zeros((3, 3)), R_d))))

    J_a_d = Jacobian[[0, 2]]
    # J_a_d = T @ Jacobian
    # J_a_d = J_a_d[[0, 2]]
    # print("J_a_d: ", J_a_d)
    # Compute the control output
    control_output = np.zeros((2, 1))
    control_output = g + J_a_d.T @ (Kp @ x_error - Kd @ (J_a_d @ qd))
    
    return control_output

if __name__ == "__main__":
    robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
    q = np.array([-1.5707963267962646,  3.0415926535912727])
    x = np.array([0.0, -0.55])
    x_d = np.array([0.0, -0.4])
    Kp = np.diag([5000.0, 100.0])
    Kd = np.diag([500.0, 10.0])
    qd = np.array([-0.001, 0.1])
    q_d = robot.inverse_kinematics(x_d)[0]
    T_d = robot.forward_kinematics(q_d)
    jacobian =robot.jacobian(q)
    a1 = robot.dh_params[2]['a']
    a2 = robot.dh_params[3]['a']
    m1 = robot.inertial_properties[1]['mass']
    m2 = robot.inertial_properties[2]['mass']
    u = np.squeeze(active_compliance_control(x_d, x, jacobian, Kp, Kd, q, qd, a1, a2, m1, m2, q_d, T_d)).T
    # hip torque should be negative, knee torque should be positive
    print("u: ", u)