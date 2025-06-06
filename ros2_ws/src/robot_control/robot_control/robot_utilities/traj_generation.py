import numpy as np
import matplotlib.pyplot as plt
from robot_control.robot_utilities.robot_kinematics import Robot


def quintic_interp(t, q, qd, qdd, n=200):
    t_points = []
    q_points = []
    qd_points = []
    qdd_points = []
    for i in range(0, len(t)-1):
        a0 = q[i]
        a1 = qd[i]
        a2 = qdd[i] / 2
        a3 = (20 * (q[i+1] - q[i]) - (8 * qd[i+1] + 12 * qd[i]) * (t[i+1] - t[i]) - (3*qdd[i] - qdd[i+1])*((t[i+1] - t[i])**2)) / (2*((t[i+1] - t[i])**3))
        a4 = (30 * (q[i] - q[i+1]) + (14 * qd[i+1] + 16 * qd[i]) * (t[i+1] - t[i]) + (3*qdd[i] - 2*qdd[i+1])*((t[i+1] - t[i])**2)) / (2*((t[i+1] - t[i])**4))
        a5 = (12 * (q[i+1] - q[i]) + (6 * qd[i+1] + 6 * qd[i]) * (t[i+1] - t[i]) + (qdd[i] - qdd[i+1])*((t[i+1] - t[i])**2)) / (2*((t[i+1] - t[i])**5))
        x = np.linspace(t[i], t[i+1], n)[:, np.newaxis]
        
        q = a0 + a1*x + a2*(x**2) + a3*(x**3) + a4*(x**4) + a5*(x**5)
        qd = a1 + 2*a2*x + 3*a3*(x**2) + 4*a4*(x**3) + 5*a5*(x**4)
        qdd = 2*a2 + 6*a3*x + 12*a4*(x**2) + 20*a5*(x**3)
        t_points.append(x)
        q_points.append(q)
        qd_points.append(qd)
        qdd_points.append(qdd)
    
    t_points = np.concatenate(t_points)
    q_points = np.concatenate(q_points)
    qd_points = np.concatenate(qd_points)
    qdd_points = np.concatenate(qdd_points)
    
    return (t_points, q_points, qd_points, qdd_points)

if __name__ == "__main__":
    robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
    t = [0, 0.35]
    q_0 = robot.inverse_kinematics(np.array([0.0, -0.3]))[0]
    q_1 = robot.inverse_kinematics(np.array([0.0, -0.115]))[0]
    q_d = np.vstack((q_0, q_1))
    print(q_d)
    qd = np.zeros((2, 2))
    qdd = np.zeros((2, 2))
    

    (t_points, q_points, qd_points, qdd_points) = quintic_interp(t, q_d, qd, qdd, n=100)
    print(t_points.shape, q_points.shape, qd_points.shape, qdd_points.shape)
    # plt.figure()
    # plt.scatter(t, q_d[:, 0], label='q_desired_x')
    # plt.plot(t_points, q_points[:, 0], label='q')
    # plt.plot(t_points, qd_points[:, 0], label='qd')
    # plt.plot(t_points, qdd_points[:, 0], label='qdd')
    # plt.legend()

    # plt.figure()
    # plt.scatter(t, q_d[:, 1], label='q_desired_z')
    # plt.plot(t_points, q_points[:, 1], label='q')
    # plt.plot(t_points, qd_points[:, 1], label='qd')
    # plt.plot(t_points, qdd_points[:, 1], label='qdd')
    # plt.legend()

    # fig, ax = plt.subplots()
    # for i in range(len(q_points)):
    #     if i % 10 == 0:
    #         robot.plot_robot_arm(q_points[i], q_points[i], fig=fig, ax=ax)


    x_0 = np.array([0.0, -0.3])
    x_1 = np.array([0.0, -0.115])
    x_d = np.vstack((x_0, x_1))
    xd = np.zeros((2, 2))
    xdd = np.zeros((2, 2))
    

    (t_points, x_points, xd_points, xdd_points) = quintic_interp(t, x_d, xd, xdd, n=100)
    q_points = []
    qd_points = []
    qdd_points = []
    for i in range(len(t_points)):
        q = robot.inverse_kinematics(x_points[i])[0]
        jacobian = robot.jacobian_square(q)
        qd = np.linalg.inv(jacobian) @ np.atleast_2d(np.atleast_2d(xd_points[i]).T)
        jacobiand = robot.jacobian_derivative(q, qd)[[0, 2]]
        qdd = np.linalg.inv(jacobian) @ (np.atleast_2d(xdd_points[i]).T - jacobiand @ qd)

        q_points.append(q)
        qdd_points.append(qdd)
        qd_points.append(qd)
        # if i % 10 == 0:
            # robot.plot_robot_arm(q, x_points[i], fig=fig, ax=ax)
    
    
    qdd_points = np.squeeze(np.array(qdd_points))
    qd_points = np.squeeze(np.array(qd_points))
    q_points = np.array(q_points)

    print(t_points.shape, q_points.shape, qd_points.shape, qdd_points.shape)
    plt.figure()
    plt.plot(t_points, q_points[:, 0], label='J1')
    plt.plot(t_points, qd_points[:, 0], label='J1d')
    plt.plot(t_points, qdd_points[:, 0], label='J1dd')
    plt.legend()

    plt.figure()
    plt.plot(t_points, q_points[:, 1], label='J2')
    plt.plot(t_points, qd_points[:, 1], label='J2d')
    plt.plot(t_points, qdd_points[:, 1], label='J2dd')
    plt.legend()
    

    plt.show()
    q2 = [0, 0]
    b = quintic_interp(t, q2, qd, qdd)