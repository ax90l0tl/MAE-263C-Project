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
        x = np.linspace(t[i], t[i+1], n)
        
        q = a0 + a1*x + a2*x**2 + a3*x**3 + a4*x**4 + a5*x**5
        qd = a1 + 2*a2*x + 3*a3*x**2 + 4*a4*x**3 + 5*a5*x**4
        qdd = 2*a2 + 6*a3*x + 12*a4*x**2 + 20*a5*x**3

        t_points.append(x)
        q_points.append(q)
        qd_points.append(qd)
        qdd_points.append(qdd)
    
    t_points = np.concatenate(t_points)
    q_points = np.concatenate(q_points)
    qd_points = np.concatenate(qd_points)
    qdd_points = np.concatenate(qdd_points)
    
    points = np.vstack((t_points, q_points, qd_points, qdd_points))
    return np.array(points)

if __name__ == "__main__":
    t = [0, 1]
    q = [-0.1, -0.5]
    qd = [0, 0]
    qdd = [0, 0]
    a = quintic_interp(t, q, qd, qdd)
    print(a.shape)
    plt.scatter(t, q, label='q_desired')
    plt.plot(a[0, :], a[1, :], label='q')
    plt.plot(a[0, :], a[2, :], label='qd')
    plt.plot(a[0, :], a[3, :], label='qdd')
    plt.legend()
    # plt.show()
    q2 = [0, 0]
    b = quintic_interp(t, q2, qd, qdd)