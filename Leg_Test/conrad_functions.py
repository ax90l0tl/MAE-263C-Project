import numpy as np

def function_G(q, g, m1, m2,l1x,l1y,l2x,l2y,l1,l2):

    q1 = q[0]
    q2 = q[1]
    q12 = q1 + q2
    sin12 = np.sin(q12)
    cos12 = np.cos(q12)
    sin1 = np.sin(q1)
    cos1 = np.cos(q1)



    G = np.zeros((2, 1))

    G[0, 0] = g*(m2*l2y*cos12 + m2*l2x*sin12 + m1*l1y*cos1 + m2*l2*sin1 + m1*l1x*sin1)
    G[1, 0] = m2*g*(l2y*cos12 + l2x*sin12)

    return G

def function_J14(q, L1, L2):
    q1, q2 = q[0], q[1]

    q1_shift = q1 - np.pi / 2
    sin_q1s = np.sin(q1_shift)
    cos_q1s = np.cos(q1_shift)
    sin_q2 = np.sin(q2)
    cos_q2 = np.cos(q2)

    common = L2 * (cos_q2 * sin_q1s + cos_q1s * sin_q2)

    J14 = np.zeros((2, 3))
    J14[0, 1] = -common - L1 * sin_q1s
    J14[0, 2] = -common
    J14[1, 1] = L2 * (cos_q2 * cos_q1s - sin_q2 * sin_q1s) + L1 * cos_q1s
    J14[1, 2] = L2 * (cos_q2 * cos_q1s - sin_q2 * sin_q1s)

    return J14

def function_p14(q, L):

    q1, q2 = q[0], q[1]
    L1, L2 = L[0], L[1]

    q1_shift = q1 - np.pi / 2
    sin_q1s = np.sin(q1_shift)
    cos_q1s = np.cos(q1_shift)
    sin_q2 = np.sin(q2)
    cos_q2 = np.cos(q2)

    p14 = np.zeros((2, 1))
    p14[0, 0] = L2 * (cos_q2 * cos_q1s - sin_q2 * sin_q1s) + L1 * cos_q1s
    p14[1, 0] = L2 * (cos_q2 * sin_q1s + sin_q2 * cos_q1s) + L1 * sin_q1s

    return p14