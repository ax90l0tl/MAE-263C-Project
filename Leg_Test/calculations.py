# Deriving the gravity compensation equation for learning purposes

from math import sin, cos, pi, radians, degrees
import numpy as np
import sympy as sp
from sympy import lambdify

# # Verify that the FK up to the CoM loctions are correct:
# l1 = 0.125
# lc1 = 0.0613
# lc2 = 0.11
# m1 = 0.478
# m2 = 0.0633
# q1, q2 = -90, 90
# th = radians(77.52)
# q1 = radians(q1 - 90)
# q2 = radians(q2)
# pc1 = np.array([
#     lc1 * cos(q1 + th),
#     lc1 * sin(q1 + th)
# ])
# pc2 = np.array([
#     l1 * cos(q1) + lc2 * cos(q1 + q2), 
#     l1 * sin(q1) + lc2 * sin(q1 + q2)
# ])
# print(pc1, pc2)

# # Use sympy to calculate jacobians of CoM
# q1, q2, l1, lc1, lc2, g, m1, m2, th = sp.symbols('q1 q2 l1 lc1 lc2 g m1 m2 th')
# pc1 = sp.Matrix([
#     lc1 * sp.cos(q1 + th),
#     lc1 * sp.sin(q1 + th)
# ])
# pc2 = sp.Matrix([
#     l1 * sp.cos(q1) + lc2 * sp.cos(q1 + q2), 
#     l1 * sp.sin(q1) + lc2 * sp.sin(q1 + q2)
# ])

# q = sp.Matrix([q1, q2])
# Jc1 = pc1.jacobian(q)
# Jc2 = pc2.jacobian(q)
# print("Jc1: ")
# sp.pprint(Jc1)
# print("Jc2: ")
# sp.pprint(Jc2)

# g0 = sp.Matrix([0, -g])
# print("Tau: ")
# Tau = Jc1.T * m1 * g0 + Jc2.T * m2 * g0
# sp.pprint(Tau)

# # Plug in some numbers to verify
# f = lambdify((q1, q2, th, l1, lc1, lc2), Tau, modules='numpy')
# l1 = 0.125
# l2 = 0.215
# lc1 = 0.0613
# lc2 = 0.11
# m0 = 0.649
# m1 = 0.478
# m2 = 0.0633
# q1, q2 = -56.14, 83.99
# th = 77.52
# result = f(radians(q1-90), radians(q2), radians(th), l1, lc1, lc2)
# print(result)

# # Final equations for gravity compensation. Arbitrary numbers are used to check answers.
# m1, m2, g = sp.symbols('m1 m2 g')
# l1 = 0.125
# lc1 = 0.0613
# lc2 = 0.11
# q1, q2 = -56.14, 83.99
# th = 77.52
# q1, q2, th = radians(q1), radians(q2), radians(th)
# Tau = -np.array([
#     m1 * g * lc1 * cos(q1 + th) + m2 * g * (l1 * cos(q1) + lc2 * cos(q1 + q2)),
#     m2 * g * lc2 * cos(q1 + q2)
# ])
# sp.pprint(Tau)


# FK calculations
l1 = 125
l2 = 215

q1 = -45   # WRT to the +x axis, CCW positive.
q2 = 90   # WRT to the extension of link 1, CCW positive.

q1 = (q1 - 90) * pi / 180  # Converting q1 to the angle WRT to the +x axis, CCW positive.
q2 = q2 * pi / 180 

# When base is at joint 1
pe = np.array([l2*cos(q1 + q2) + l1*cos(q1), 
               l2*sin(q1 + q2) + l1*sin(q1)])
print("Foot contact WRT joint 1", pe)

# When base is at foot contact
q1p = q2 + (q1 + pi)
q2p = - q2
print(f"q1p = {degrees(q1p)}, q2p = {degrees(q2p)}")
pe = np.array([l1*cos(q1p + q2p) + l2*cos(q1p), 
               l1*sin(q1p + q2p) + l2*sin(q1p)])
print("Joint 1 WRT foot contact", pe)
