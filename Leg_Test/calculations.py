# Deriving the gravity compensation equation for learning purposes

from math import sin, cos, pi
import numpy as np
import sympy as sp
from sympy import lambdify

# l1 = 20
# l2 = 20
# lc1 = 10
# lc2 = 10
# m1 = 1
# m2 = 1
# g = 9.81

# q1 = -90
# q2 = 90

# q1 = q1 * pi / 180
# q2 = q2 * pi / 180

# pc1 = np.array([ lc1 * sin(q1), 
#                 -lc1 * cos(q1)])
# pc2 = np.array([ l1 * sin(q1) - lc2 * sin(q1 + q2), 
#                 -l1 * cos(q1) + lc2 * cos(q1 + q2)])

# # Use sympy to calculate jacobians of CoM
# q1, q2, l1, lc1, lc2, g, m1, m2 = sp.symbols('q1 q2 l1 lc1 lc2 g m1 m2')
# pc1 = sp.Matrix([
#     lc1 * sp.sin(q1),
#    -lc1 * sp.cos(q1)
# ])
# pc2 = sp.Matrix([
#     l1 * sp.sin(q1) + lc2 * sp.sin(q1 + q2), 
#    -l1 * sp.cos(q1) - lc2 * sp.cos(q1 + q2)
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
# f = lambdify((q1, q2, l1, lc1, lc2), Tau, modules='numpy')
# result = f(-40*pi/180, 80*pi/180, 20, 10, 10)
# print(result)

# Final equations for gravity compensation. Arbitrary numbers are used to check answers.
m1, m2, g = sp.symbols('m1 m2 g')
l1 = 20
lc1 = 10
lc2 = 10
q1 = -77 * pi / 180
q2 = 110 * pi / 180
Tau = -np.array([
    m1 * g * lc1 * sin(q1) + m2 * g * (l1 * sin(q1) + lc2 * sin(q1 + q2)),
    m2 * g * lc2 * sin(q1 + q2)
])
sp.pprint(Tau)
