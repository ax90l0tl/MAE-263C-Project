import numpy as np

class DynamicModel:
    def __init__(self, Robot):
        self.mass = Robot.inertial_properties['mass']
        self.inertia = Robot.inertial_properties['inertia']
        self.fk = Robot.forward_kinematics
        self.jacobian = Robot.jacobian
        self.gravity = 9.81 # gravitational acceleration magnitude
    
        self.friction = Robot.friction

    def inertia_matrix(self, q):
        I = self.inertia
        L = self.lengths
        Lc = self.lengths_center
        m = self.mass
        M = np.array([I[0] + I[1] + L[0]^2*m[2] + L[1]^2*m[2] + L[0]^2*m[1] + Lc[0]^2*m[0] + Lc[1]^2*m[1] + 2*L[0]*L[1]*m[2]*cos(q[1]) + 2*L[0]*Lc[1]*m[1]*np.cos(q[1]), m[1]*Lc[1]^2 + L[0]*m[1]*np.cos(q[1])*Lc[1] + I[1] + 0.5*m[2]*(2*L[1]^2 + 2*L[0]*np.cos(q[1])*L[1])]
        [                                      m[1]*Lc[1]^2 + L[0]*m[1]*np.cos(q[1])*Lc[1] + I[1] + 0.5*m[2]*(2*L[1]^2 + 2*L[0]*np.cos(q[1])*L[1]),                                                        m[2]*L[1]^2 + m[1]*Lc[1]^2 + I[1]]).reshape(len(q),len(q))
        return M
    
    def nonlinear_vector(self, q, qd): # Includes non-linear coupling (coriolis/centripetal) and gravity terms
        L = self.lengths
        Lc = self.lengths_center
        g = self.gravity
        m = self.mass

        n = np.array([
            0.5 * m[2] * (
            2 * L[0] * L[1] * np.sin(q[1]) * qd[1]**2 +
            4 * L[0] * L[1] * np.sin(q[1]) * qd[0] * qd[1]
            )
            - m[2] * g * (L[1] * np.cos(q[0] + q[1]) + L[0] * np.cos(q[0]))
            - g * m[1] * (Lc[1] * np.cos(q[0] + q[1]) + L[0] * np.cos(q[0]))
            - Lc[0] * g * m[0] * np.cos(q[0])
            + L[0] * Lc[1] * m[1] * np.sin(q[1]) * qd[1]**2
            + 2 * L[0] * Lc[1] * m[1] * np.sin(q[1]) * qd[0] * qd[1],

            - (Lc[1] * m[1] + L[1] * m[2]) * (
            L[0] * np.sin(q[1]) * qd[0]**2 +
            g * np.cos(q[0] + q[1])
            )
        ])

        return n
    
    def gravity_vector(self, q): # Gravity Comp Only
        L = self.lengths
        Lc = self.lengths_center
        g = self.gravity
        m = self.mass

        G = np.array([
            -g*(m[2]*(L[1]*np.cos(q[0] + q[1]) + L[0]*np.cos(q[0])) + m[1]*(Lc[1]*np.cos(q[0] + q[1]) + L[0]*np.cos(q[0])) + Lc[0]*m[0]*np.cos(q[0])),
                                                                           -g*np.cos(q[0] + q[1])*(Lc[1]*m[1] + L[1]*m[2])
        ])

        return G
    
    def friction_model(self, q, qd): # NEEDS TESTING ON HARDWARE
        # Placeholder for dynamic model computation
        # This should be replaced with the actual dynamic model computation

        return np.zeros((len(q), 1))
    

    


#