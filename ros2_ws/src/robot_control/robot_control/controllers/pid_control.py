import numpy as np

def pid_control(q, q_d, qd, Kp, Kd, Ki, error_prev, dt):
    error = q_d - q
    u = Kp @ error + Kd @ (0 - qd) + Ki @ (error_prev + (error-error_prev*dt))
    return u, error