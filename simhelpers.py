import numpy as np

def quatderiv(quat, omega):
    quat = np.asarray(quat)
    omega = np.asarray(omega)
    quat_norm = np.linalg.norm(quat)
    quat_normalized = quat/quat_norm
    quat_correction = .01*(quat_normalized-quat)
    P = omega[0]
    Q = omega[1]
    R = omega[2]
    return np.asarray(0.5*np.matrix([[ 0, -P, -Q, -R],
                                     [ P,  0,  R, -Q],
                                     [ Q, -R,  0,  P],
                                     [ R,  Q, -P,  0]]) * np.matrix(quat_normalized).T).flatten() + quat_correction
