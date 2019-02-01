import numpy as np
from math import cos, sin

def Quaternion2Euler(quat):
    e0 = quat[0]
    e1 = quat[1]
    e2 = quat[2]
    e3 = quat[3]

    phi = np.arctan2(2*(e0*e1 + e2*e3), (e0**2 + e3**2 - e1**2 - e2**2))
    theta = np.arcsin(2*(e0*e2 - e1*e3))
    psi = np.arctan2(2*(e0*e3 + e1*e2), (e0**2 + e1**2 - e2**2 - e3**2))

    return phi, theta, psi


def Euler2Quaternion(eul):
    phi2 = eul[0]/2
    tha2 = eul[1]/2
    psi2 = eul[2]/2

    e0 = cos(psi2)*cos(tha2)*cos(phi2) + sin(psi2)*sin(tha2)*sin(phi2)
    e1 = cos(psi2)*cos(tha2)*sin(phi2) - sin(psi2)*sin(tha2)*cos(phi2)
    e2 = cos(psi2)*sin(tha2)*cos(phi2) + sin(psi2)*cos(tha2)*sin(phi2)
    e3 = sin(psi2)*cos(tha2)*cos(phi2) - cos(psi2)*sin(tha2)*sin(phi2)

    return np.array([e0, e1, e2, e3])
