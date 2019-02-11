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

def Quaternion2Rotation(quat):
    e0 = quat[0]
    e1 = quat[1]
    e2 = quat[2]
    e3 = quat[3]

    R = np.array([[e1**2 + e0**2 - e2**2 -e3**2, 2*(e1*e2 - e3*e0), 2*(e1*e3 + e2*e0)],
                  [2*(e1*e2 + e3*e0), e2**2 + e0**2 - e1**2 - e3**2, 2*(e2*e3 - e1*e0)],
                  [2*(e1*e3 - e2*e0), 2*(e2*e3 + e1*e0), e3**2 + e0**2 - e1**2 - e2**2]])

    return R

def Euler2Rotation(phi, theta, psi):
    """
    Converts euler angles to rotation matrix (R_b^i, i.e., body to inertial)
    """
    # only call sin and cos once for each angle to speed up rendering
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)

    R_roll = np.array([[1, 0, 0],
                       [0, c_phi, s_phi],
                       [0, -s_phi, c_phi]])
    R_pitch = np.array([[c_theta, 0, -s_theta],
                        [0, 1, 0],
                        [s_theta, 0, c_theta]])
    R_yaw = np.array([[c_psi, s_psi, 0],
                      [-s_psi, c_psi, 0],
                      [0, 0, 1]])

    R = R_roll @ R_pitch @ R_yaw  # inertial to body (Equation 2.4 in book)
    return R.T  # transpose to return body to inertial
