import numpy as np


def body_to_inertial(phi, theta, psi):
    """
    Convert from body frame to inertial frame using 3-2-1 Euler angles.
    
    Args:
        phi: Roll angle (rotation around x-axis)
        theta: Pitch angle (rotation around y-axis) 
        psi: Yaw angle (rotation around z-axis)
    
    Returns:
        R: 3x3 rotation matrix to express a vector in the body frame as a vector in the inertial frame
    """
    # Pre-compute trig functions
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)

    # Build rotation matrix
    R = np.array([
        [c_theta*c_psi, s_phi*s_theta*c_psi - c_phi*s_psi, c_phi*s_theta*c_psi + s_phi*s_psi],
        [c_theta*s_psi, s_phi*s_theta*s_psi + c_phi*c_psi, c_phi*s_theta*s_psi - s_phi*c_psi],
        [-s_theta, s_phi*c_theta, c_phi*c_theta]
    ])
    
    return R

def inertial_to_body(phi, theta, psi):
    """
    Convert from inertial frame to body frame using 3-2-1 Euler angles.
    """
    return body_to_inertial(phi, theta, psi).T

def euler_rates(phi, theta, psi):
    R = np.array([
        [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
    ])

    return R