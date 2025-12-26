import numpy as np
from scipy.spatial.transform import Rotation

def orientation_error(desired, current):
    """
    This function calculates a 3-dimensional orientation error vector for use in the
    impedance controller. It does this by computing the delta rotation between the
    inputs and converting that rotation to exponential coordinates (axis-angle
    representation, where the 3d vector is axis * angle).
    See https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation for more information.
    Optimized function to determine orientation error from matrices

    Args:
        desired (np.array): 2d array representing target orientation matrix
        current (np.array): 2d array representing current orientation matrix

    Returns:
        np.array: 2d array representing orientation error as a matrix
    """
    rc1 = current[0:3, 0]
    rc2 = current[0:3, 1]
    rc3 = current[0:3, 2]
    rd1 = desired[0:3, 0]
    rd2 = desired[0:3, 1]
    rd3 = desired[0:3, 2]

    error = 0.5 * (np.cross(rc1, rd1) + np.cross(rc2, rd2) + np.cross(rc3, rd3))

    return error

def pose_error(target_pos, target_quat, ee_pos, ee_quat) -> np.ndarray:
    """
    Calculate the rotational error (orientation difference) between the target and current orientation.

    Parameters:
        target_ori_mat (numpy.ndarray): The target orientation matrix.
        current_ori_mat (numpy.ndarray): The current orientation matrix.

    Returns:
        numpy.ndarray: The rotational error in axis-angle representation.
    """
    err_pos = target_pos - ee_pos
    err_ori = orientation_error(quat2mat(target_quat), quat2mat(ee_quat))

    return np.concatenate([err_pos, err_ori])

def quat2mat(quaternion):
    # Ensure the quaternion is normalized
    quaternion /= np.linalg.norm(quaternion)
    
    # Create a rotation object from the quaternion
    rotation = Rotation.from_quat(quaternion)
    
    # Convert the rotation to a 3x3 rotation matrix
    rotation_matrix = rotation.as_matrix()
    
    return rotation_matrix