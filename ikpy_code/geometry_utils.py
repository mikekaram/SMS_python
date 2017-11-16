# coding= utf8
"""
.. module:: geometry_utils
This module contains helper functions used to compute 3D geometric transformations.
"""
import numpy as np
import sympy


def Rx_matrix(theta):
    """Rotation matrix around the X axis"""
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])


def Rz_matrix(theta):
    """Rotation matrix around the Z axis"""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])


def symbolic_Rz_matrix(symbolic_theta):
    """Matrice symbolique de rotation autour de l'axe Z"""
    return sympy.Matrix([
        [sympy.cos(symbolic_theta), -sympy.sin(symbolic_theta), 0],
        [sympy.sin(symbolic_theta), sympy.cos(symbolic_theta), 0],
        [0, 0, 1]
    ])


def Ry_matrix(theta):
    """Rotation matrix around the Y axis"""
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])


def rotation_matrix(phi, theta, psi):
    """Retourne la matrice de rotation décrite par les angles d'Euler donnés en paramètres"""
    return np.dot(Rz_matrix(phi), np.dot(Rx_matrix(theta), Rz_matrix(psi)))


def symbolic_rotation_matrix(phi, theta, symbolic_psi):
    """Retourne une matrice de rotation où psi est symbolique"""
    return sympy.Matrix(Rz_matrix(phi)) * sympy.Matrix(Rx_matrix(theta)) * symbolic_Rz_matrix(symbolic_psi)


def angles_from_rotation_matrix(R):
    if(R[0, 0] == 0 and R[1, 0] == 0):
        b = np.pi / 2
        a = 0
        c = np.arctan2(R[0, 1], R[1, 1])
    else:
        b = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
        a = np.arctan2(R[1, 0], R[0, 0])
        c = np.arctan2(R[2, 1], R[2, 2])

    return [a, b, c]


def ticks_to_angle(ticks, resolution_bits):
    if not ticks:
        return []
    else:
        ticks = np.array(ticks).astype(np.float64)
        angle = ticks * 2 * np.pi / (2**resolution_bits)
        # print(angle)
        return angle


def angle_to_ticks(angle, resolution_bits):
    ticks = angle / (2 * np.pi) * 2**resolution_bits
    return ticks


def angle_difference(target_angle, source_angle):
    a = target_angle - source_angle
    # print(a)
    a = np.mod(a + np.pi, 2 * np.pi) - np.pi
    return a


def angle_difference_with_limits(target_angle, source_angle, angle_limits_clockwise):
    if len(angle_limits_clockwise) == 0:
        return angle_difference(target_angle, source_angle)
    else:
        return -np.sign(angle_difference(target_angle, source_angle)) * (2 * np.pi - abs(angle_difference(target_angle, source_angle))) if ((angle_difference(target_angle, source_angle) >= 0 and angle_difference(angle_limits_clockwise[0], source_angle) >= 0 and angle_difference(target_angle, angle_limits_clockwise[0]) >= 0) or (angle_difference(target_angle, source_angle) < 0 and angle_difference(angle_limits_clockwise[1], source_angle) < 0 and angle_difference(target_angle, angle_limits_clockwise[1]) < 0)) else angle_difference(target_angle, source_angle)


def wrap_to_pi(theta):
    return np.mod(theta + np.pi, 2 * np.pi) - np.pi


def quaternion_from_rotation_matrix(R):
    ita = .5 * np.sqrt(R[0, 0] + R[1, 1] + R[2, 2] + 1) if (R[0, 0] + R[1, 1] + R[2, 2] + 1 > 0) else -.5 * np.sqrt(abs(R[0, 0] + R[1, 1] + R[2, 2] + 1))
    # print(R[0, 0] - R[1, 1] - R[2, 2] + 1)
    # ex = .5 * np.sign(R[2, 1] - R[1, 2]) * np.sqrt(R[0, 0] - R[1, 1] - R[2, 2] + 1) if (R[0, 0] - R[1, 1] - R[2, 2] + 1 > 0) else -.5 * np.sign(R[2, 1] - R[1, 2]) * np.sqrt(abs(R[0, 0] - R[1, 1] - R[2, 2] + 1))
    # ey = .5 * np.sign(R[0, 2] - R[2, 0]) * np.sqrt(R[1, 1] - R[2, 2] - R[0, 0] + 1) if (R[1, 1] - R[2, 2] - R[0, 0] + 1 > 0) else -.5 * np.sign(R[0, 2] - R[2, 0]) * np.sqrt(abs(R[1, 1] - R[2, 2] - R[0, 0] + 1))
    # ez = .5 * np.sign(R[1, 0] - R[0, 1]) * np.sqrt(R[2, 2] - R[0, 0] - R[1, 1] + 1) if (R[2, 2] - R[0, 0] - R[1, 1] + 1 > 0) else -.5 * np.sign(R[1, 0] - R[0, 1]) * np.sqrt(abs(R[2, 2] - R[0, 0] - R[1, 1] + 1))
    ex = .5 * np.sign(R[2, 1] - R[1, 2]) * np.sqrt(R[0, 0] - R[1, 1] - R[2, 2] + 1)
    ey = .5 * np.sign(R[0, 2] - R[2, 0]) * np.sqrt(R[1, 1] - R[2, 2] - R[0, 0] + 1)
    ez = .5 * np.sign(R[1, 0] - R[0, 1]) * np.sqrt(R[2, 2] - R[0, 0] - R[1, 1] + 1)
    # print(ex, ey, ez)
    return ita, np.array([ex, ey, ez])


def quaternion_from_angle_axis(theta, r):
    ita = np.cos(theta / 2)
    e = r * np.sin(theta / 2)
    return ita, e


def angle_axis_from_quaternion(ita, e):
    theta = 2 * np.arccos(ita)
    r = e / np.sin(theta / 2)
    return theta, r


def angle_axis_from_rotation_matrix(R):
    theta = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)
    r = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(theta))
    return theta, r


def rotation_matrix_from_angle_axis(theta, rx, ry, rz):
    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    R = np.array([[rx**2 * (1 - ctheta) + ctheta, rx * ry * (1 - ctheta) - rz * stheta, rx * rz * (1 - ctheta) + ry * stheta], [rx * ry * (1 - ctheta) + rz * stheta, ry**2 * (1 - ctheta) + ctheta, ry * rz * (1 - ctheta) - rx * stheta], [rx * rz * (1 - ctheta) - ry * stheta, ry * rz * (1 - ctheta) + rx * stheta, rz**2 * (1 - ctheta) + ctheta]])
    return R


def rotation_matrix_from_quaternion(ita, ex, ey, ez):
    R = np.array([[2 * (ita**2 + ex**2) - 1, 2 * (ex * ey - ita * ez), 2 * (ex * ez + ita * ey)], [2 * (ex * ey + ita * ez), 2 * (ita**2 + ey**2) - 1, 2 * (ey * ez - ita * ex)], [2 * (ex * ez - ita * ey), 2 * (ey * ez + ita * ex), 2 * (ita**2 + ez**2) - 1]])
    return R


def rpy_matrix(roll, pitch, yaw):
    """Returns a rotation matrix described by the extrinsinc roll, pitch, yaw coordinates"""
    return np.dot(Rz_matrix(yaw), np.dot(Ry_matrix(pitch), Rx_matrix(roll)))


def axis_rotation_matrix(axis, theta):
    """Returns a rotation matrix around the given axis"""
    [x, y, z] = axis
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [x**2 + (1 - x**2) * c, x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [x * y * (1 - c) + z * s, y ** 2 + (1 - y**2) * c, y * z * (1 - c) - x * s],
        [x * z * (1 - c) - y * s, y * z * (1 - c) + x * s, z**2 + (1 - z**2) * c]
    ])


def is_rotation_matrix(R):
    epsilon = 0.001
    if abs(np.linalg.det(R) - 1) < epsilon and np.dot(R[:, 0], R[:, 1]) < epsilon and np.dot(R[:, 1], R[:, 2]) < epsilon:
        return True
    else:
        return False


def symbolic_axis_rotation_matrix(axis, symbolic_theta):
    """Returns a rotation matrix around the given axis"""
    [x, y, z] = axis
    c = sympy.cos(symbolic_theta)
    s = sympy.sin(symbolic_theta)
    return sympy.Matrix([
        [x**2 + (1 - x**2) * c, x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [x * y * (1 - c) + z * s, y ** 2 + (1 - y**2) * c, y * z * (1 - c) - x * s],
        [x * z * (1 - c) - y * s, y * z * (1 - c) + x * s, z**2 + (1 - z**2) * c]
    ])


def homogeneous_translation_matrix(trans_x, trans_y, trans_z):
    """Returns a translation matrix the homogeneous space"""
    return np.array([[1, 0, 0, trans_x], [0, 1, 0, trans_y], [0, 0, 1, trans_z], [0, 0, 0, 1]])


def from_transformation_matrix(transformation_matrix):
    """Converts a transformation matrix to a tuple (translation_vector, rotation_matrix)"""
    return (transformation_matrix[:, -1], transformation_matrix[:-1, :-1])


def to_transformation_matrix(translation, orientation_matrix=np.zeros((3, 3))):
    """Converts a tuple (translation_vector, orientation_matrix)  to a transformation matrix

    :param np.array translation: The translation of your frame presented as a 3D vector.
    :param np.array orientation_matrix: Optional : The orientation of your frame, presented as a 3x3 matrix.
    """
    matrix = np.eye(4)

    matrix[:-1, :-1] = orientation_matrix
    matrix[:-1, -1] = translation
    return matrix


def cartesian_to_homogeneous(cartesian_matrix, matrix_type="numpy"):
    """Converts a cartesian matrix to an homogenous matrix"""
    dimension_x, dimension_y = cartesian_matrix.shape
    # Square matrix
    # Manage different types fo input matrixes
    if matrix_type == "numpy":
        homogeneous_matrix = np.eye(dimension_x + 1)
    elif matrix_type == "sympy":
        homogeneous_matrix = sympy.eye(dimension_x + 1)
    # Add a column filled with 0 and finishing with 1 to the cartesian matrix to transform it into an homogeneous one
    homogeneous_matrix[:-1, :-1] = cartesian_matrix

    return homogeneous_matrix


def cartesian_to_homogeneous_vectors(cartesian_vector, matrix_type="numpy"):
    """Converts a cartesian vector to an homogenous vector"""
    dimension_x = cartesian_vector.shape[0]
    # Vector
    if matrix_type == "numpy":
        homogeneous_vector = np.zeros(dimension_x + 1)
        # Last item is a 1
        homogeneous_vector[-1] = 1
        homogeneous_vector[:-1] = cartesian_vector
    return homogeneous_vector


def homogeneous_to_cartesian_vectors(homogeneous_vector):
    """Converts a cartesian vector to an homogenous vector"""
    return homogeneous_vector[:-1]


def homogeneous_to_cartesian(homogeneous_matrix):
    """Converts a cartesian vector to an homogenous matrix"""
    # Remove the last column
    return homogeneous_matrix[:-1, :-1]
