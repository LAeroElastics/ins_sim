import numpy as np


def scalar2vec(v1, v2, v3):
    result = np.zeros((3,1))
    result[0] = v1
    result[1] = v2
    result[2] = v3
    return result

def quat2cbnMat(quaternion):
    q0 = quaternion.getX()
    q1 = quaternion.getY()
    q2 = quaternion.getZ()
    w = quaternion.getW()

    mat = np.zeros((3, 3))

    mat[0, 1] = q0 * q1 - q2 * w
    mat[0, 2] = q0 * q2 + q1 * w
    mat[1, 0] = q0 * q1 + q2 * w
    mat[1, 2] = q1 * q2 - q0 * w
    mat[2, 0] = q0 * q2 - q1 * w
    mat[2, 1] = q1 * q2 + q0 * w
    mat *= 2.0

    mat[0, 0] = q0 ** 2.0 - q1 ** 2.0 - q2 ** 2.0 + w ** 2.0
    mat[1, 1] = q1 ** 2.0 - q0 ** 2.0 - q2 ** 2.0 + w ** 2.0
    mat[2, 2] = q2 ** 2.0 - q0 ** 2.0 - q1 ** 2.0 + w ** 2.0

    return mat


def quat2cenMat(position):
    mat = np.zeros((3, 3))
    phi = position[0]
    lam = position[1]

    mat[0, 0] = -np.cos(lam) * np.sin(phi)
    mat[0, 1] = -np.sin(lam) * np.sin(phi)
    mat[0, 2] = np.cos(phi)
    mat[1, 0] = -np.sin(lam)
    mat[1, 1] = np.cos(lam)
    mat[1, 2] = 0.0
    mat[2, 0] = -np.cos(lam) * np.cos(phi)
    mat[2, 1] = -np.sin(lam) * np.cos(phi)
    mat[2, 2] = -np.sin(phi)

    return mat


def skew_mat_3_3(v):
    result = np.zeros((3, 3))
    result[0, 1] = -v[2]
    result[0, 2] = v[1]
    result[1, 0] = v[2]
    result[1, 2] = -v[0]
    result[2, 0] = -v[1]
    result[2, 1] = v[0]

    return result


def skew_mat_4_4(v, inverse=False):
    result = np.zeros((4, 4))
    result[0, 1] = v[2]
    result[0, 2] = -v[1]
    result[0, 3] = -v[0]
    result[1, 0] = -v[2]
    result[1, 2] = v[0]
    result[1, 3] = -v[1]
    result[2, 0] = v[1]
    result[2, 1] = -v[0]
    result[2, 3] = -v[2]
    result[3, 0] = v[0]
    result[3, 1] = v[1]
    result[3, 2] = v[2]

    if inverse:
        result[0, 3] *= -1
        result[1, 3] *= -1
        result[2, 3] *= -1
        result[3, 0] *= -1
        result[3, 1] *= -1
        result[3, 2] *= -1

    return result
