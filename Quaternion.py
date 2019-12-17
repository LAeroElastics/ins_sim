import numpy as np


class Quaternion(object):
    def __init__(self, q=np.zeros((4, 1))):
        self.__value = q

    def getX(self):
        return float(self.__value[0])

    def getY(self):
        return float(self.__value[1])

    def getZ(self):
        return float(self.__value[2])

    def getW(self):
        return float(self.__value[3])

    def __neg__(self):
        return Quaternion(-self.__value)

    def __add__(self, q):
        q = Quaternion(q)
        return Quaternion(self.__value + q.__value)

    def __radd__(self, q):
        return self + q

    def __sub__(self, q):
        q = Quaternion(q)
        return Quaternion(self.__value - q.__value)

    def __rsub__(self, q):
        return self - q

    def __mul__(self, q):
        pass

    def __rmul__(self, scalar):
        pass

    def __abs__(self):
        return np.sqrt(self.getX() ** 2 + self.getY() ** 2 + self.getZ() ** 2 + self.getW() ** 2)

    def __eq__(self, q):
        q = Quaternion(q)
        return self.__value == q.__value

    def toArray(self):
        return self.__value

    def regulerize(self):
        self.__value /= np.linalg.norm(self.__value)

    def conj(self):
        return Quaternion(np.array([-self.getX(), -self.getY(), -self.getZ(), self.getW()]))

    def toMatrix(self):
        q0 = self.getX()
        q1 = self.getY()
        q2 = self.getZ()
        w = self.getW()

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

