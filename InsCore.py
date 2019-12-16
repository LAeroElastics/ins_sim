import numpy as np

import Earth as earth
import Quaternion as q
import utils

_INS_PI = 3.1415926535897932384626433832795

_DIM_ACC = (3, 1)
_DIM_GYRO = (3, 1)
_DIM_VEC3 = (3, 1)
_DIM_MAT33 = (3, 3)
_DIM_MAT44 = (4, 4)

_X, _Y, _Z, _W = range(4)


class InsCore(object):
    def __init__(self):
        self.a_b = np.zeros(_DIM_ACC)
        self.omega_n_b = np.zeros(_DIM_GYRO)

        self.phi = 0.0
        self.lam = 0.0
        self.h = 0.0

        self.v_N = 0.0
        self.v_E = 0.0
        self.v_D = 0.0

        self.q_b_n = q.Quaternion()
        self.C_b_n = np.zeros(_DIM_MAT33)
        self.C_e_n = np.zeros(_DIM_MAT33)

        self.omega_i2n_4n = np.zeros(_DIM_VEC3)
        self.omega_i2b_4b = np.zeros(_DIM_VEC3)
        self.omega_i2e_4i = np.zeros(_DIM_VEC3)
        self.omega_i2e_4i[2] = earth.omega_e
        self.omega_i2e_4n = np.zeros(_DIM_VEC3)

        self.omega_n2b_4b = np.zeros(_DIM_VEC3)
        self.omega_e2n_4n = np.zeros(_DIM_VEC3)

    def initPosition(self, phi, lam, h):
        self.phi, self.lam, self.h = phi, lam, h

    """
    def initAttitude(self, yaw, roll, pitch):
        self._euler2quat(yaw, roll, pitch)
        self._euler2matrix(yaw, roll, pitch)
    """

    def initAttitude(self, quaternion):
        self.q_b_n = quaternion
        print(self.q_b_n.getW())
        self.C_b_n = utils.quat2cbnMat(self.q_b_n)
        self.C_e_n = utils.quat2cenMat(self.positionVector())

    def initVelocity(self, v_N, v_E, v_D):
        self.v_N, self.v_E, self.v_D = v_N, v_E, v_D

    # latitude
    def phi(self):
        return self.phi

    # longitude
    def lam(self):
        return self.lam

    # altitude
    def altitude(self):
        return self.h

    def v_N(self):
        return self.v_N

    def v_E(self):
        return self.v_E

    def v_D(self):
        return self.v_D

    def yaw(self):
        return np.arctan2(2.0 * (self.q_b_n.getX() * self.q_b_n.getY() + self.q_b_n.getZ() * self.q_b_n.getW()),
                          self.q_b_n.getX() ** 2 - self.q_b_n.getY() ** 2 - self.q_b_n.getZ() ** 2 + self.q_b_n.getW() ** 2)

    def pitch(self):
        return np.arcsin(2.0 * (self.q_b_n.getX() * self.q_b_n.getZ() - self.q_b_n.getY() * self.q_b_n.getW()))

    def roll(self):
        return np.arctan2(2.0 * (self.q_b_n.getY() * self.q_b_n.getZ() + self.q_b_n.getX() * self.q_b_n.getW()),
                          self.q_b_n.getZ() ** 2 - self.q_b_n.getX() ** 2 - self.q_b_n.getY() ** 2 + self.q_b_n.getW() ** 2)

    def positionVector(self):
        return utils.scalar2vec(self.phi, self.lam, self.h)

    def velocityVector(self):
        return utils.scalar2vec(self.v_N, self.v_E, self.v_D)

    def meridian(self):
        return earth.meridian(self.positionVector())

    def normal(self):
        return earth.normal(self.positionVector())

    def update_omega_i2n_4n(self):
        self.omega_i2n_4n[_X] = self.v_E / (self.h + self.normal()) + earth.omega_e * np.cos(self.phi)
        self.omega_i2n_4n[_Y] = -self.v_N / (self.h + self.meridian())
        self.omega_i2n_4n[_Z] = -self.v_E / (self.h + self.normal()) * np.tan(self.phi) - earth.omega_e * np.sin(
            self.phi)

    def update_omega_e2n_4n(self):
        self.omega_e2n_4n[_X] = self.v_E * np.cos(self.phi) / (self.h + self.normal()) / np.cos(self.phi)
        self.omega_e2n_4n[_Y] = -self.v_N / (self.h + self.meridian())
        self.omega_e2n_4n[_Z] = -self.v_E * np.sin(self.phi) / (self.h + self.meridian()) / np.cos(self.phi)

    def update_omega_i2e_4n(self):
        self.omega_i2e_4n[_X] = earth.omega_e * np.cos(self.phi)
        self.omega_i2e_4n[_Z] = -earth.omega_e * np.sin(self.phi)

    def centrifugal(self):
        result = np.zeros(_DIM_VEC3)
        result[_X] = (self.h + self.normal()) * np.cos(self.phi) * np.cos(self.lam)
        result[_Y] = (self.h + self.normal()) * np.cos(self.phi) * np.sin(self.lam)
        result[_Z] = (self.h + self.normal() * (1.0 - earth.eps ** 2.0)) * np.sin(self.phi)
        return result

    def deltaq_n_b(self):
        delta_q = np.zeros(_DIM_MAT44)
        delta_q[0, 3] = delta_q[1, 2] = self.omega_n2b_4b[_X]
        delta_q[1, 3] = delta_q[2, 0] = self.omega_n2b_4b[_Y]
        delta_q[0, 1] = delta_q[2, 3] = self.omega_n2b_4b[_Z]
        delta_q[2, 1] = delta_q[3, 0] = -self.omega_n2b_4b[_X]
        delta_q[0, 2] = delta_q[3, 1] = -self.omega_n2b_4b[_Y]
        delta_q[1, 0] = delta_q[3, 2] = -self.omega_n2b_4b[_Z]

        return 0.5 * delta_q @ self.q_b_n.toArray()

    def update(self, accel, gyro, deltaT):
        # attitude
        self.update_omega_i2n_4n()
        self.omega_n2b_4b = gyro - np.linalg.inv(self.C_b_n) @ self.omega_i2n_4n
        delta_n_b = self.deltaq_n_b()
        self.q_b_n += delta_n_b * deltaT
        self.q_b_n.regulerize()

        #
        self.C_b_n = utils.quat2cbnMat(self.q_b_n)

        # Update velocity
        self.update_omega_e2n_4n()
        self.update_omega_i2e_4n()

        # transform b-frame acceleration to n-frame
        delta_V_ned = self.C_b_n @ accel

        #
        tmp1 = utils.skew_mat_3_3(self.omega_e2n_4n)
        tmp1 += 2.0 * utils.skew_mat_3_3(self.omega_i2e_4n)
        delta_V_ned -= tmp1 @ self.velocityVector()

        # add gravity
        delta_V_ned += earth.gravity(self.positionVector())

        # add centrifugal force
        tmp2 = utils.skew_mat_3_3(self.omega_i2e_4i)
        delta_V_ned -= self.C_e_n @ (tmp2 @ tmp2 @ self.centrifugal())

        # Update velocity in nav-frame
        self.v_N += delta_V_ned[_X] * deltaT
        self.v_E += delta_V_ned[_Y] * deltaT
        self.v_D += delta_V_ned[_Z] * deltaT

        # Update C_e_n matrix
        delta_C_e_n = -utils.skew_mat_3_3(self.omega_e2n_4n) @ self.C_e_n
        self.C_e_n += delta_C_e_n * deltaT

        # Update position
        self.phi += self.v_N / (self.h + self.meridian()) * deltaT
        self.lam += self.v_E / (self.h + self.normal()) * deltaT
        self.h -= self.v_D * deltaT
