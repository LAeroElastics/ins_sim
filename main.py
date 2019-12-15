import numpy as np
import InsCore as ins

import Quaternion as q

file_path = "./input/"


def to_data_id_type(value):
    return value.astype(np.int64)


def to_data_status_type(value):
    return value.astype(np.int64)


def to_data_sensed_type(value):
    return value.astype(np.float32)


def to_data_timestamp_type(value):
    pass


class Auxdata(object):
    def __init__(self, file_name, data_type=1):
        self.full_path = file_path + file_name
        self.i_data = 0
        self.data = None
        if data_type == 1:
            self.read_data_type1()
        elif data_type == 2:
            self.read_data_type2()

    def read_data_type1(self):
        data = np.genfromtxt(self.full_path, dtype=np.str, delimiter=',', skip_header=0)
        # data_id = to_data_id_type(data[:, 0])
        # data_status = to_data_status_type(data[:, 1])
        data_values = to_data_sensed_type(data[:, 0:3])

        self.data = data_values

    def read_data_type2(self):
        data = np.genfromtxt(self.full_path, dtype=np.str, delimiter=',', skip_header=0)
        data_values = to_data_sensed_type(data[:, 1:4])

        self.data = np.array([data_values])

    def next(self):
        value = self.data[self.i_data, :]
        self.i_data = self.i_data + 1
        return value


def main():
    """
    #
    # DIRTY. JUST FOR TESTING.
    #
    """
    aux_omega = Auxdata("gyro.csv")
    aux_accel = Auxdata("acc.csv")

    init_latitude = np.radians(35. + 42. / 60 + 50.818 / 3600)
    init_longitude = np.radians(139. + 45. / 60 + 48.670 / 3600)
    init_altitude = 100
    init_position = np.zeros((3, 1))
    init_position[0] = init_latitude
    init_position[1] = init_longitude
    init_position[2] = init_altitude
    rotate_radius = 100.
    rotate_period = 120.
    rotate_omega = np.pi * 2. / rotate_period
    init_velocity = np.zeros((3, 1))
    init_velocity[0] = rotate_omega * rotate_radius
    # init_attitude = np.array([0.0868903, 0, 0, 0.9962179])  # 姿勢角(roll: Φ, pitch: θ, yaw: ψ)
    init_attitude = q.Quaternion([0.0868902910275923, 0.0, 0.0, 0.9962178864711978])
    nav = ins.InsCore()
    nav.initPosition(init_latitude, init_longitude, init_altitude)
    rotate_radius = 100.
    rotate_period = 120.
    rotate_omega = np.pi * 2. / rotate_period
    nav.initVelocity(rotate_omega * rotate_radius, 0.0, 0.0)
    nav.initAttitude(init_attitude)
    # ins.initialize()
    for i in range(29999):
        accel = aux_accel.next().reshape(3, 1)
        omega = aux_omega.next().reshape(3, 1)
        nav.update(accel, omega, 1. / 50.)
        print(np.degrees([nav.yaw(), nav.pitch(), nav.roll()]), nav.velocityVector().transpose(), nav.positionVector().transpose())
        i = i + 1


if __name__ == '__main__':
    main()
