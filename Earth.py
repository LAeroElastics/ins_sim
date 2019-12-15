import numpy as np

r_e = 6378137
omega_e = 7.292115e-5
eps = 0.0818191908426
g_WGS0 = 9.7803267714
g_WGS1 = 0.00193185138639


def gravity(position):
    phi = position[0]
    result = np.zeros((3, 1))
    result[2] = g_WGS0 * (1.0 + g_WGS1 * np.sin(phi) ** 2.0) / np.sqrt(1.0 - eps ** 2.0 * np.sin(phi) ** 2.0)
    return result


# FIXME I DONT LIKE THIS(HOW DID SHE KNOW THE INDEX OF LATITUDE??)
def meridian(position):
    phi = position[0]
    return r_e * (1.0 - eps ** 2.0) * (1.0 - eps ** 2.0 * np.sin(phi) ** 2.0) ** (-1.5)


def normal(position):
    phi = position[0]
    return r_e * (1.0 - eps ** 2.0 * np.sin(phi) ** 2.0) ** (-0.5)
