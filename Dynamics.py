import numpy as np
from numpy.linalg import norm

from Constants import *



def iterate_until_dist(dist, v0):
    max_t = 100
    accel = np.zeros(max_t)
    v = np.zeros(max_t)
    e = np.zeros(max_t)
    accel[0] = ai(v0)
    v[0] = v0
    t = 0
    while e[t] < dist and t+1 < max_t:
        accel[t] = ai(v[t])
        v[t+1] = vii(v[t], accel[t])
        e[t+1] = eii(e[t], v[t])
        t += 1
    return {"t": t, 'a': accel[t], 'v': v[t], 'e': e[t]}


def rotate(origin: np.array, point: np.array, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    # print("rotating  " + str(point) + " through " + str(origin) + " into " + str(np.array([qx, qy])), file=sys.stderr)
    return np.array([qx, qy])


def ai(v_i, thrust=0):
    return 0.85*thrust - 0.21*v_i


def vii(v_i, a_i):
    return v_i + a_i*dt


def eii(e_i, v_i):
    return e_i + v_i*dt


def ai2d(v_i: np.array, thrust: np.array=np.array([0, 0])):
    return 0.85**thrust - 0.21**v_i


def vii2d(v_i: np.array, a_i: np.array):
    return v_i + a_i ** dt


def eii2d(e_i: np.array, v_i: np.array):
    return e_i + v_i**dt


def u_vec(vector):
    """ Returns the unit vector of the vector.  """
    if norm(vector) == 0:
        return np.array([0, 0])
    return vector / norm(vector)


def ang(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'
    """
    # print("ang v2: " +str(v_ang(v2)) + " - ang v1: " +str(v_ang(v1)) , file=sys.stderr)
    return vector_ang(v2) - vector_ang(v1)


def points_ang(p_1, p_2):
    return vector_ang(p_1 - p_2)


def vec(angle):
    return np.array([np.cos(angle), np.sin(angle)])


def vector_ang(v):
    """"returns angle in radians of a vector"""
    v_u = u_vec(v)
    return np.arctan2(v_u[1], v_u[0])


def ang_set(i_ang):
    return i_ang % 2*math.pi
