import numpy as np


class Check:
    def __init__(self, i_x=-1, i_y=-1):
        self.p = np.array([i_x, i_y])

    def __eq__(self, other: np.array):
        return np.array_equal(self.p, other.p)

    def x(self):
        return self.p[0]

    def y(self):
        return self.p[1]
