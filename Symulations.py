
from numpy.linalg import norm
import matplotlib.pyplot as plt
from Dynamics import *


class State:
    def __init__(self):
        self.e = np.array([0, 0])
        self.v = np.array([0, 0])
        self.a = np.array([0, 0])
        self.ang = 0

    def step(self, thrust: int, d_ang: int):

        if d_ang > 18:
            d_ang = 18
        elif d_ang < -18:
            d_ang = -18
        d_ang = d_ang*rad
        self.ang = self.ang + d_ang

        if thrust > 100:
            thrust = 100
        if thrust < 0:
            thrust = 0
        thrust_vec = np.array([thrust*np.cos(self.ang), thrust*np.sin(self.ang)])
        self.a = -0.21 * self.v + 0.85 * thrust_vec

        self.e = self.e + self.v
        self.v = self.v + self.a



plt.axis([-5000, 5000, -5000, 5000])
plt.ion()
state = State()
while True:
    plt.scatter(state.e[0], state.e[1])
    plt.quiver(state.e[0], state.e[1], [state.v[0]], [state.v[1]])
    direction = vec(state.ang)
    plt.quiver(state.e[0], state.e[1], direction[0], direction[1])
    print(str(norm(state.v)))
    state.step(100, 15)
    plt.pause(0.1)