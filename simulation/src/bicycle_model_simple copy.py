import math
import sys
import numpy as np

max_steer = np.deg2rad(45.0)
dt = 0.1


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, u=0.0, v=0.0, yaw_dot=0.0):
        self.mass = 1
        self.kf = 0.1
        self.kr = 0.2
        self.lf = 0.1
        self.lr = 0.2
        self.Iz = 1

        self.x = x
        self.y = y
        self.yaw = yaw
        self.u = u
        self.v = v
        self.yaw_dot = yaw_dot


def update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer

    state.x = state.x + (state.u * math.cos(state.yaw) - state.v * math.sin(state.yaw)) * dt
    state.y = state.y + (state.v * math.cos(state.yaw) + state.u * math.sin(state.yaw)) * dt
    state.yaw = state.yaw + state.yaw_dot * dt
    state.u = state.u + a * dt
    state.v = (state.mass * state.u * state.v + (state.lf * state.kf - state.lr * state.kr) * state.yaw_dot * dt - state.kf * delta * state.u * dt - state.mass * state.u * state.u * state.yaw_dot * dt) / (
        state.mass * state.u - (state.kf + state.kr) * dt)

    state.yaw_dot = (state.Iz * state.u * state.yaw_dot + (state.lf * state.kf - state.lr * state.kr) * state.v * dt - state.lf * state.kf * delta * state.u * dt) / (
        state.Iz * state.u - (state.lf * state.lf * state.kf - state.lr * state.lr * state.kr) * dt)
    return state


if __name__ == "__main__":
    # Initial position and orientation of the circle
    state = State(x=-0.0, y=-0.0, yaw=0.0, u=0.0, v=0.0, yaw_dot=0.0)

    # Move forward for 10 meters
    distance_traveled = 0.0
    while (distance_traveled < 10):
        state.update(state, 1, 0)
        distance_traveled += np.sqrt(state.u * state.u + state.v * state.v) * dt
        print(state.x, state.y, distance_traveled)
