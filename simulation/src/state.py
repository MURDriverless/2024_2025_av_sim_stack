import numpy as np

class State:
    def __init__(self, state = None):

        if state is None:
            state = [0, 0, 0, 0, 0, 0, 0]

        self.x = state[0]
        self.y = state[1]
        self.yaw = state[2]
        self.u = state[3]
        self.v = state[4]
        self.w = state[5]
        self.delta = state[6]

    def __add__(self, other):
        if len(other) != 7:
            raise ValueError("Vector must have exactly 7 elements.")
        new_state = State([
            self.x + other[0],
            self.y + other[1],
            self.yaw + other[2],
            self.u + other[3],
            self.v + other[4],
            self.w + other[5],
            self.delta + other[6]
        ])
        return new_state


    def get_state(self):
        return np.array([self.x, self.y, self.yaw, self.u, self.v, self.w, self.delta])

    def print_state(self):
        print(f"State: x={self.x}, y={self.y}, yaw={self.yaw}, u={self.u}, v={self.v}, w={self.w}, delta={self.delta}")

    def __repr__(self):
        return f"State(x={self.x}, y={self.y}, yaw={np.rad2deg(self.yaw)}, u={self.u}, v={self.v}, w={self.w}, delta={np.rad2deg(self.delta)})"
    

class Input:
    def __init__(self, F=0.0, delta_dot=0.0):
        self.F = F
        self.delta_dot = delta_dot


class State_C:
    def __init__(self, state = None):

        if state is None:
            state = [0, 0, 0, 0, 0, 0, 0]

        self.x = state[0]
        self.y = state[1]
        self.yaw = state[2]
        self.u = state[3]

    def __add__(self, other):
        if len(other) != 7:
            raise ValueError("Vector must have exactly 4 elements.")
        new_state = State([
            self.x + other[0],
            self.y + other[1],
            self.yaw + other[2],
            self.u + other[3]
        ])
        return new_state


    def get_state(self):
        return np.array([self.x, self.y, self.yaw, self.u])

    def print_state(self):
        print(f"State: x={self.x}, y={self.y}, yaw={self.yaw}, u={self.u}")

    def __repr__(self):
        return f"State(x={self.x}, y={self.y}, yaw={np.rad2deg(self.yaw)}, u={self.u})"