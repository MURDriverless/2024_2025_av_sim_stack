
import numpy as np

class Wheel_encoder():

    def __init__(self, gpu):

        self.gpu = gpu
        self.resolution = None
        self.speed = None

    def update(self):

        self.speed, _ = [self.gpu.car.state.u, self.gpu.car.state.v]

        return self.speed

