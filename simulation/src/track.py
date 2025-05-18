import math
import numpy as np

import world as w
from cone import Cone


class Track:
    def __init__(self, world, name, cones=None, cars=None):

        if world not in w.worlds:
            raise ValueError(f"World: {world.name} doesnt exist")
    
        self.world = world
        world.add_track(self)
        self.name = name
        self.cones = []
        self.start_line = []
        self.cars = []

        if cones:
            for item in cones:
                if isinstance(item, Cone):
                    self.cones.append(item)
                elif (
                    isinstance(item, (list, tuple))
                    and len(item) == 2
                    and isinstance(item[0], (list, tuple))
                    and len(item[0]) == 2
                    and isinstance(item[1], str)
                ):
                    x, y = item[0]
                    color = item[1]
                    self.cones.append(Cone(x, y, color))
                    if color == 'orange':
                        self.start_line.append(Cone(x, y, color))

                else:
                    raise ValueError("Cones must be Cone objects or [[x, y], color] format")

        # Vector along the start line
        dx = self.start_line[1].x - self.start_line[0].x
        dy = self.start_line[1].y - self.start_line[0].y
        start_vec = np.array([dx, dy])

        # Compute the two possible normals (perpendiculars)
        normal1 = np.array([-dy, dx])
        normal2 = np.array([dy, -dx])

        # Choose the normal that points more in the +y direction
        chosen_normal = normal1 if normal1[1] > normal2[1] else normal2

        # Normalize and compute angle
        chosen_normal = chosen_normal / np.linalg.norm(chosen_normal)
        self.start_angle = math.atan2(chosen_normal[1], chosen_normal[0])  # radians
        
        self.start_point = [0.5*(self.start_line[0].x+self.start_line[1].x), 0.5*(self.start_line[0].y+self.start_line[1].y)]

        if cars:
            self.cars = cars

    def add_cone(self, cone):
        self.cones.append(cone)

    def add_car(self, car):
        self.cars.append(car)

    def get_car_names(self):
        """Returns a list of all car names on this track."""
        return [car.name for car in self.cars]
    
    def get_car_by_name(self, name):
        """Returns the car object with the given name."""
        for car in self.cars:
            if car.name == name:
                return car
        raise ValueError(f"Car '{name}' not found on track '{self.name}'.")

    def get_cones_by_color(self, color):
        return [cone for cone in self.cones if cone.color == color]

    def __repr__(self):
        return f"Track(name='{self.name}', cones={len(self.cones)}, cars={len(self.cars)})"


class SkidTrack(Track):
    def __init__():
        super().__init__()

class SpeedTrack(Track):
    def __init__():
        super().__init__()