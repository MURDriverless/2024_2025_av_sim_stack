import math
import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt


class Camera:
    def __init__(self, car, range_max=10.0, fov = 180, fps=30):
        
        car.camera = True
        car.gpu.camera = self
        self.car = car

        self.range_max = range_max
        self.fov = fov
        self.fps = fps
        self.sense_data = []
        self.points = []
        self.noise = 0


class Mono(Camera):
    def __init__(self, car, range_max=10.0, fov = 110, fps=30):
        
        car.lidar = self
        self.car = car
        self.range_max = range_max
        self.fov = fov
        self.fps = fps
        self.sense_data = []
        self.points = []
        self.noise = 0

    def find_visible_cones(self):

        visible_cones = []
        forward_vec = np.array([np.cos(self.state.yaw), np.sin(self.state.yaw)])

        # Step 2: Check distance to each cone
        for cone in self.track.cones:

            dx = cone.x - self.state.x
            dy = cone.y - self.state.y
            dist = math.hypot(dx, dy)

            direct_to_cone = np.array([dx,dy])/dist
            dot_product = np.dot(direct_to_cone, forward_vec)                                      # Check if the cone is in front
            dot_product = np.clip(np.dot(forward_vec, direct_to_cone), -1.0, 1.0)
            angle_deg = np.degrees(np.arccos(dot_product))


            if dist <= self.visible_range and angle_deg <= np.degrees(self.visible_angle):
                visible_cones.append(cone)
                self.seen_cones.add(cone)

        self.visible_cones = visible_cones



class Stereo(Camera):
    def __init__(self, range_max=10.0, fov = 110, fps=30):
        
        self.car = None
        self.baseline = 0.12
        self.range_max = range_max
        self.fov = fov
        self.fps = fps
        self.sense_data = []
        self.points = []
        self.noise = 0

        self.visible_cones = []
        self.forward_cones = []
        self.new_cone = 0

        self.seen_cones = set()
        self.seen_valid_edges = set()
        self.seen_invalid_edges = set()
        self.valid_edges = []
        self.invalid_edges = []

        self.path_taken = []
        self.path = []
        self.seen_path = set()

    def find_visible_cones(self):

        visible_cones = []

        forward_vec = np.array([np.cos(self.car.state.yaw), np.sin(self.car.state.yaw)])

        # Step 2: Check distance to each cone
        for cone in self.car.track.cones:

            dx = cone.x - self.car.state.x
            dy = cone.y - self.car.state.y
            dist = math.hypot(dx, dy)

            direct_to_cone = np.array([dx,dy])/dist
            dot_product = np.dot(direct_to_cone, forward_vec)                                      # Check if the cone is in front
            dot_product = np.clip(np.dot(forward_vec, direct_to_cone), -1.0, 1.0)
            angle_deg = np.degrees(np.arccos(dot_product))


            if dist <= self.range_max and angle_deg <= 85:
                visible_cones.append(cone)
                self.seen_cones.add(cone)

        self.visible_cones = visible_cones


    def plot_camera(self):

        # Simulated colors for each point (yellow, blue, orange, or None)
        color_dict = {
            "yellow": [1, 1, 0],  # RGB for yellow
            "blue": [0, 0, 1],    # RGB for blue
            "orange": [1, 0.647, 0],  # RGB for orange
            "None": [1, 1, 1]  # White for None
        }

        lidar_image = np.zeros((1, self.car.gpu.lidar.angle_num, 3))

        for i, point in enumerate(self.car.gpu.lidar.sense_data):
            flip = self.car.gpu.lidar.angle_num - 1 - i
            color_name = point["color"]
            lidar_image[0, flip] = color_dict.get(color_name, color_dict["None"])  # Default to white if None

        # Display the image
        plt.figure(figsize=(10, 1)) 
        plt.imshow(lidar_image)
        plt.axis('off')  # Hide axes for a cleaner display
        plt.show()

        # plt.figure()

        # # First, extract positions from cones
        # yellow_positions = np.array([cone.get_position() for cone in self.forward_cones if cone.color == 'yellow'])
        # blue_positions = np.array([cone.get_position() for cone in self.forward_cones if cone.color == 'blue'])

        # if len(yellow_positions) > 0:                                                                                       # Plot yellow and blue cones
        #     plt.scatter(yellow_positions[:, 0], yellow_positions[:, 1], c='yellow', marker='o', label="Yellow Cones")
        # if len(blue_positions) > 0:
        #     plt.scatter(blue_positions[:, 0], blue_positions[:, 1], c='blue', marker='o', label="Blue Cones")

        # for pt1, pt2 in self.invalid_edges:
        #     plt.plot([pt1.x, pt2.x], [pt1.y, pt2.y], 'r--', linewidth=1.5)      # Plot invalid edges (stored as tuples of positions)
        

        # for pt1, pt2 in self.valid_edges:
        #     plt.plot([pt1.x, pt2.x], [pt1.y, pt2.y], c="black")                 # Plot valid edges (from Delaunay simplices)

        
        # midpoints_array = np.array(self.path)
        # if len(midpoints_array) > 0:
        #     plt.scatter(midpoints_array[:, 0], midpoints_array[:, 1], c='green', marker='x', label="Midpoints") # Plot midpoints

        # plt.scatter(self.car.state.x, self.car.state.y, c='purple', marker='s', label="Car")                            # Plot car

        # plt.title("Camera Vision")

        # seen_midpoints_array = np.array(list(self.seen_path))
        # if len(seen_midpoints_array) > 0:
        #     plt.scatter(seen_midpoints_array[:, 0], seen_midpoints_array[:, 1], c='cyan', marker='x', label="Midpoints")    # Plot seen midpoints


        # for pt1, pt2 in self.seen_valid_edges:
        #     plt.plot([pt1.x, pt2.x], [pt1.y, pt2.y], c="black")                                                 # Plot valid edges (from Delaunay simplices)

        # for pt1, pt2 in self.seen_invalid_edges:
        #     plt.plot([pt1.x, pt2.x], [pt1.y, pt2.y], 'r--', linewidth=1.5)                                      # Plot invalid edges (stored as tuples of positions)




    def update(self):

        self.find_visible_cones()