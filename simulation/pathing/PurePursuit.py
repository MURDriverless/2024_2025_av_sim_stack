import numpy as np

class PurePursuitController:
    def __init__(self, lookahead_distance=5.0):
        self.lookahead_distance = lookahead_distance

    # def find_target_point(self, path, vehicle_state):
    #     x, y = vehicle_state['x'], vehicle_state['y']
    #     for i in range(len(path) - 1):
    #         point = np.array(path[i])
    #         dist = np.linalg.norm(point - np.array([x, y]))
    #         if dist >= self.lookahead_distance:
    #             return point
    #     return np.array(path[-1])  # fallback: last point

    # FIX: Looks forward along the path by arc length instead of finding the first
    # distant-enough point by Euclidean distance
    def find_target_point(self, path, vehicle_state):
        position = np.array([vehicle_state['x'], vehicle_state['y']])
        distances = np.linalg.norm(path - position, axis=1)
        closest_index = np.argmin(distances)

        # Walk forward from the closest index to find lookahead point
        arc_dist = 0.0
        for i in range(closest_index, len(path) - 1):
            seg = np.linalg.norm(path[i + 1] - path[i])
            arc_dist += seg
            if arc_dist >= self.lookahead_distance:
                return path[i + 1]

        return path[-1]  # fallback

    def compute_control(self, vehicle_state, target_point, wheelbase):
        dx = target_point[0] - vehicle_state['x']
        dy = target_point[1] - vehicle_state['y']

        # Transform to vehicle coordinates
        local_x = np.cos(-vehicle_state['yaw']) * dx - np.sin(-vehicle_state['yaw']) * dy
        local_y = np.sin(-vehicle_state['yaw']) * dx + np.cos(-vehicle_state['yaw']) * dy

        if local_x == 0:
            return 0.0

        # Pure pursuit curvature -> steering
        curvature = 2.0 * local_y / (self.lookahead_distance ** 2)
        steering_angle = np.arctan(wheelbase * curvature)
        return steering_angle
