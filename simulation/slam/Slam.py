import numpy as np

class SLAM:
    def __init__(self):
        self.poses = []             # Vehicle poses over time
        self.map_landmarks = []     # Observed cone landmarks

    def update(self, vehicle_state, detections):
        # Log pose
        self.poses.append({
            'x': vehicle_state['x'],
            'y': vehicle_state['y'],
            'yaw': vehicle_state['yaw']
        })

        # Convert detected cones from vehicle to global frame
        for cone in detections:
            rel_x = cone['distance'] * np.cos(cone['angle'])
            rel_y = cone['distance'] * np.sin(cone['angle'])

            # Rotate and translate into global coordinates
            global_x = vehicle_state['x'] + np.cos(vehicle_state['yaw']) * rel_x - np.sin(vehicle_state['yaw']) * rel_y
            global_y = vehicle_state['y'] + np.sin(vehicle_state['yaw']) * rel_x + np.cos(vehicle_state['yaw']) * rel_y

            self.map_landmarks.append({
                'x': global_x,
                'y': global_y,
                'color': cone['color'],
                'detection_type': cone['detection_type']
            })

    def get_map(self):
        return self.map_landmarks

    def get_trajectory(self):
        return self.poses
