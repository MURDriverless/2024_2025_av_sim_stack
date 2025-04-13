# track_gen/perception.py
import numpy as np

class CameraSensor:
    def __init__(self, fov_deg=90, max_range=30.0, min_range=0.5, stereo_cutoff=10.0):
        self.fov_rad = np.radians(fov_deg)  # field of view in radians
        self.max_range = max_range
        self.min_range = min_range
        self.stereo_cutoff = stereo_cutoff  # below this distance, stereo is available

    def get_visible_cones(self, vehicle_state, cones):
        visible = []

        # Extracts vehicle position and orientation, and computes the vehicle heading unit vector in global coordinates
        x_v, y_v, yaw = vehicle_state['x'], vehicle_state['y'], vehicle_state['yaw']
        heading = np.array([np.cos(yaw), np.sin(yaw)])

        # For each cone, calculates the vector and distance from the vehicle to the cone
        for cone in cones:
            dx = cone['x'] - x_v
            dy = cone['y'] - y_v
            rel_pos = np.array([dx, dy])
            distance = np.linalg.norm(rel_pos)

            # FIlter the cones that are too far or too close 
            if distance < self.min_range or distance > self.max_range:
                continue

            # Computes the angle between the heading and the cone direction
            # Skips cones outside the cameras FOV
            angle = np.arccos(np.clip(np.dot(rel_pos / distance, heading), -1.0, 1.0))
            if angle > self.fov_rad / 2:
                continue

            # Labels the cone depending on which camera identified it at that time instance
            cone_type = 'stereo' if distance <= self.stereo_cutoff else 'mono'
            visible.append({
                'x': cone['x'],
                'y': cone['y'],
                'distance': distance,
                'angle': angle,
                'color': cone['color'],
                'side': cone['side'],
                'detection_type': cone_type
            })

        # Return detected cones at that time instance
        return visible

    def visualize_detections(self, detections):
        # Plots cones depending whether they were detected by the stereo or mono camera
        import matplotlib.pyplot as plt
        for cone in detections:
            if cone['detection_type'] == 'stereo':
                # Checks if label is NOT alreadyu in the existing legend labels. Avoids duplicate entries in the legend.
                plt.scatter(cone['x'], cone['y'], c='lime', s=50, marker='o', label='Stereo Cone' if 'Stereo Cone' not in plt.gca().get_legend_handles_labels()[1] else "")
            else:
                plt.scatter(cone['x'], cone['y'], c='red', s=50, marker='^', label='Mono Cone' if 'Mono Cone' not in plt.gca().get_legend_handles_labels()[1] else "")
