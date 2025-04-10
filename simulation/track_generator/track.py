from scipy.interpolate import splprep, splev
import numpy as np

class Track:
    def __init__(self, num_waypoints=12, track_width=5, cone_spacing=5, total_length=250):
        self.num_waypoints = num_waypoints
        self.track_width = track_width
        self.cone_spacing = cone_spacing
        self.total_length = total_length
        self.centerline = []
        self.left_cones = []
        self.right_cones = []

    def generate_track(self):
        # Generate random waypoints on a perturbed circle
        angles = np.linspace(0, 2*np.pi, self.num_waypoints, endpoint=False)
        radii = np.random.uniform(20, 30, size=self.num_waypoints)  # Controls how "round" the track is
        x = radii * np.cos(angles)
        y = radii * np.sin(angles)

        # Fit a periodic B-spline to get a smooth centerline
        tck, u = splprep([x, y], s=0.5, per=True)
        num_samples = int(self.total_length / self.cone_spacing)
        # u_fine = np.linspace(0, 1, num_samples)
        # x_smooth, y_smooth = splev(u_fine, tck)
        # self.centerline = np.vstack((x_smooth, y_smooth)).T

        # Changed: Changes smoothness of centerline
        u_dense = np.linspace(0, 1, 10 * num_samples)
        x_dense, y_dense = splev(u_dense, tck)
        dense_path = np.vstack((x_dense, y_dense)).T

        # Compute arc-length spacing for centerline
        dists = np.cumsum(np.linalg.norm(np.diff(dense_path, axis=0), axis=1))
        dists = np.insert(dists, 0, 0)
        arc_samples = np.linspace(0, dists[-1], num_samples)
        x_resampled = np.interp(arc_samples, dists, x_dense)
        y_resampled = np.interp(arc_samples, dists, y_dense)

        self.centerline = np.vstack((x_resampled, y_resampled)).T
        # End of change

        # Find closest point to origin
        closest_index = np. argmin(np.linalg.norm(self.centerline, axis=1))
        closest_point = self.centerline[closest_index]

        # Rotate and translate so taht the closest point is at (0,0)
        self.centerline = np.roll(self.centerline, -closest_index, axis=0)
        offset = self.centerline[0]
        self.centerline -= offset

        # Ensure the loop closes by appending the start point
        if not np.allclose(self.centerline[0], self.centerline[-1], atol=1e-3):
            self.centerline = np.vstack([self.centerline, self.centerline[0]])

        self._generate_cones()

    def _generate_cones(self):
        for i in range(len(self.centerline)):
            p = self.centerline[i]
            p_next = self.centerline[(i + 1) % len(self.centerline)]
            direction = p_next - p
            direction /= np.linalg.norm(direction)
            normal = np.array([-direction[1], direction[0]])

            left = p + (self.track_width / 2) * normal
            right = p - (self.track_width / 2) * normal

            # left_cones.append(left.tolist())
            # right_cones.append(right.tolist())

            # Check if it's the origin / start
            # if np.allclose(p, [0.0, 0.0], atol=1e-3):
            #     left_cones.append({
            #         "x": left[0],
            #         "y": left[1],
            #         "side": "left",
            #         "color": "orange"
            #     })

            #     right_cones.append({
            #         "x": right[0],
            #         "y": right[1],
            #         "side": "left",
            #         "color": "orange"
            #     })

            # else:
            #     # add color to the cones as well as which side of the track these cones are placed
            #     left_cones.append({ 
            #         "x": left[0],
            #         "y": left[1],
            #         "side": "left",
            #         "color": "yellow"
            #     })

            #     right_cones.append({
            #         "x": right[0],
            #         "y": right[1],
            #         "side": "right",
            #         "color": "blue"
            #     })

            if i == 0:
                # Place orange cones at start/finish line
                self.left_cones.append({"x": left[0], "y": left[1], "side": "left", "color": "orange"})
                self.right_cones.append({"x": right[0], "y": right[1], "side": "right", "color": "orange"})
            else:
                # add color to the cones as well as which side of the track these cones are placed
                self.left_cones.append({"x": left[0], "y": left[1], "side": "left", "color": "yellow"})
                self.right_cones.append({"x": right[0], "y": right[1], "side": "right", "color": "blue"})

    def get_track_data(self):
        return {
            "centerline": self.centerline.tolist(),
            "left_cones": self.left_cones,
            "right_cones": self.right_cones
        }

    def get_start_finish_line(self):
        return self.left_cones[0], self.right_cones[0]