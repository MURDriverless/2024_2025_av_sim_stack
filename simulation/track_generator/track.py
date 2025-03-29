from scipy.interpolate import splprep, splev
import numpy as np

class Track:
    def __init__(self, num_waypoints=15, track_width=5, cone_spacing=5, total_length=250):
        self.num_waypoints = num_waypoints
        self.track_width = track_width
        self.cone_spacing = cone_spacing
        self.total_length = total_length
        self.centerline = []
        self.left_cones = []
        self.right_cones = []

    def generate_track(self):
        # Generate random waypoints on a perturbed circle
        angles = np.linspace(0, 2*np.pi, num_waypoints, endpoint=False)
        radii = np.random.uniform(20, 30, size=num_waypoints)  # Controls how "round" the track is
        x = radii * np.cos(angles)
        y = radii * np.sin(angles)

        # Close the loop
        # x = np.append(x, x[0])
        # y = np.append(y, y[0])

        # Fit a periodic B-spline to get a smooth centerline
        tck, u = splprep([x, y], s=0.5, per=True)
        num_samples = int(total_length / cone_spacing)
        u_fine = np.linspace(0, 1, num_samples)
        x_smooth, y_smooth = splev(u_fine, tck)
        centerline = np.vstack((x_smooth, y_smooth)).T

        # Find closest point to origin
        closest_index = np. argmin(np.linalg.norm(centerline, axis=1))
        closest_point = centerline[closest_index]

        # Rotate and translate so taht the closest point is at (0,0)
        centerline = np.roll(centerline, -closest_index, axis=0)
        offset = centerline[0]
        centerline -= offset

        self._generate_cones()

    def _generate_cones(self):
        for i in range(len(centerline)):
            p = centerline[i]
            p_next = centerline[(i + 1) % len(centerline)]
            direction = p_next - p
            direction /= np.linalg.norm(direction)
            normal = np.array([-direction[1], direction[0]])

            left = p + (track_width / 2) * normal
            right = p - (track_width / 2) * normal

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
                left_cones.append({"x": left[0], "y": left[1], "side": "left", "color": "orange"})
                right_cones.append({"x": right[0], "y": right[1], "side": "right", "color": "orange"})
            else:
                # add color to the cones as well as which side of the track these cones are placed
                left_cones.append({"x": left[0], "y": left[1], "side": "left", "color": "yellow"})
                right_cones.append({"x": right[0], "y": right[1], "side": "right", "color": "blue"})

    def get_track_data(self):
        return {
            "centerline": centerline.tolist(),
            "left_cones": left_cones,
            "right_cones": right_cones
        }

    def get_start_finish_line(self):
        return self.left_cones[0], self.right_cones[0]