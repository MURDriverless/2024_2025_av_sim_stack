import math
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
from scipy.spatial import cKDTree

from state import State
import path_generator
from cone import Cone
from cone import TrackedCone
from wheel_encoder import Wheel_encoder

import fastslam2 as fs2

#from fastslam import Particle, initialize_particles, resample_particles

timesteps = 50000

class GPU:
    def __init__(self, car):
        self.car = car
        self.camera = None
        self.lidar = None
        self.controller = None
        self.we = Wheel_encoder(self)

        self.time = 0
        self.start = False

        self.slow_gen = path_generator.SlowLap()
        self.fast_gen = path_generator.FastLap()

        self.camera_data = None
        self.controller_data = None
        self.lidar_data = None
        self.prev_lidar_data = None
        self.lidar_filt = None
        self.prev_lidar_filt = None

        self.control_average = []

        self.tracked_cones = []       # Currently tracked and active
        self.inactive_cones = []      # Cones no longer seen but preserved
        self.next_cone_id = 1         # Global ID counter

        self.particles = [fs2.Particle(x=0,y=0,yaw=np.deg2rad(45),car=car) for _ in range(fs2.N_PARTICLE)]
        self.control_ekf = fs2.ControlEKF(car)
        self.estimated_pose = np.array([0, 0, 0])

        self.map = None
        self.path = None
        self.path_taken = [self.car.get_position()]

        self.state_e = State([0, 0, np.deg2rad(45), 0, 0, 0, 0])
        self.alpha = 0.0

        self.matlab = False

    def kalman():
        help = 0

    def plot_matched_cones_by_index(self, matches):
        plt.figure(figsize=(8, 6))
        
        for prev_idx, curr_idx in matches:
            prev_point = self.prev_lidar_data[prev_idx]['position']
            curr_point = self.lidar.relative_sense_data[curr_idx]['position']

            if prev_point is None or curr_point is None:
                continue
                
            # Plot previous point
            plt.scatter(prev_point[0], prev_point[1], color='blue', label='Previous' if 'Previous' not in plt.gca().get_legend_handles_labels()[1] else "")
            
            # Plot current point
            plt.scatter(curr_point[0], curr_point[1], color='red', label='Current' if 'Current' not in plt.gca().get_legend_handles_labels()[1] else "")

            # Draw line between matched points
            plt.plot([prev_point[0], curr_point[0]], [prev_point[1], curr_point[1]], 'k--', linewidth=1)

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Matched Points Across Frames (Index-Based)')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        plt.show()




    def euclidean_distance(point1, point2):
        """
        Euclidean distance between two points.
        :param point1: the first point as a tuple (a_1, a_2, ..., a_n)
        :param point2: the second point as a tuple (b_1, b_2, ..., b_n)
        :return: the Euclidean distance
        """
        a = np.array(point1)
        b = np.array(point2)

        return np.linalg.norm(a - b, ord=2)


    def point_based_matching(self, point_pairs):
        """
        Computes the optimal 2D rigid transformation (rotation + translation) using SVD.
        :param point_pairs: list of point pairs [((x, y), (xp, yp)), ...]
        :return: (rotation_angle, translation_x, translation_y)
        """ 
        if len(point_pairs) == 0:
            return None, None, None

        # Convert to numpy arrays
        src = np.array([p[0] for p in point_pairs])  # original points
        dst = np.array([p[1] for p in point_pairs])  # transformed (target) points

        # Compute centroids
        src_mean = np.mean(src, axis=0)
        dst_mean = np.mean(dst, axis=0)

        # Center the points
        src_centered = src - src_mean
        dst_centered = dst - dst_mean

        # Compute covariance matrix
        H = src_centered.T @ dst_centered

        # Compute SVD
        U, _, Vt = np.linalg.svd(H)

        # Compute rotation matrix
        R = Vt.T @ U.T

        # Handle reflection case
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        # Extract angle from rotation matrix
        rot_angle = np.arctan2(R[1, 0], R[0, 0])

        # Compute translation
        t = dst_mean - R @ src_mean

        return rot_angle, t[0], t[1]



    def icp(self, reference_points, points, max_iterations=200, distance_threshold=0.3, convergence_translation_threshold=1e-19,
            convergence_rotation_threshold=1e-19, point_pairs_threshold=10, verbose=False):
        """
        An implementation of the Iterative Closest Point algorithm that matches a set of M 2D points to another set
        of N 2D (reference) points.

        :param reference_points: the reference point set as a numpy array (N x 2)
        :param points: the point that should be aligned to the reference_points set as a numpy array (M x 2)
        :param max_iterations: the maximum number of iteration to be executed
        :param distance_threshold: the distance threshold between two points in order to be considered as a pair
        :param convergence_translation_threshold: the threshold for the translation parameters (x and y) for the
                                                transformation to be considered converged
        :param convergence_rotation_threshold: the threshold for the rotation angle (in rad) for the transformation
                                                to be considered converged
        :param point_pairs_threshold: the minimum number of point pairs the should exist
        :param verbose: whether to print informative messages about the process (default: False)
        :return: the transformation history as a list of numpy arrays containing the rotation (R) and translation (T)
                transformation in each iteration in the format [R | T] and the aligned points as a numpy array M x 2
        """

        transformation_history = []

        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(reference_points)

        for iter_num in range(max_iterations):
            if verbose:
                print('------ iteration', iter_num, '------')

            closest_point_pairs = []  # list of point correspondences for closest point rule

            distances, indices = nbrs.kneighbors(points)
            for nn_index in range(len(distances)):
                if distances[nn_index][0] < distance_threshold:
                    closest_point_pairs.append((points[nn_index], reference_points[indices[nn_index][0]]))

            # if only few point pairs, stop process
            if verbose:
                print('number of pairs found:', len(closest_point_pairs))
            if len(closest_point_pairs) < point_pairs_threshold:
                if verbose:
                    print('No better solution can be found (very few point pairs)!')
                break

            # compute translation and rotation using point correspondences
            closest_rot_angle, closest_translation_x, closest_translation_y = self.point_based_matching(closest_point_pairs)
            if closest_rot_angle is not None:
                if verbose:
                    print('Rotation:', math.degrees(closest_rot_angle), 'degrees')
                    print('Translation:', closest_translation_x, closest_translation_y)
            if closest_rot_angle is None or closest_translation_x is None or closest_translation_y is None:
                if verbose:
                    print('No better solution can be found!')
                break

            # transform 'points' (using the calculated rotation and translation)
            c, s = math.cos(closest_rot_angle), math.sin(closest_rot_angle)
            rot = np.array([[c, -s],
                            [s, c]])
            aligned_points = np.dot(points, rot.T)
            aligned_points[:, 0] += closest_translation_x
            aligned_points[:, 1] += closest_translation_y

            # update 'points' for the next iteration
            points = aligned_points

            # update transformation history
            transformation_history.append(np.hstack((rot, np.array([[closest_translation_x], [closest_translation_y]]))))

            # check convergence
            if (abs(closest_rot_angle) < convergence_rotation_threshold) \
                    and (abs(closest_translation_x) < convergence_translation_threshold) \
                    and (abs(closest_translation_y) < convergence_translation_threshold):
                if verbose:
                    print('Converged!')
                break

        return transformation_history, points


    def match_lidar_points(self, max_iterations=50, threshold=1, verbose=False):
        """
        Match lidar points using the ICP method and return matched points.
        """
        raw_matches = []

        # Get the previous and current lidar data
        prev_points = np.array([cone['position'] for cone in self.prev_lidar_filt])  # previous frame
        curr_points = np.array([cone['position'] for cone in self.lidar.sense_filt])  # current frame


        # Perform ICP to estimate the transformation
        transformation_history, aligned_points = self.icp(prev_points, curr_points, max_iterations, threshold, verbose=verbose)

        self.plot_transformation(prev_points, curr_points, aligned_points)

        if len(transformation_history) > 0:
            # Start with identity transformation
            overall_transform = np.eye(3)

            for transform in transformation_history:
                # Convert 2x3 [R | t] into a full 3x3 homogeneous transform
                T = np.eye(3)
                T[:2, :2] = transform[:, :2]
                T[:2, 2] = transform[:, 2]

                # Compose transformations
                overall_transform = overall_transform @ T

            # Apply the overall transformation to the pose
            self.apply_transformation_to_pose(overall_transform)

            return raw_matches

        return []
    
    def apply_transformation_to_pose(self, transformation):
        """
        Apply transformation from ICP (which is in the car's frame) to update pose in the global/inertial frame.
        """
        R = transformation[:2, :2]
        t = transformation[:2, 2]

        # Step 1: Convert the translation from body frame to inertial frame
        theta = self.state_e.yaw  # current orientation of the car
        rotation_to_global = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])
        t_global = rotation_to_global @ t

        # Step 2: Apply translation in inertial frame
        self.state_e.x += t_global[0]
        self.state_e.y += t_global[1]

        # Step 3: Update yaw with the rotation from ICP
        delta_yaw = np.arctan2(R[1, 0], R[0, 0])
        self.state_e.yaw += delta_yaw


    def apply_transformation_to_pose_d(self, R, T):
        """
        Apply transformation from ICP (which is in the car's frame) to update pose in the global/inertial frame.
        """

        # Step 1: Convert the translation from body frame to inertial frame
        theta = self.state_d.yaw  # current orientation of the car
        rotation_to_global = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])
        t_global = rotation_to_global @ T

        # Step 2: Apply translation in inertial frame
        self.state_d.x += t_global[0]
        self.state_d.y += t_global[1]

        # Step 3: Update yaw with the rotation from ICP
        delta_yaw = np.arctan2(R[1, 0], R[0, 0])
        self.state_d.yaw += delta_yaw

    

    def plot_transformation(self, reference_points, initial_points, aligned_points):
        """
        Plot the reference points, the initial points, and the transformed points
        to visually verify the ICP transformation.
        
        :param reference_points: The reference points (N x 2 numpy array)
        :param initial_points: The original points (M x 2 numpy array)
        :param aligned_points: The transformed (aligned) points (M x 2 numpy array)
        """
        # Create a figure and axis for plotting
        plt.figure(figsize=(8, 8))
        
        # Plot the reference points (e.g., in blue)
        plt.scatter(reference_points[:, 0], reference_points[:, 1], color='blue', label='Reference Points', marker='x')

        # Plot the initial points (e.g., in red)
        plt.scatter(initial_points[:, 0], initial_points[:, 1], color='red', label='Initial Points', marker='x')
        
        # Plot the aligned points (e.g., in green)
        plt.scatter(aligned_points[:, 0], aligned_points[:, 1], color='green', label='Transformed Points (ICP)', marker='x')
        
        
        # Adding labels and legend
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('ICP Point Matching')
        plt.legend()
        plt.grid(True)
        
        # Show the plot
        plt.show()


    def car_to_world(self, data=None):
        """
        Transforms either:
        - a list of Cone-like objects from car frame to world frame, or
        - a single target (x, y) point.

        If `data` is None, defaults to self.lidar_data.
        """
        yaw = self.car.state.yaw
        R = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw),  np.cos(yaw)]
        ])
        
        if data is None:
            data = self.lidar_data

        # If a single tuple or list is passed, interpret it as a point
        if isinstance(data, (tuple, list, np.ndarray)) and len(data) == 2 and isinstance(data[0], (int, float)):
            local_pos = np.array(data)
            rotated = R @ local_pos
            world_x = rotated[0] + self.car.state.x
            world_y = rotated[1] + self.car.state.y
            return (world_x, world_y)

        # Otherwise, assume it's a list of cone-like objects
        transformed = []
        for c in data:
            local_pos = np.array([c.x, c.y])
            rotated = R @ local_pos
            world_x = rotated[0] + self.car.state.x
            world_y = rotated[1] + self.car.state.y
            transformed.append(Cone(world_x, world_y, c.color))
        
        return transformed
    
    def world_to_car(self, data):
        """
        Transforms coordinates from the world frame to the car's local frame.
        Accepts:
        - A single Cone object
        - A list of Cone objects
        - A single (x, y) point (tuple, list, or np.array)
        - A list of (x, y) points
        Returns transformed data in the same structure.
        """
        yaw = self.car.state.yaw
        R_inv = np.array([
            [np.cos(yaw), np.sin(yaw)],
            [-np.sin(yaw), np.cos(yaw)]
        ])

        def transform_point(x, y):
            dx = x - self.car.state.x
            dy = y - self.car.state.y
            local = R_inv @ np.array([dx, dy])
            return local[0], local[1]

        # Case 1: Single Cone object
        if hasattr(data, 'x') and hasattr(data, 'y'):
            x_local, y_local = transform_point(data.x, data.y)
            return Cone(x_local, y_local, data.color)

        # Case 2: List of Cone objects
        if isinstance(data, list) and all(hasattr(d, 'x') and hasattr(d, 'y') for d in data):
            return [Cone(*transform_point(d.x, d.y), d.color) for d in data]

        # Case 3: Single point (x, y)
        if isinstance(data, (tuple, list, np.ndarray)) and len(data) == 2 and all(isinstance(i, (int, float)) for i in data):
            return transform_point(data[0], data[1])

        # Case 4: List of (x, y) points
        if isinstance(data, list) and all(isinstance(d, (tuple, list, np.ndarray)) and len(d) == 2 for d in data):
            return [transform_point(p[0], p[1]) for p in data]

        raise ValueError("Unsupported input type for world_to_car transformation.")


    # def update_tracked_cones(self, current_cones, tracked_cones, inactive_cones, next_cone_id, frame_num, dist_thresh=0.5, max_missed=10):
    #     assigned_ids = []
    #     unmatched = set(range(len(current_cones)))

    #     # Step 1: Attempt to match current cones to tracked cones
    #     for t_cone in tracked_cones:
    #         best_j = -1
    #         best_dist = float('inf')
    #         for j in unmatched:
    #             c = current_cones[j]
    #             dist = np.linalg.norm([t_cone.x - c.x, t_cone.y - c.y])
    #             if dist < best_dist and dist < dist_thresh and c.color == t_cone.color:
    #                 best_dist = dist
    #                 best_j = j
    #         if best_j != -1:
    #             c = current_cones[best_j]
    #             # Update matched tracked cone
    #             t_cone.x = c.x
    #             t_cone.y = c.y
    #             t_cone.last_seen = frame_num
    #             t_cone.missed_count = 0  # Reset missed count
    #             assigned_ids.append((c, t_cone.id))
    #             unmatched.remove(best_j)
    #         else:
    #             # No match found → increment missed count
    #             t_cone.missed_count += 1

    #     # Step 2: Retire cones that have been missed too long
    #     active_cones = []
    #     for cone in tracked_cones:
    #         if cone.missed_count <= max_missed:
    #             active_cones.append(cone)
    #         else:
    #             inactive_cones.append(cone)
    #     tracked_cones[:] = active_cones

    #     # Step 3: Add unmatched cones as new tracked cones
    #     for j in unmatched:
    #         c = current_cones[j]
    #         new_cone = TrackedCone(c.x, c.y, c.color, next_cone_id)
    #         new_cone.last_seen = frame_num
    #         new_cone.missed_count = 0
    #         tracked_cones.append(new_cone)
    #         assigned_ids.append((c, next_cone_id))
    #         next_cone_id += 1

    #     return assigned_ids, tracked_cones, inactive_cones, next_cone_id

    def update_tracked_cones(self, current_cones, tracked_cones, inactive_cones, next_cone_id, frame_num,
                         dist_thresh=0.5, max_missed=10, dx=0.0, dy=0.0, dyaw=0.0):
        assigned_ids = []
        unmatched = set(range(len(current_cones)))

        # Step 0: Predict tracked_cone positions into current frame
        cos_yaw = np.cos(-dyaw)
        sin_yaw = np.sin(-dyaw)
        for t_cone in tracked_cones:
            # Apply inverse motion (cone moves relative to car)
            x_shifted = t_cone.x - dx
            y_shifted = t_cone.y - dy
            t_cone.x = cos_yaw * x_shifted - sin_yaw * y_shifted
            t_cone.y = sin_yaw * x_shifted + cos_yaw * y_shifted

        # Step 1: Attempt to match current cones to predicted tracked cones
        for t_cone in tracked_cones:
            best_j = -1
            best_dist = float('inf')
            for j in unmatched:
                c = current_cones[j]
                dist = np.linalg.norm([t_cone.x - c.x, t_cone.y - c.y])
                if dist < best_dist and dist < dist_thresh and c.color == t_cone.color:
                    best_dist = dist
                    best_j = j
            if best_j != -1:
                c = current_cones[best_j]
                # Update matched tracked cone
                t_cone.x = c.x
                t_cone.y = c.y
                t_cone.last_seen = frame_num
                t_cone.missed_count = 0
                assigned_ids.append((c, t_cone.id))
                unmatched.remove(best_j)
            else:
                t_cone.missed_count += 1

        # Step 2: Retire missed cones
        active_cones = [cone for cone in tracked_cones if cone.missed_count <= max_missed]
        inactive_cones.extend([cone for cone in tracked_cones if cone.missed_count > max_missed])
        tracked_cones[:] = active_cones

        # Step 3: Add unmatched cones as new tracked cones
        for j in unmatched:
            c = current_cones[j]
            new_cone = TrackedCone(c.x, c.y, c.color, next_cone_id)
            new_cone.last_seen = frame_num
            tracked_cones.append(new_cone)
            assigned_ids.append((c, next_cone_id))
            next_cone_id += 1

        return assigned_ids, tracked_cones, inactive_cones, next_cone_id

    
    def initialise_tracked_cones(self, initial_cones, frame_num=0):

        for c in initial_cones:
            tracked = TrackedCone(c.x, c.y, c.color, self.next_cone_id)
            tracked.last_seen = frame_num
            tracked.missed_count = 0
            self.tracked_cones.append(tracked)
            self.next_cone_id += 1


    def update(self):

        if not self.start:

            if self.lidar and not self.car.lidar:
                self.lidar_data = self.lidar.update()
                self.initialise_tracked_cones(self.lidar_data)
                self.map = self.car_to_world(self.lidar_data)
                self.car.lidar = True

            elif self.camera and not self.car.camera:
                self.camera_data = self.camera.update()
                self.car.camera = True
            
            elif (self.lidar_data or self.camera_data) and self.controller and not self.car.controller:
                self.path = self.slow_gen.update(self.lidar_data)['path']
                self.controller.set_path(self.path)
                u_est = self.we.update()
                self.controller_data = self.controller.update(u_est)
                self.controler = True  

            return


        for i in range(timesteps):

            self.car.update(self.controller_data)
            self.path_taken.append(self.car.get_position())

            input = self.controller_data

            u_meas = self.car.state.u  # from wheel encoder
            v_meas = self.car.state.v  # from lateral accel, or zero if no measurement
            w_meas = self.car.state.w  # from IMU yaw rate

            fused_velocities = self.control_ekf.fuse(input, u_meas, v_meas, w_meas, self.state_e)
            self.control_average.append(fused_velocities)

            if (not self.car.lap_finish) and (self.lidar) and (i % 5 == 0):
                if (i % 50 == 0):
                    self.lidar_data = self.lidar.update()
                    self.lidar_filt = self.lidar.filt_cones


                    z = np.zeros((2, len(self.lidar_filt)))
                    for idx, cone in enumerate(self.lidar_filt):
                        r = np.hypot(cone.x, cone.y)
                        b = np.arctan2(cone.y, cone.x)
                        z[:, idx] = [r, b]
                        # print(f'distance: {r}, angle: {np.rad2deg(b)}')


                    # 🟢 4. Run FastSLAM 2.0
                    if self.control_average:
                        u_avg = np.mean(self.control_average, axis=0)
                        self.particles = fs2.fast_slam2(self.particles, u_avg, z, self.car.state)
                        self.control_average = []  # reset for next batch

                    # 🟢 5. Estimate final state from particles
                    x_est = fs2.calc_final_state(self.particles)
                    self.state_e.x, self.state_e.y, self.state_e.yaw = x_est.flatten()

                    # 🟢 6. Update world map for plotting/debug
                    self.map = self.car_to_world(self.lidar_data)

                    
                else:
                    self.lidar_data = self.lidar.update()
                    self.map = self.car_to_world(self.lidar_data)


            if (not self.car.lap_finish) and (self.camera) and (i % 100 == 0):
                self.camera_data = self.camera.update()

            self.path = self.slow_gen.update(self.lidar_data)['path']
            self.controller.set_path(self.path)
            u_est = self.we.update()
            self.controller_data = self.controller.update(u_est)


            if i % 1000 == 0:
                if self.matlab:
                    self.plotting()
                print(self.car.state)
                self.state_e.u, self.state_e.v, self.state_e.w = fused_velocities.flatten()
                self.state_e.delta = self.car.state.delta
                print(self.state_e)
                

            self.time += 1


    def plotting(self):

        plt.figure()

        yellow_positions_a = np.array([cone.get_position() for cone in self.map if cone.color == 'yellow'])
        blue_positions_a = np.array([cone.get_position() for cone in self.map if cone.color == 'blue'])

        if len(self.car.track.start_line) > 0:
            start_line = np.array([cone.get_position() for cone in self.car.track.start_line])
            plt.scatter(start_line[:, 0], start_line[:, 1], c='orange', marker='o', label="Start Cones")

        if len(yellow_positions_a) > 0:
            plt.scatter(yellow_positions_a[:, 0], yellow_positions_a[:, 1], c='yellow', marker='^', label="Seen Yellow Cones")
        if len(blue_positions_a) > 0:
            plt.scatter(blue_positions_a[:, 0], blue_positions_a[:, 1], c='blue', marker='^', label="Seen Blue Cones")

        if len(self.path_taken) > 1:
            xs, ys = zip(*self.path_taken)
            plt.plot(xs, ys, label="Car Path", color="purple", linewidth=2)

        if hasattr(self, "particles"):
            particle_xs = [p.state.x for p in self.particles]
            particle_ys = [p.state.y for p in self.particles]
            particle_ws = [p.w for p in self.particles]

            max_w = max(particle_ws)
            min_w = min(particle_ws)
            if max_w > min_w:
                normalized_ws = [(w - min_w) / (max_w - min_w) for w in particle_ws]
            else:
                normalized_ws = [0.0 for _ in particle_ws]

            colors = plt.cm.RdYlGn_r(normalized_ws)
            plt.scatter(particle_xs, particle_ys, c=colors, s=10, alpha=0.7, label="Particles")

        # ✅ FIXED: Dynamic landmark plotting
        if hasattr(self, "particles") and len(self.particles) > 0 and len(self.particles[0].lm) > 0:
            n_landmarks = len(self.particles[0].lm)
            mean_lms = []

            for lm_id in range(n_landmarks):
                lm_positions = []
                for p in self.particles:
                    if lm_id < len(p.lm):
                        lm = p.lm[lm_id]
                        if np.linalg.norm(lm) > 1e-6:  # filter out default-initialized landmarks (optional)
                            lm_positions.append(lm)
                if lm_positions:
                    mean_pos = np.mean(lm_positions, axis=0)
                    mean_lms.append(mean_pos)

            if mean_lms:
                mean_lms = np.array(mean_lms)
                plt.scatter(mean_lms[:, 0], mean_lms[:, 1], c='black', marker='x', label='Mean Landmark Position')

        # Target and car
        target = self.car_to_world(self.controller.target)
        plt.scatter(target[0], target[1], c='red', marker='x', label="Target")
        plt.scatter(self.car.state.x, self.car.state.y, c='purple', marker='s', label="Car")

        plt.axis('equal')
        plt.legend()
        plt.title("Car Simulation")
        plt.show()

        if self.camera:
            self.camera.plot_camera()

        if self.lidar:
            self.lidar.plot_lidar()


    # def plotting(self):

    #         plt.figure()

    #         yellow_positions_a = np.array([cone.get_position() for cone in self.map if cone.color == 'yellow'])
    #         blue_positions_a = np.array([cone.get_position() for cone in self.map if cone.color == 'blue'])


    #         if len(self.car.track.start_line) > 0:
    #             start_line = np.array([cone.get_position() for cone in self.car.track.start_line])
    #             plt.scatter(start_line[:, 0], start_line[:, 1], c='orange', marker='o', label="Start Cones")


    #         if len(yellow_positions_a) > 0:
    #             plt.scatter(yellow_positions_a[:, 0], yellow_positions_a[:, 1], c='yellow', marker='^', label="Seen Yellow Cones")  # Plot yellow and blue cones seen
    #         if len(blue_positions_a) > 0:
    #             plt.scatter(blue_positions_a[:, 0], blue_positions_a[:, 1], c='blue', marker='^', label="Seen Blue Cones")

    #         if len(self.path_taken) > 1:
    #             xs, ys = zip(*self.path_taken)
    #             plt.plot(xs, ys, label="Car Path", color="purple", linewidth=2)             # Plot the trajectory as a line

    #         if hasattr(self, "particles"):
    #             particle_xs = [p.state.x for p in self.particles]
    #             particle_ys = [p.state.y for p in self.particles]
    #             particle_ws = [p.w for p in self.particles]

    #             # Normalize weights to [0, 1]
    #             max_w = max(particle_ws)
    #             min_w = min(particle_ws)
    #             if max_w > min_w:
    #                 normalized_ws = [(w - min_w) / (max_w - min_w) for w in particle_ws]
    #             else:
    #                 normalized_ws = [0.0 for _ in particle_ws]  # all weights are equal

    #             # Use red for high weights, green for low (reverse colormap)
    #             colors = plt.cm.RdYlGn_r(normalized_ws)

    #             plt.scatter(particle_xs, particle_ys, c=colors, s=10, alpha=0.7, label="Particles")         

            
    #         if hasattr(self, "particles") and len(self.particles) > 0:
    #             n_landmarks = self.particles[0].lm.shape[0]
    #             mean_lms = []

    #             for lm_id in range(n_landmarks):
    #                 lm_positions = []
    #                 for p in self.particles:
    #                     if np.any(p.lm[lm_id] != 0):  # Only include if landmark was observed
    #                         lm_positions.append(p.lm[lm_id])
    #                 if lm_positions:
    #                     mean_pos = np.mean(lm_positions, axis=0)
    #                     mean_lms.append(mean_pos)

    #             if mean_lms:
    #                 mean_lms = np.array(mean_lms)
    #                 plt.scatter(mean_lms[:, 0], mean_lms[:, 1], c='black', marker='x', label='Mean Landmark Position')

    #         target = self.car_to_world(self.controller.target)
    #         plt.scatter(target[0], target[1], c='red', marker='x', label="Target")
    #         plt.scatter(self.car.state.x, self.car.state.y, c='purple', marker='s', label="Car")                                            # Plot car

    #         #if self.controller:
    #         #    self.controller.plot_control()


    #         plt.axis('equal')
    #         plt.legend()
    #         plt.title("Car Simulation")
    #         plt.show()

    #         if self.camera:
    #             self.camera.plot_camera()

    #         if self.lidar:
    #             self.lidar.plot_lidar()

                        #     # === FastSLAM Step ===
                    # control_input = np.array([0.0, 0.0, 0.0])  # Replace with car motion delta if you have odometry
                    # dt = 0.02
                    # motion_noise = 0.05

                    # for p in self.particles:
                    #     p.predict(control_input, dt, motion_noise)
                    #     for cone in self.lidar.cones:
                    #         relative_pos = self.world_to_car((cone.x, cone.y))
                    #         p.update_landmark(cone.color, np.array(relative_pos), R=np.eye(2)*0.05)

                    # self.particles = resample_particles(self.particles)
                    # self.estimated_pose = np.mean([p.pose for p in self.particles], axis=0)
        
    