import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

from state import State
import path_generator
from cone import Cone
from wheel_encoder import Wheel_encoder

import fastslam2 as fs2

#from fastslam import Particle, initialize_particles, resample_particles

timesteps = 500000

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


    def update(self):

        if not self.start:

            if self.lidar and not self.car.lidar:
                self.lidar_data = self.lidar.update()
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

            if (not self.car.lap_finish) and (self.lidar) and (i % 50 == 0):
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
        
    