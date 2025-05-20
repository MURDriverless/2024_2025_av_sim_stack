import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import matplotlib.cm as cm
from collections import Counter
from scipy.optimize import minimize
from collections import deque

from cone import Cone


class Lidar:
    def __init__(self, pos=[0,0], yaw=0, pos_c=[0,0], range_min=0.1, range_max=10.0, angle_min=-np.deg2rad(70), angle_max = np.deg2rad(70),resolution=math.pi/200, fps=30):
        
        self.car = None
        self.pos = pos
        self.yaw = yaw
        self.pos_c = pos_c

        self.sense_yaw = yaw

        self.range_min = range_min
        self.range_max = range_max
        self.angle_min = angle_min
        self.angle_max = angle_max

        self.sense_angle_min = yaw + angle_min
        self.sense_angle_max = yaw + angle_max

        self.resolution = resolution
        self.angle_num = int((self.angle_max - self.angle_min)/self.resolution)
        if self.angle_num % 2 == 0:
            self.angle_num += 1  # Ensure middle ray exists
        self.angle_num += 1  # To include both endpoints

        self.range_num = 1000
        self.fps = fps
        self.sense_data = []
        self.points = []
        self.cluster = []
        self.sense_filt = []

        self.cone_pos = []
        self.cones = []

        self.angle_noise_std = np.deg2rad(0.01)  # radians

    


    def ray_circle_intersection(self, o, d, c, r):
        # o = origin (2D), d = direction (unit vector), c = circle center, r = radius
        oc = o - c
        a = np.dot(d, d)             # should be 1 if d is normalized
        b = 2 * np.dot(oc, d)
        c_val = np.dot(oc, oc) - r*r

        discriminant = b*b - 4*a*c_val
        if discriminant < 0:
            return None  # no intersection

        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2*a)
        t2 = (-b + sqrt_disc) / (2*a)

        # Return the closest positive t in range
        t_vals = [t for t in (t1, t2) if 0 <= t <= self.range_max]
        if not t_vals:
            return None
        t_min = min(t_vals)
        return t_min * d  # intersection point


    # @profile
    def sense_obstacle_fast_cones(self, track):

        data = []
        visible_cones = []

        angles = np.linspace(self.angle_min, self.angle_max, self.angle_num, True)
        

        for angle in angles:
            data.append({
                'position': None,
                'distance': self.range_max,
                'angle': angle,
                'color': None
            })

        for cone in track.cones:

            cone = self.car.gpu.world_to_car(cone)
            
            vec_to_cone = np.array([cone.x - self.pos_c[0], cone.y - self.pos_c[1]])
            dist = np.linalg.norm(vec_to_cone)

            if dist > self.range_max:
                continue
            
            angle_to_cone = np.arctan2(vec_to_cone[1], vec_to_cone[0])

            if (self.angle_max < angle_to_cone) or (angle_to_cone < self.angle_min):
                continue
            
            #visible_cones.append(cone)
            #self.seen_cones.add(cone)

            angular_radius = np.arcsin(cone.radius / dist)
            lower_bound = angle_to_cone - angular_radius
            upper_bound = angle_to_cone + angular_radius

            candidate_indices = np.where((angles >= lower_bound) & (angles <= upper_bound))[0]

            for idx in candidate_indices:
                ray_angle = angles[idx]
                ray_dir = np.array([np.cos(ray_angle), np.sin(ray_angle)])

                hit = self.ray_circle_intersection(self.pos_c, ray_dir, np.array([cone.x, cone.y]), cone.radius)
                
                if hit is not None:
                    hit_dist = np.linalg.norm(hit)
                    if hit_dist < data[idx]['distance']:

                        # Range-dependent noise std (precision)
                        sigma_range = 0.005 + (0.03 - 0.005) * (hit_dist / self.range_max)
                        noisy_dist = hit_dist + np.random.normal(0, sigma_range)

                        # Add systematic bias once per scan
                        if not hasattr(self, "range_bias"):
                            self.range_bias = np.random.uniform(-0.025, 0.025)
                        noisy_dist += self.range_bias

                        # Angular noise
                        noisy_angle = ray_angle + np.random.normal(0, self.angle_noise_std)

                        # Compute noisy position
                        noisy_pos = noisy_dist * np.array([np.cos(noisy_angle), np.sin(noisy_angle)])

                        # Update the stored data immediately
                        data[idx]['distance'] = noisy_dist
                        data[idx]['position'] = tuple(noisy_pos)
                        data[idx]['color'] = cone.color

        self.sense_data = data
        self.points = {
            'positions': [d['position'] for d in data if d['position'] is not None],
            'indices': [i for i, d in enumerate(data) if d['position'] is not None]
        }

    # def fast_euclidean_clustering(self, distance_threshold=0.5, min_cluster_size=1):
    #     positions = self.points['positions']
    #     original_indices = self.points['indices']

    #     N = len(positions)  # ✅ Avoid repeated len()
    #     tree = cKDTree(positions)
    #     visited = np.zeros(N, dtype=bool)
    #     clusters = []

    #     for i in range(N):
    #         if visited[i]:
    #             continue

    #         queue = deque([i])  # ✅ Faster than list for queue behavior
    #         cluster = []

    #         while queue:
    #             idx = queue.pop()  # ✅ LIFO here; change to popleft() for BFS
    #             if visited[idx]:
    #                 continue
    #             visited[idx] = True
    #             cluster.append(idx)

    #             neighbors = tree.query_ball_point(positions[idx], distance_threshold)

    #             # ✅ Avoid redundant `visited[n]` checks
    #             for n in neighbors:
    #                 if not visited[n]:
    #                     queue.append(n)

    #         if len(cluster) >= min_cluster_size:
    #             # ✅ No changes here — this is already efficient
    #             original_cluster = [original_indices[j] for j in cluster]
    #             clusters.append(original_cluster)

    #     return clusters


    # @profile
    def fast_euclidean_clustering(self, distance_threshold=0.5, min_cluster_size=1):
        positions = self.points['positions']
        original_indices = self.points['indices']

        tree = cKDTree(positions)
        visited = np.zeros(len(positions), dtype=bool)
        clusters = []

        for i in range(len(positions)):
            if visited[i]:
                continue

            queue = [i]
            cluster = []

            while queue:
                idx = queue.pop()
                if visited[idx]:
                    continue
                visited[idx] = True
                cluster.append(idx)
                neighbors = tree.query_ball_point(positions[idx], distance_threshold)
                queue.extend([n for n in neighbors if not visited[n]])

            if len(cluster) >= min_cluster_size:
                # Map back to original data indices
                original_cluster = [original_indices[j] for j in cluster]
                clusters.append(original_cluster)

        return clusters

    def get_filtered_sense_data_from_clusters(self, clusters, min_cluster_size=3):
        """
        Returns a filtered list of LiDAR points from self.lidar.relative_sense_data that belong to
        clusters with at least `min_cluster_size` points.
        
        :param clusters: List of clusters (each cluster is a list of indices into sense_data)
        :param min_cluster_size: Minimum number of points a cluster must have to be kept
        :return: Filtered list of LiDAR point dictionaries
        """
        filtered_points = []

        for cluster in clusters:
            if len(cluster) >= min_cluster_size:
                for idx in cluster:
                    filtered_points.append(self.sense_data[idx])

        return filtered_points
    
    def get_filtered_clusters_from_clusters(self, clusters, min_cluster_size=3):
        """
        Returns only the clusters that have at least `min_cluster_size` points.
        
        :param clusters: List of clusters (each cluster is a list of indices)
        :param min_cluster_size: Minimum number of points a cluster must have to be kept
        :return: Filtered list of clusters
        """
        return [cluster for cluster in clusters if len(cluster) >= min_cluster_size]
    
    def plot_lidar(self):
        plt.figure(figsize=(12, 6))  # wider to fit two plots

        # --- Subplot 1: Cartesian LiDAR View ---
        ax1 = plt.subplot(1, 2, 1)

        for i, d in enumerate(self.sense_data):
            angle = d['angle']
            dx = math.cos(angle) * d['distance']
            dy = math.sin(angle) * d['distance']
            ray_end = np.array([dx, dy]) + self.pos_c  # apply offset to endpoint
            rayx = [self.pos_c[0], ray_end[0]]
            rayy = [self.pos_c[1], ray_end[1]]
            ax1.plot(rayx, rayy, '-b', linewidth=0.5)

        # Plot cone centers with dominant cluster color
        if self.clusters_filt and self.cone_pos_filt:
            for cluster, pos in zip(self.clusters_filt, self.cone_pos_filt):
                if pos is None:
                    continue

                color_counts = Counter()
                for idx in cluster:
                    sense = self.sense_data[idx]
                    if sense['position'] is not None:
                        color = sense.get('color', 'black')
                        color_counts[color] += 1

                dominant_color = color_counts.most_common(1)[0][0] if color_counts else 'black'
                circle = plt.Circle(pos, 0.1, color=dominant_color, fill=True, ec=None)
                ax1.add_patch(circle)

        if self.car:
            ax1.scatter(0, 0, c='purple', marker='s', label="Car")
            ax1.scatter(self.pos_c[0], self.pos_c[1], c='cyan', marker='s', label="Lidar")

        ax1.set_title("LiDAR Vision (Cartesian)")
        ax1.set_aspect('equal')
        ax1.legend()

        # --- Subplot 2: Polar Cluster View ---
        self.plot_detections(self.clusters, subplot_index=122)

        plt.tight_layout()
        plt.show()



    def plot_detections(self, clusters, subplot_index=111):
        """
        Plot LiDAR detections grouped by clusters (now using original indices into self.sense_data).
        """
        import matplotlib.pyplot as plt
        import matplotlib.cm as cm
        import numpy as np

        # Generate a color for each cluster
        colors = cm.rainbow(np.linspace(0, 1, len(clusters)))

        ax = plt.subplot(subplot_index, polar=True)

        for cluster, color in zip(clusters, colors):
            cluster_angles = []
            cluster_distances = []
            for idx in cluster:
                d = self.sense_data[idx]
                if d['position'] is not None:
                    cluster_angles.append(d['angle'])
                    cluster_distances.append(d['distance'])

            ax.scatter(cluster_angles, cluster_distances, s=10, color=color,
                    label=f'Cone {clusters.index(cluster)}')

        ax.set_title("Clustered LiDAR Scan (Polar)")
        ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))


    @staticmethod
    def circle_from_3_points(z1:complex, z2:complex, z3:complex) -> tuple[complex, float]:
        if (z1 == z2) or (z2 == z3) or (z3 == z1):
            raise ValueError(f"Duplicate points: {z1}, {z2}, {z3}")
            
        w = (z3 - z1)/(z2 - z1)
        
        # You should change 0 to a small tolerance for floating point comparisons
        if abs(w.imag) <= 0:
            raise ValueError(f"Points are collinear: {z1}, {z2}, {z3}")
            
        c = (z2 - z1)*(w - abs(w)**2)/(2j*w.imag) + z1  # Simplified denominator
        # r = abs(z1 - c)
        
        return c     # , r
    
    # @staticmethod
    # def fit_circle_fixed_radius(points: np.ndarray, known_radius: float) -> np.ndarray:
    #     """
    #     Estimate circle center (x, y) from noisy 2D points, given a known radius.
    #     Uses nonlinear least squares minimization.
    #     """
    #     def objective(center):
    #         dists = np.linalg.norm(points - center, axis=1)
    #         return np.sum((dists - known_radius)**2)

    #     # Initial guess: centroid
    #     initial = np.mean(points, axis=0)
    #     result = minimize(objective, initial, method='BFGS')

    #     if result.success:
    #         return result.x  # center (x, y)
    #     else:
    #         return None
        
    @staticmethod
    def fit_circle_fixed_radius(points: np.ndarray, known_radius: float) -> np.ndarray:
        """
        Fast approximate circle center estimate given known radius.
        Fits a center such that all points lie ~radius from it using algebraic averaging.
        """
        if len(points) < 2:
            return None

        # Compute pairwise midpoints and normals
        centers = []
        for i in range(len(points)):
            for j in range(i+1, len(points)):
                p1, p2 = points[i], points[j]
                midpoint = (p1 + p2) / 2
                d = np.linalg.norm(p2 - p1)

                if d == 0 or d > 2 * known_radius:
                    continue  # Cannot form valid circle

                h = np.sqrt(known_radius**2 - (d / 2)**2)  # height to center
                direction = (p2 - p1) / d
                normal = np.array([-direction[1], direction[0]])

                # Two possible centers
                center1 = midpoint + h * normal
                center2 = midpoint - h * normal
                centers.append(center1)
                centers.append(center2)

        if not centers:
            return None

        return np.mean(centers, axis=0)  # Average for robustness


    @staticmethod
    def fit_circle_from_two_points(p1: np.ndarray, p2: np.ndarray, radius: float, lidar_origin=np.array([0, 0])) -> np.ndarray:
        """
        Given two points and a known radius, return the more plausible circle center.
        Assumes the correct center is farther from the LiDAR origin.
        """
        midpoint = (p1 + p2) / 2
        chord_vec = p2 - p1
        chord_len = np.linalg.norm(chord_vec)
        
        if chord_len > 2 * radius:
            return None  # No circle possible

        # Perpendicular direction
        norm_vec = np.array([-chord_vec[1], chord_vec[0]]) / chord_len
        h = np.sqrt(radius**2 - (chord_len / 2)**2)

        # Two candidate centers
        c1 = midpoint + h * norm_vec
        c2 = midpoint - h * norm_vec

        # Pick the center farther from the LiDAR
        dist1 = np.linalg.norm(c1 - lidar_origin)
        dist2 = np.linalg.norm(c2 - lidar_origin)
        return c1 if dist1 > dist2 else c2

    

    # @profile
    def estimate_cone_center(self, cluster_indices, known_radius=0.1):
        """
        Estimate the center of a cone from clustered LiDAR point indices (original indices into self.sense_data).
        """
        # Extract positions from sense_data using original indices
        cluster = np.array([self.sense_data[i]['position'] for i in cluster_indices if self.sense_data[i]['position'] is not None])
        n = len(cluster)

        if n == 0:
            return None

        elif n == 1:
            idx = cluster_indices[0]
            sense = self.sense_data[idx]
            pos_hit = np.array(sense['position'])
            ray_angle = sense['angle']

            # Compute the ray direction (relative to pos_c)
            ray_dir = np.array([np.cos(ray_angle), np.sin(ray_angle)])

            # Shift back along the ray by known_radius to approximate the center
            estimated_center = pos_hit - known_radius * ray_dir

            return estimated_center + self.pos_c

        elif n == 2:
            p1, p2 = cluster
            center = self.fit_circle_from_two_points(p1, p2, known_radius, lidar_origin=np.array(self.pos_c))
            return center if center is not None else None


        else:
            # first = complex(*cluster[0])
            # middle = complex(*cluster[len(cluster) // 2])
            # last = complex(*cluster[-1])
            # pos = self.circle_from_3_points(first, middle, last)  # returns complex center and radius
            # return np.array([pos.real, pos.imag]) + self.pos_c
            center = self.fit_circle_fixed_radius(cluster, known_radius)
            if center is not None:
                return center + self.pos_c
            else:
                return None

        

    # @profile
    def get_detected_cones(self, clusters, cone_pos):
        """
        Converts self.cone_pos and cluster color info into a list of Cone objects.
        Assumes self.clusters holds indices into self.sense_data.
        """
        detected_cones = []

        if clusters and cone_pos:
            for cluster, pos in zip(clusters, cone_pos):
                if pos is None:
                    continue

                # Count dominant color in the cluster
                color_counts = Counter()
                for idx in cluster:
                    sense = self.sense_data[idx]
                    if sense['position'] is not None:
                        color = sense.get('color', 'black')
                        color_counts[color] += 1

                dominant_color = color_counts.most_common(1)[0][0] if color_counts else 'black'
                cone = Cone(pos[0], pos[1], dominant_color)
                detected_cones.append(cone)

        return detected_cones



    def update(self, track=None):

        if self.car:
            self.pos = np.array(self.car.get_position()) + np.array([0.5*math.cos(-self.car.state.yaw), 0.5*math.sin(-self.car.state.yaw)])
            self.pos_c = np.array([0.5, 0])
            self.sense_yaw = self.yaw + self.car.state.yaw
            self.sense_angle_min = self.angle_min + self.sense_yaw
            self.sense_angle_max = self.angle_max + self.sense_yaw
            track = self.car.track
            
        self.sense_obstacle_fast_cones(track)
        self.clusters = self.fast_euclidean_clustering()
        self.clusters_filt = self.get_filtered_clusters_from_clusters(self.clusters)
        self.prev_cone_pos = self.cone_pos
        new_pos = []
        new_pos_filt = []

        for cluster in self.clusters:
            new_pos.append(self.estimate_cone_center(cluster))

        for cluster in self.clusters_filt:
            new_pos_filt.append(self.estimate_cone_center(cluster))
       
        self.cone_pos = new_pos
        self.cone_pos_filt = new_pos_filt
        self.cones = self.get_detected_cones(self.clusters, new_pos)
        self.filt_cones = self.get_detected_cones(self.clusters_filt, new_pos_filt)

        return self.cones


