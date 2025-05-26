Lidar Perception Pipeline Tutorial
==================================

Welcome to the **LiDAR Perception Tutorial** — your first step into seeing the world of autonomous perception!

This tutorial is designed to walk you through how our simulated LiDAR system detects cones, estimates their positions, and feeds into the Path Generation and the SLAM pipelines.

.. contents::
   :local:
   :depth: 2

Setting the Stage
--------------------

Our vehicle is equipped with a **LiDAR sensor** mounted at the front. It spins in an arc, casting rays and measuring distances to detect obstacles — in our case, **cones**.

.. code-block:: python

   from lidar import Lidar
   lidar_sensor = Lidar(pos=[0,0], yaw=0, pos_c=[0,0], range_min=0.1, range_max=10.0, angle_min=-np.deg2rad(80), angle_max = np.deg2rad(80),resolution=math.pi/500, fps=30)

This initializes the LiDAR system by creating a LiDAR object. The LiDAR itself can be customized by changing it's default parameters.

+-----------------------------------------------------------------------+-------------------------------------------------------------+
| **Parameter**                                                         | **Description**                                             |
+=======================================================================+=============================================================+
| **Position in the global frame**:math:`\ [pos]`                       | Sets the LiDAR’s position on the car.                       |
+-----------------------------------------------------------------------+-------------------------------------------------------------+
| **Yaw relative to the heading of the car**:math:`[yaw]`               | Orientation of the LiDAR relative to the heading of the car.|
+-----------------------------------------------------------------------+-------------------------------------------------------------+
| **Position on the car relative to the center of mass**: [`pos_c`]     | Sets the local coordinate frame origin of the LiDAR.        |
+-----------------------------------------------------------------------+-------------------------------------------------------------+
| **Range** :math:`\in [range_{min}, range_{max}]`                      | 0.1 m to 10.0 m                                             |
+-----------------------------------------------------------------------+-------------------------------------------------------------+
| **Field of View** :math:`\in [angle_{min}, angle_{max}]`              | ±80 degrees                                                 |
+-----------------------------------------------------------------------+-------------------------------------------------------------+
| **Resolution** :math:`[resolution]`                                   | π/200 radians                                               |
+-----------------------------------------------------------------------+-------------------------------------------------------------+

Each frame, the LiDAR "scans" and attempts to detect cones around the car.

How LiDAR "Sees"
--------------------

The key perception happens in :py:meth:`Lidar.sense_obstacle_fast`.

.. code-block:: python

   def sense_obstacle_fast(track):
      ...

This casts out virtual rays in the car's heading and checks whether they hit cone-like objects using a geometric trick — ray-circle intersection! This also adds additional sensor noise (range + angle) to simulate realism.

Ray-Circle Intersection Explained
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   def ray_circle_intersection(self, o, d, c, r):
       ...

Inside :py:meth:`sense_obstacle_fast()`, the function :py:meth:`ray_circle_intersection` is called. This function determines if a ray (origin `o`, direction `d`) intersects a cone modeled as a circle (`c`, radius `r`). If an intersection exists within LiDAR range, it's added to the `sense_data`.

From Points to Clusters
-----------------------------

The LiDAR doesn’t "see cones" — it sees scattered points on a pointcloud. To extract cones, we use **clustering**.

.. code-block:: python

   def fast_euclidean_clustering(self, distance_threshold=0.5, min_cluster_size=1):
      ...
      return clusters

We group nearby points using a fast KD-tree-based clustering method. Each cluster should (ideally) correspond to one cone.

Estimating Cone Centers
--------------------------

Once clusters are found, we filter out small clusters (e.g. noise) below a size threshold:

.. code-block:: python

   def get_filtered_clusters_from_clusters(self, clusters, min_cluster_size=3):
      return [cluster for cluster in clusters if len(cluster) >= min_cluster_size]

and once noisy clusters are filter out, we estimate where the cone *actually* is:

.. code-block:: python

   def estimate_cone_center(self, cluster_indices, known_radius=0.1):
      ...

Each cluster's center is estimated depending on the number of points:
- 1 point → backtracks the raw to estimate cone center
- 2 points → midpoint + perpendicular offset
- 3+ points → uses least-squares circle fitting with known radius

Creating Detected Cones
---------------------------

We convert cluster centers + color back to usable cone objects:

.. code-block:: python

   def get_detected_cones(self, clusters, cone_pos):
      ...
      return detected_cones

Each `Cone` has an estimated `(x, y)` and color. It's important to note that the coordinates of the detected cones are represented in the **local frame** of the car.

Visualizing the LiDAR Scan
-----------------------------

Want to *see* what LiDAR sees? Use:

.. code-block:: python

   lidar_sensor.plot_lidar()

.. note::
   Ensure that you've created the world, track, and car objects, and that the lidar object has been added to the vehicle using: ``car_name.add_lidar(lidar_sensor)``

This opens a dual view:

- **Left:** Cartesian rays (each ray = one LiDAR reading)
- **Right:** Polar plot of cone clusters

.. image:: ../_static/tut_lidar_plot.png
   :alt: Example LiDAR plot
   :align: center
   :width: 100%

Interactive Exercise: Play with Parameters
--------------------------------------------

Try modifying these values and re-running the simulation:

1. **Resolution** — What happens if you make `resolution = math.pi/50`?
2. **Range** — What if `range_max = 5.0`?
3. **Noise** — What if you add random noise to cone positions?

Challenge: Add a Dynamic Obstacle
------------------------------------

Extend the `sense_obstacle_fast()` function to ignore cones moving faster than 0.5 m/s.

Summary
----------

In this tutorial, you learned how our LiDAR pipeline:

- Casts rays to detect obstacles
- Groups hits into cone-like clusters
- Estimates cone positions and returns them for use in other pipelines

