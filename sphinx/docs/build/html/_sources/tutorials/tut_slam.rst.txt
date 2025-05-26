SLAM & FastSLAM 2.0 Tutorial
============================

Welcome to the world of **SLAM** — Simultaneous Localization and Mapping — and its fast-paced cousin **FastSLAM 2.0**.

This tutorial walks you through:

- What SLAM is and why autonomous vehicles need it
- How **FastSLAM 2.0** works with particles and Kalman filters
- How we implement SLAM in our system using `fastslam2.py` and `gpu.py`
- How to visualize, debug, and interact with particles and landmarks

.. contents::
   :local:
   :depth: 2

What Is SLAM?
------------------

**SLAM (Simultaneous Localization and Mapping)** is a core problem in robotics:

> "Build a map of the world… while figuring out where you are in it."

In the context of FSAE Driverless, the world is defined by **cones**. We must:

- Detect cones using sensors (LiDAR, camera)
- Estimate our car’s pose (x, y, yaw)
- Build a consistent map of cone positions
- Update both **in real-time**, with **no prior knowledge**

SLAM is hard because of:

- Sensor noise
- Limited field of view
- Uncertainty in vehicle motion

FastSLAM 2.0: SLAM with Particles
--------------------------------------

FastSLAM 2.0 is a modern SLAM algorithm that uses:

- **Particles** to represent possible car poses
- Each particle carries its **own map** of landmarks (cones)
- Landmark estimates are refined using **Extended Kalman Filters (EKF)**

Think of it as 5 little robots inside your car, each guessing where it is — and where the cones are!

Particle Class
------------------

Particles are defined in `fastslam2.py`:

.. code-block:: python

   class Particle:
       def __init__(self, x, y, yaw, car):
           self.state = State([x, y, yaw, 0, 0, 0, 0])
           self.lm = []       # landmark positions
           self.lmP = []      # landmark covariances
           self.w = 1.0 / N_PARTICLE

Each particle stores:

- Car's Pose estimate (`state`)
- Landmark estimates (`lm`)
- Uncertainty for each landmark (`lmP`)
- Importance weight (`w`)

.. rubric:: Try This:

Print all particle positions at frame 0:

.. code-block:: python

   for p in particles:
       print(p.state)

SLAM Update Steps
---------------------

At each frame:

1. **Predict Particle Motion**:
    We use a noisy motion model:

   .. code-block:: python

      noisy_u = u + random_noise
      x_pred = motion_model(p.state, noisy_u)

   The prediction includes:

   - **u**: forward velocity
   - **v**: lateral slip
   - **w**: yaw rate

   Blended with ground truth using **confidence-weighted smoothing**:
   
   .. code-block:: python
   
      p.state.x = (1 - alpha) * pred + alpha * truth

2. **Landmark Association**:
   Each observation is either matched to an existing landmark or added as a new one.

3. **EKF Update**:
   If matched, we update the landmark with a Kalman filter.

4. **Proposal Sampling**:
   We update the pose of the particle based on observation likelihood.

5. **Resampling**:
   If particle weights diverge, we resample the best ones.

   .. code-block:: python

      if n_eff < threshold:
          resample()

How SLAM Is Used in `GPU`
----------------------------

The `GPU` class in `gpu.py` handles the system-level orchestration:

.. code-block:: python

   self.particles = [fs2.Particle(...)]
   self.control_ekf = fs2.ControlEKF(...)

Inside `gpu.update()`:

.. code-block:: python

   if i % 50 == 0:
       self.lidar_data = self.lidar.update()
       z = [range, bearing]  # from cones

       u_avg = average_velocities
       self.particles = fast_slam2(self.particles, u_avg, z, self.car.state)

       x_est = calc_final_state(self.particles)
       self.state_e = x_est

.. rubric:: Try This:

Visualize how particles spread as noise increases. What if `noise_std` is doubled?

Landmark Observation & Association
--------------------------------------

Measurements from LiDAR are transformed into polar format:

.. code-block:: python

   z = [range, bearing]  # relative to car

For each particle:
1. Try to associate observed cones to known landmarks (via Mahalanobis distance)
2. Update that landmark via Kalman filter, or
3. Add a new landmark if no match found

.. code-block:: python

   if best_lm_id is not None:
       update_landmark(...)
   else:
       add_new_lm(...)

.. rubric:: Try This:

Print when a new landmark is added:

.. code-block:: python

   print(f"Added landmark at step {i}")

Resampling
-------------

Particles are resampled using **low variance resampling** when the effective number of particles is low:

.. code-block:: python

   n_eff = 1.0 / sum(w^2)
   if n_eff < threshold:
       resample()

Only the **best-fit** particles survive

Visualizing SLAM
---------------------

Particles and landmarks are plotted in:

.. code-block:: python

   gpu.plotting()

It includes:
- Car trajectory
- Particle cloud (colored by weight)
- Estimated landmarks (black Xs)
- Target and current pose

.. rubric:: Try This:

Add this inside plotting to color the best particle:

.. code-block:: python

   best = max(particles, key=lambda p: p.w)
   plt.plot(best.state.x, best.state.y, 'r*', label='Best Particle')

Tips for Tuning
------------------

- Increase `N_PARTICLE` for better accuracy
- Tune `Q` and `R` matrices for realistic sensor noise
- Use Mahalanobis threshold to control association strictness

.. rubric:: Try This:

Double `Q` (observation noise) and see how landmark uncertainty grows:

.. code-block:: python

   Q = np.diag([0.2, np.deg2rad(2.0)]) ** 2

Summary
----------

You now understand:

- What SLAM is and why we use it  
- How FastSLAM 2.0 tracks pose and cones in parallel  
- How each particle carries its own map  
- How we run SLAM in real-time with GPU and LiDAR  
- How to visualize and tune your SLAM system

Next up: Use your SLAM-estimated map in path planning and autonomous laps!

