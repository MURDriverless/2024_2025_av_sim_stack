FastSLAM 2.0 Tutorial
=====================

Welcome to the world of **FastSLAM 2.0** – where your autonomous car learns the track as it drives! In this tutorial, you’ll:

- Understand how particles track both **pose** and **landmarks**
- Explore `fastslam2.py`, our core implementation of FastSLAM 2.0
- See how `GPU` connects the SLAM system to the car in `gpu.py`

.. contents::
   :local:
   :depth: 2

📦 What is FastSLAM 2.0?
------------------------

FastSLAM 2.0 is a probabilistic localization and mapping algorithm using:
- **Particles** to estimate the car’s pose
- **Extended Kalman Filters** inside each particle to estimate landmark positions

Unlike traditional SLAM, each particle carries its own private map 🧠.

✨ Particle Structure
----------------------

Particles are defined in `fastslam2.py`:

.. code-block:: python

   class Particle:
       def __init__(self, x, y, yaw, car):
           self.state = State([x, y, yaw, 0, 0, 0, 0])
           self.lm = []       # list of landmark positions [x, y]
           self.lmP = []      # list of landmark covariances
           self.w = 1.0 / N_PARTICLE
           self.car = car

Each particle contains:
- `state`: the car’s estimated pose
- `lm`: landmark locations
- `lmP`: uncertainties for each landmark

Try This:
^^^^^^^^^^

Print all particle poses at timestep 0:

.. code-block:: python

   for p in particles:
       print(p.state)

🚗 Predicting Particle Motion
-----------------------------

We use a noisy motion model:

.. code-block:: python

   noisy_u = u + np.random.randn(3, 1) * noise_std
   x_pred = motion_model(p.state, noisy_u)

The prediction includes:
- **u**: forward velocity
- **v**: lateral slip
- **w**: yaw rate

Blended with ground truth using **confidence-weighted smoothing**:

.. code-block:: python

   p.state.x = (1 - alpha) * pred + alpha * truth

Try This:
^^^^^^^^^^

Visualize how particles spread as noise increases. What if `noise_std` is doubled?

📡 Landmark Observation & Association
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

Try This:
^^^^^^^^^^

Print when a new landmark is added:

.. code-block:: python

   print(f"Added landmark at step {i}")

🔄 Resampling
-------------

Particles are resampled using **low variance resampling** when the effective number of particles is low:

.. code-block:: python

   n_eff = 1.0 / sum(w^2)
   if n_eff < threshold:
       resample()

Only the **best-fit** particles survive 🔥

🧠 In `gpu.py`: How GPU Uses FastSLAM
--------------------------------------

The `GPU` class initializes SLAM like this:

.. code-block:: python

   self.particles = [fs2.Particle(...)]
   self.control_ekf = fs2.ControlEKF(...)

Every SLAM update loop in `GPU.update()`:
1. Sensors provide current `z` observations
2. EKF fuses car state into estimated control input `u`
3. `fast_slam2()` is called:

.. code-block:: python

   self.particles = fs2.fast_slam2(self.particles, u_avg, z, self.car.state)

4. The estimated state is computed from particles:

.. code-block:: python

   x_est = fs2.calc_final_state(self.particles)
   self.state_e = x_est

👀 Try This Visualization
--------------------------

Plot particles at runtime:

.. code-block:: python

   xs = [p.state.x for p in self.particles]
   ys = [p.state.y for p in self.particles]
   plt.scatter(xs, ys, c='gray', label='Particles')

Highlight high-weight particles in green!

📊 Bonus: Landmarks from Particles
----------------------------------

We compute mean landmark positions across particles for visualization:

.. code-block:: python

   mean_lms = np.mean([p.lm[i] for p in particles if i < len(p.lm)], axis=0)

Try This:
^^^^^^^^^^

Plot landmarks vs ground truth cones. How close are they?

📚 Summary
----------

✅ Tracked pose with FastSLAM 2.0 particles  
✅ Estimated landmark positions with EKF updates  
✅ Integrated FastSLAM into GPU → car.update() loop  
✅ Visualized particles and landmark estimates

Next Up: 🔄 Fuse this map with a controller to drive a **FastSLAM-autonomous lap!**

