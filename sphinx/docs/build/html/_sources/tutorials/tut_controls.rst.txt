Path Following with Pure Pursuit
================================

Welcome to the **Path Following** tutorial, featuring the star of our show: **Pure Pursuit Control**.

By the end of this tutorial, you’ll understand:
- What Pure Pursuit is
- How it selects target points
- How it generates steering + throttle commands
- How to tune its behavior for different racing styles

.. contents::
   :local:
   :depth: 2

What is Pure Pursuit?
------------------------

Imagine you are holding a stick and trying to chase a laser dot on the floor. You always aim the stick toward that dot. That's *Pure Pursuit*!

It finds a target point some distance ahead on the path, and steers the car to follow a circular arc toward it.

How Does It Work?
--------------------

Pure Pursuit boils down to two main steps:

#. **Find the target point** — the point within lookahead distance and in front of the car.
#. **Compute a turning curvature** — the arc the car should follow to reach the target.

This is implemented in:

.. code-block:: python

   def find_target_point(self, position, heading):
       ...
       # Returns a point ahead of the vehicle

   def compute_steering_rate(self, position, heading):
       ...
       # Uses target to compute curvature and steering

Controller Framework
------------------------

At the base of our control system is an abstract `Controller` class:

.. code-block:: python

   class Controller:
       def __init__(self):
           self.car = None
           self.time = 0.0

       def set_car(self, car):
           self.car = car

       def set_path(self, path):
           raise NotImplementedError()

       def pursue(self):
           raise NotImplementedError()

The purpose of this class is to define a common interface:

- `set_car()` attaches the car to the controller.
- `set_path()` defines the reference trajectory.
- `pursue()` returns the control commands at each timestep.

Any controller must implement `set_path()` and `pursue()`.

Enter: PurePursuit
----------------------

The `PurePursuit` class inherits from `Controller` and implements a geometric steering algorithm that uses a lookahead point.

.. code-block:: python

   class PurePursuit(Controller):
       def __init__(self, lookahead_distance, u_max, k_speed_c, k_throttle):
           self.lookahead_distance = lookahead_distance
           self.u_max = u_max
           self.k_speed_c = k_speed_c
           self.k_throttle = k_throttle
           ...

Constructor Parameters:
- `lookahead_distance`: how far ahead the target point should be.
- `u_max`: maximum velocity.
- `k_speed_c`: controls how much curvature slows the car.
- `k_throttle`: controls how aggressively the car accelerates.

Step 1: Finding the Target Point
-----------------------------------

We check all path points to see if they are:

- Within the **lookahead distance**
- Inside a **±85° field of view**

The furthest such point is selected as the target.

.. code-block:: python

   if angle_deg <= 85 and distance <= self.lookahead_distance:
       target_point = point

Try this:
^^^^^^^^^^

Set `lookahead_distance = 5.0` vs `2.0`. How does the car's responsiveness change?

Step 2: Curvature to Steering
--------------------------------

Once we have a target point, we compute the steering angle using circle geometry:

.. math::

   \kappa = \frac{2y}{L^2}

.. code-block:: python

   curvature = (2 * local_y) / L2
   desired_delta = atan(wheelbase * curvature)

This angle is then **rate-limited** to prevent crazy steering:

.. code-block:: python

   delta_dot = np.clip(..., -max_delta_dot, +max_delta_dot)

Try this:
^^^^^^^^^^

Modify `max_delta_dot` in radians per second. Try values like:

.. code-block:: python

   self.max_delta_dot = np.deg2rad(30)  # smooth
   self.max_delta_dot = np.deg2rad(90)  # snappy

Throttle Control
-------------------

Speed is reduced when curvature is high (tight turns). This ensures safety and stability:

.. code-block:: python

   velocity = u_max / (1 + k_speed_c * abs(curvature))

Then we calculate throttle force with a simple proportional controller:

.. code-block:: python

   F = k_throttle * (desired_u - current_u)

Try this:
^^^^^^^^^^

Play with these parameters:

- `k_speed_c = 5.0` → slows down more in corners
- `k_throttle = 500.0` → accelerates faster

Full Control Loop
--------------------

All of this is wrapped inside the `pursue()` function:

.. code-block:: python

   # need to redo

This returns a `throttle` and `steering rate` command for the car to execute.

Interactive Tuning Table
----------------------------

Try these settings for different racing personalities:

+----------------+-------------------+------------------+-------------------+
| Style          | Lookahead (m)     | Curvature Gain   | Throttle Gain     |
+================+===================+==================+===================+
| Smooth & Safe  | 4.0               | 5.0              | 300               |
+----------------+-------------------+------------------+-------------------+
| Aggressive     | 2.5               | 2.5              | 800               |
+----------------+-------------------+------------------+-------------------+
| Test Only      | 1.5               | 0.5              | 150               |
+----------------+-------------------+------------------+-------------------+

Bonus: Plot Your Path & Targets
----------------------------------

You can modify the `plotting()` method in `Car` to show the current target:

.. code-block:: python

   plt.scatter(controller.target[0], controller.target[1], c='red', marker='x', label="Target")

Summary
----------

In this tutorial, you learned:

- What Pure Pursuit is and how it works  
- How to find the target point and compute steering  
- How to dynamically adjust speed with curvature  
- How to tune your controller for different behaviors

Next up: Advanced control methods like **LQR** and **Model Predictive Control (MPC)** — but Pure Pursuit is already race-ready!
