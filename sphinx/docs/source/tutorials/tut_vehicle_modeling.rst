Vehicle Modeling Tutorial: Physics on Wheels
============================================

Welcome to the vehicle modeling tutorial — where code meets physics to simulate the dynamics of our autonomous car! In this module, you'll explore:

- The **Car** class and how it simulates real-world dynamics
- The **State** class and how the car’s brain tracks motion
- The transition between **kinematic** and **dynamic** modeling
- Integration with control inputs

.. contents::
   :local:
   :depth: 2

What Is the `Car` Class?
---------------------------

Our `Car` class is a subclass of `Vehicle`, and it's the core of our simulation. It encapsulates the car's physics, sensors, controllers, and interactions with the world.

.. code-block:: python

   car = Car(world, track, car_name="MUR Autonomous Mk1", state)

The car is initialized with:
- The `world` and `track` it belongs to
- A `car_name` to identify it
- An initial `State`, which should default to the track’s start line

The `State` Class: Memory of the Machine
------------------------------------------

The `State` object keeps track of everything about the car:

.. code-block:: python

   from state import State
   state = State([x, y, yaw, u, v, w, delta])

Attributes:

- `x, y`: Position in world coordinates
- `yaw`: Heading angle (radians)
- `u`: Forward velocity (longitudinal)
- `v`: Lateral velocity
- `w`: Yaw rate
- `delta`: Steering angle

Access methods:

- `get_state()` → Returns a NumPy array of the state
- `print_state()` → Nicely prints values
- `__repr__()` → Human-readable string

.. rubric:: Try This:

Print your car's state every 50 frames:

.. code-block:: python

   for i in range(1000):
      if i % 50 == 0:
          car.state.print_state()

Vehicle Parameters
----------------------

The car is parameterized with physical constants:

+----------------+-------------------------------+
| Property       | Value                         |
+================+===============================+
| Mass           | 240 kg                        |
+----------------+-------------------------------+
| lf (front)     | 0.722 m                       |
+----------------+-------------------------------+
| lr (rear)      | 0.888 m                       |
+----------------+-------------------------------+
| Inertia `Iz`   | 1500 kg·m²                    |
+----------------+-------------------------------+
| Max Steering   | ±30°                          |
+----------------+-------------------------------+
| Cm, Cd         | Motor/aerodynamic drag coeffs |
+----------------+-------------------------------+

These values define how your car **responds to forces** and should be representative of the actual MUR vehicle.

Kinematic and Dynamic Models
------------------------------

We will be modeling the vehicle dynamics of our autonomous vehicle with the bicycle model with non-linear tire force laws. This model is simple enough for our simulation to be solved in real-time.

The car blends two motion models, (1) a kinematic model, (2) a dynamic model. This is due to the non-linear behavior of the vehicle when it is driving at its operational limits.

1. Kinematic Model (low speeds)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assumes perfect grip, no tire forces — ideal for parking lot speeds. This is important for slow speeds at race start and in tight corners, however, this model is not suited for fast driving as it neglects the interaction between the tires and ground. The equations of motion for the kinematic model is given by the following:

.. math::

    \begin{align}
    & \dot{x} = u \cos(\theta) - v \sin(\theta) \\
    & \dot{y} = u \sin(\theta) + v \cos(\theta) \\
    & \dot{\psi} = \omega \\
    & \dot{u} = \frac{1}{m} F_x \\
    & \dot{v} = (\dot{\delta} v_x + \delta \dot{v_x}) \frac{l_r}{ l_r + l_f} \\
    & \dot{w} = (\dot{\delta} v_x + \delta \dot{v_x}) \frac{1}{ l_r + l_f} \\
    \end{align}

The dynamics of the above kinematics states is calculated within the following function:

.. code-block:: python

   def compute_state_dot_k(self, F, state):
       ...

2. Dynamic Model (high speeds)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Uses tire slip angles, lateral forces, and inertia for realism at the limit of vehicle handling. This model captures the non-linearities that become significant at high speeds or aggressive maneuvers. The full dynamic model is given by the following:

.. math::

    \begin{align}
    & \dot{x} = u \cos(\theta) - v \sin(\theta) \\
    & \dot{y} = u \sin(\theta) + v \cos(\theta) \\
    & \dot{\psi} = \omega \\
    & \dot{u} = \frac{1}{m} (F_{x,r} - F_{y,f} \sin{\delta} + m v_y r) \\
    & \dot{v} = \frac{1}{m} (F_{y,r} + F_{y,f} \cos{\delta} - m v_x r) \\
    & \dot{w} = \frac{1}{I_z} (F_{y,f} \cos{delta} - F_{y,r} l_r + \tau_{TV}) \\
    \end{align}

The dynamics of the full dynamic model is calculated within the following function:

.. code-block:: python

   def compute_state_dot_d(self, F, state):
       ...

3. Blending Factor
^^^^^^^^^^^^^^^^^^

As stated, the full vehicle model is a blend of the two above models. Depending on forward speed `u`, a **mix factor** chooses how much of each model to use, i.e. :math:`v_x \in [u_{min}, u_{max}]`. When `u` is below :math:`u_{min}`, the kinematic model is solely used and when `u` is above :math:`u_{max}`, the dynamic model is purely used; when `u` is in between, a mixture of both models are used.

.. code-block:: python

   mix_factor = min(max((u - MAX_KIN) / (MIN_DYN - MAX_KIN), 0), 1)

The full derivate of the state model is now computed from:

.. code-block:: python

   state_dot = (1 - mix_factor) * kinematic + mix_factor * dynamic


RK4 Integration
------------------

Because we want to run this in real-time and not at a specific time instance, the car updates its state using **Runge-Kutta 4th order integration**. More specifically, it is primarily used for numerically solving the differential equations from the vehicle model. These ordinary differential equations (ODEs) include longitudinal/lateral dynamics, tire models, and steering and throttle/braking input effects.

.. code-block:: python

   m1 = self.compute_state_dot(self.state)
   ...
   self.state += state_var

Therefore, this provides stable and accurate motion updates, especially through curves.

Control Inputs
------------------

Each update step accepts an `Input` object:

.. code-block:: python

   from state import Input
   control = Input(F=400, delta_dot=0.05)
   car.update(control)

Inputs:

- `F`: Throttle/brake force (in Newtons)
- `delta_dot`: Steering rate (rad/s)

Try This:
^^^^^^^^^^

Create a tiny test loop:

.. code-block:: python

   for i in range(100):
       car.update(Input(F=300, delta_dot=0.0))

Modular Add-ons
-------------------

Sensors and controllers are plug-and-play which we will go over in the following tutorials. To add these sensors in, use:

.. code-block:: python

   car.add_lidar(my_lidar)
   car.add_controller(my_controller)

The car also links modules to a `GPU` object, which manages updates to sensors and controls. It can be considered as a `helper` class and will not necessarily have its own tutorial/section.

Interactive Debug Tips
-------------------------

1. Want to log heading?

   .. code-block:: python

      print("Yaw (deg):", np.rad2deg(car.state.yaw))

2. Check steering saturation:

   .. code-block:: python

      if abs(car.state.delta) >= car.max_steer:
          print("Steering limit reached!")

Summary
----------

You’ve just built a car in code! So far, we've now learned how to:

- Modeled car motion with kinematic and dynamic equations  
- Tracked full vehicle state with the `State` class  
- Integrated inputs with RK4  
- Connected sensors and control modules  

Next up: Controller logic using **Pure Pursuit**, **LQR**, or **MPC**!

