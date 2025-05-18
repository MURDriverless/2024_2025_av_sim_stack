Vehicle Modeling: Anatomy of a Car
==================================

Ready to unleash your autonomous car? In this tutorial, we dive deep into the `Car` class and the **physics-based vehicle model** that powers it.

.. contents::
   :local:
   :depth: 2

Meet the `Car` Class
-----------------------

This is where the action happens. Your car is a subclass of `Vehicle`, and is initialized with:

.. code-block:: python

   car = Car(world, track, car_name="MUR Autonomous Mk1", State([x,y,yaw,u,v,w,delta]))
   # Parameters
   # world = name of the World class you've created
   # track = name of the track class you've created/generated
   # car_name = name of the autonomous car
   # State([]) = initial starting state of the vehicle

It connects to a `World`, a `Track`, and sets its initial `State` using the track's start line.

Physical Properties
-----------------------

Let’s peek under the hood. In the provided `vehicle.py` file, the car has:

- Mass in kg: 
   - `mass = 240`
- Front/Rear Lengths in meters: 
   - `lf = 1.0`, `lr = 1.9`
   - Note that this distance is from the center of mass to the front/rear wheelbase.
- Inertia in the `Z` direction: 
   - `Iz = 1500`
- Motor constant that converts the drive command to force applied at the wheel:
   - `Cm = 0.15`
- Max steering of `60°` given by: 
   - `max_steer = np.deg2rad(60)`
- Pacejka's tire formula "peak" coefficient:
   - `Bp = 10`
- Pacejka's tire formula "shape" coefficient:
   - `Cp = 1.9`
- Pacejka's tire formula "stiffness" coefficient:
   - `Dp = 1`
- Pacejka's tire formula "curvature" coefficient:
   - `Ep = 0.97`

This data feeds into the **equations of motion**.

Kinematic and Dynamic Models
-------------------------------

The car blends two motion models depending on speed:

.. code-block:: python

   mix_factor = (state.u - MAX_KIN) / (MIN_DYN - MAX_KIN)

- At low speed → uses kinematic equations
- At high speed → uses dynamic model with tire forces

.. rubric:: Try This:

Print `mix_factor` every frame:

.. code-block:: python

   print("Mix factor:", mix_factor)

Control Inputs
-----------------

The car responds to:

- `F`: throttle/brake force
- `delta_dot`: rate of change of steering

Apply inputs like this:

.. code-block:: python

   from state import Input
   control = Input(F=400, delta_dot=0.02)
   car.update(control)

Integration: RK4
--------------------

State updates are computed using 4th-order Runge-Kutta (RK4). This gives accurate motion even in tight corners or high speeds.

.. code-block:: python

   m1 = compute_state_dot(...)
   ...
   state_var = (1/6) * (m1 + 2*m2 + 2*m3 + m4)

Add-ons: Sensors and Controllers
-----------------------------------

Attach autonomy modules like this:

.. code-block:: python

   car.add_lidar(my_lidar)
   car.add_camera(my_camera)
   car.add_controller(my_controller)

They all link through `car.gpu` — a wrapper that holds the active modules.

.. rubric:: Try This:

Create a minimal working sim:

.. code-block:: python

   car.start_controller()

Sim Loop Option:
-------------------

Try calling this in a loop for 100 timesteps:

.. code-block:: python

   for i in range(100):
       car.update(Input(F=300, delta_dot=0.0))

Summary
----------

- Car connects world, track, and sensors  
- Blends dynamic & kinematic physics  
- Modular add-ons (LiDAR, camera, controller)  
- Supports Runge-Kutta integration and curvature-aware steering  

Next up: Lidar Perception and Path Planning!

