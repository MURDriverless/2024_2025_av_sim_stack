Controls
########

What is controls? At its core, the **controls** pipeline is the part of the autonomous system that tells the vehicle *how* to move. It’s the system's internal coach, whispering commands to actuators placed near the steering rack, throttle, and brake—guiding every turn, acceleration, and deceleration.

But how does the vehicle know *when* to move, and by *how much*? That’s where the rest of the autonomy stack lends a hand. If you've been reading the quick start guide in order (and we hope you have), you'll recall that the **pathing** pipeline produces a desired trajectory—a virtual route that says, "Hey car, here’s where you should go."

This trajectory is handed off to the controls pipeline like a relay baton. From there, a **path-following algorithm** gets to work, computing the necessary actuation commands to make the vehicle trace that path as closely as physics (and tire friction) will allow.

Now, we say "attempt" for a reason—vehicles aren’t drawing with laser precision here. The real world is messy, and cars can’t magically teleport onto the ideal path. Instead, control algorithms strive to *minimize the deviation* between the planned trajectory and the one the car actually takes. Think of it as trying to trace a line while riding a skateboard—you may wobble a little, but the goal is to stay as close as possible.

In essence, the controls system is the final translator between intention and action—the difference between a plan and motion.

Pure Pursuit: A Technically Elegant, Mildly Obsessive Navigator
===============================================================

What path-following algorithm do we use in our architecture? Well, the title of this section answers that. Pure Pursuit is a classic geometric path tracking algorithm that behaves like a driver who’s fixated on chasing a moving dot on the road — but in a good way. In the context of FSAE Driverless, it's a fundamental method used in the low-level control layer to convert a planned path into smooth, real-time steering commands for the autonomous race car.

.. rubric:: The Core idea

At each control cycle, the algorithm selects a lookahead point on the reference path — typically a fixed or adaptive distance ahead of the vehicle. The vehicle then calculates the circular arc that would steer it from its current position to that lookahead point, and commands a steering angle that would follow that arc. Technically speaking, we have the:

- Lookahead Distance (L): This is the distance ahead of the vehicle where the target point is selected. It can be fixed or dynamically adjusted based on speed.
- Goal Point Selection: The algorithm identifies a point on the reference trajectory at a distance, L, from the vehicle's current position, in the vehicle's local coordinate frame.
- Curvature Calculation: The required curvature, k, is computed using:k = 2y/L^2 where y is the lateral offset of the lookahead point from the vehicle’s heading.
- Steering Angle Command: This curvature is then converted into a steering angle using the vehicle’s geometry (often using a bicycle model).