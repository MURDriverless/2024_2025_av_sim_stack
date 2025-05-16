Path Generation with Delaunay Triangulation
===========================================

Welcome to the **Path Generation Tutorial**!

In this section, you'll learn how we extract a raceable centerline path from a cloud of cones using a computational geometry technique called **Delaunay triangulation**. Don't worry — it's way cooler than it sounds.

.. contents::
   :local:
   :depth: 2

What Are We Trying to Do?
----------------------------

Given a set of yellow and blue cones from LiDAR, we want to find the **centerline path** between them so the vehicle knows where to drive. It's like creating a yellow-brick road... except it's floating in your mind (or code).

.. image:: ../_static/tut_dt.png
   :alt: Target centerline generation
   :align: center
   :width: 100%

Delaunay Triangulation
-------------------------

Our first tool: `scipy.spatial.Delaunay`

This algorithm connects points to form triangles such that no point is inside the circumcircle of any triangle. Here's how we use it:

.. code-block:: python

   cone_positions = np.array([cone.get_position() for cone in self.forward_cones])
   tri_vis = Delaunay(cone_positions)

This gives us a network of triangles spanning all cones in sight. But not all triangles are useful...

Filtering Valid Triangles
----------------------------

We *only* keep triangles that look like part of the track.

.. code-block:: python

   for simplex in tri_vis.simplices:
       colors = [self.forward_cones[i].color for i in simplex]
       if ('yellow' in colors and 'blue' in colors) or (colors.count('orange') == 2):
           valid_triangles.append(simplex)

This skips triangles with cones all of the same color — those are likely from the same side of the track.

Constructing Midpoints
--------------------------

From each valid triangle, we extract **edges** that cross the track (i.e. one yellow + one blue cone). We take their midpoint as part of the path:

.. code-block:: python

   midpoint = (
       (cone1.x + cone2.x) / 2.0,
       (cone1.y + cone2.y) / 2.0
   )
   midpoints.append(midpoint)

These midpoints form our *centerline path*. Your car will eventually follow this!

Visualize Your Midpoints
----------------------------

Try plotting your midpoints as dots overlaid on the cones:

.. code-block:: python

   xs, ys = zip(*self.path)
   plt.scatter(xs, ys, c='green', label='Path Midpoints')
   plt.axis('equal')
   plt.legend()
   plt.show()

Interactive Checkpoint
--------------------------

Here’s a fun mini-challenge!

#. Modify the filtering criteria to:
   - Only accept triangles with **exactly 1 yellow, 1 blue, and 1 orange** cone.
   - Bonus: Filter out midpoints that are **less than 2 meters** from any orange cone.

#. Print how many midpoints you generate each frame:
   .. code-block:: python

      print(f"Frame {self.i}: {len(self.path)} midpoints generated")

Persistent Path Building
----------------------------

We don't throw away midpoints from previous frames — that would be wasteful. We store all midpoints we've ever seen in:

.. code-block:: python

   self.seen_path.add(midpoint)

This is helpful for SLAM or loop closure, and avoids flickering paths from noisy sensor data.

Summary
----------

You've just learned:

- What Delaunay triangulation is
- How we extract a driveable path using cone pair midpoints
- Why filtering and midpoint tracking matter

This centerline will become the **target trajectory** for the path following algorithm (coming soon in the Pure Pursuit tutorial!).

Want More?
----------

Try extending this:

- Visualize edges used vs rejected (`valid_edges` vs `invalid_edges`)
- Animate how the path builds up over time
- Use a confidence metric: cones seen more than N times are more trustworthy

Stay tuned for: 🏎️ **Path Following with Pure Pursuit**

