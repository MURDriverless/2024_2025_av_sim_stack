Path Generation
================

What is path generation? In the context of this pipeline, path generation—also known as *path generation*—refers to the process of computing the desired trajectory that the vehicle **should** follow. However, it's crucial to understand that just because we've generated a path doesn't mean the vehicle magically knows how to follow it.

The task of interpreting and executing this path—known as *path following*—is handled by a separate algorithm within the controls pipeline, which falls outside the scope of this discussion.

Think of it this way: our path generation pipeline is saying to the vehicle, "Here's the route we plan to take," but it's not yet telling how to steer or when to accelerate. Those details come later.

Where's the path?
------------------

Before we can even consider telling the vehicle how to steer, we first need to generate a viable path. At first glance, this might seem as simple as drawing a straight line from point A to point B. And sure, that might work—if the track were a featureless runway.

Fortunately, the FSAE driverless track is far more dynamic. It includes everything from high-speed straights to sweeping curves and tight slaloms—features that challenge the vehicle’s cornering capabilities and make the path planning problem far more engaging (and fun).

Now the question becomes, how do we draw that line (or path)?

Imagine you're an autonomous car on an FSAE track. You wake up, blink your LiDAR eyes, and all you see are a bunch of blue cones to your left and yellow cones to your right. You don't know where the track goes, but you do know: "follow the road, not the cones."

Enter the *Path Generation Pipeline* — your inner map-maker, part artist, part algorithm. Its job? Take these raw, unstructured, disorganized cone observations and stitch them into a smooth, followable path that avoids embarrassment at turn 1.

.. note::

   The FSAE track is a bounded environment — blue cones on the left, yellow on the right, no surprises in the middle. This consistent coloring is your best friend when it comes to computational sanity.

Delaunay Triangulation: Not Just Fancy Geometry

-----------------------------------------------

To make sense of the chaos, we employ *Delaunay Triangulation*. No, it's not a French dessert. It's a computational geometry technique that creates triangles from scattered points (cones in our case) such that no point lies inside the circumcircle of any triangle.

Why Delaunay?

- It's robust to noise (perfect for those occasional mis-detected cones).
- It avoids skinny triangles, which means better spatial representation.
- Most importantly: it gives us an implicit connectivity between cones — even though they’re just point clouds to begin with.

Here's the clever bit: by triangulating all the cones together — left and right — and filtering triangles that connect *one left and one right cone*, we reveal the hidden *midpoints*. These midpoints are the ghostly centerline the cones never explicitly gave us. They form the **ideal path** our vehicle should follow.

.. code-block:: python

   # Pseudo-snippet for triangulated midpoint generation
   for triangle in delaunay_triangles:
       if triangle_has_one_blue_and_one_yellow_cone(triangle):
           midpoint = average(blue_cone, yellow_cone)
           path_points.append(midpoint)

Think of it like this: every blue-yellow triangle gives us a bridge from the left boundary to the right, and the midpoint of that bridge is where we want to be — balanced, elegant, and fast.

Midpoint Filtering and Smoothing

--------------------------------

But raw midpoints are like rough diamonds — uncut and possibly out of order. We need to:

1. **Order** the midpoints to form a continuous path. This is done using nearest-neighbor traversal or heuristics based on vehicle pose.
2. **Smooth** the path using splines or moving averages — because while our car loves racing, it hates jerky steering.

The output? A continuous, smooth centerline path that navigates right through the middle of the track — the best compromise between aggression and control.

.. figure:: ../tutorials/_static/dt0.png
   :align: center
   :width: 80%

   *Visual of Delaunay triangulation and centerline extraction from cone boundaries.*

Next Steps

----------

This freshly cooked path doesn’t drive the car by itself (yet). It’s handed off to the next star of the show: the *Controls Pipeline*. But that’s a story for another page.

