Pathing
#######

What is pathing? In the context of this pipeline, pathing—also known as *path generation*—refers to the process of computing the desired trajectory that the vehicle **should** follow. However, it's crucial to understand that just because we've generated a path doesn't mean the vehicle magically knows how to follow it.

The task of interpreting and executing this path—known as *path following*—is handled by a separate algorithm within the controls pipeline, which falls outside the scope of this discussion.

Think of it this way: our pathing pipeline is saying to the vehicle, "Here's the route we plan to take," but it's not yet telling how to steer or when to accelerate. Those details come later.

Let's Draw a Line
=================

Before we can even consider telling the vehicle how to steer, we first need to generate a viable path. At first glance, this might seem as simple as drawing a straight line from point A to point B. And sure, that might work—if the track were a featureless runway.

Fortunately, the FSAE driverless track is far more dynamic. It includes everything from high-speed straights to sweeping curves and tight slaloms—features that challenge the vehicle’s cornering capabilities and make the path planning problem far more engaging (and fun).

Now the question becomes, how do we draw that line (or path)?