From the LiDAR pipeline to the Path Gen Pipeline
=================================================

Recall that that we stated in the Path Generation Tutorial that the entire track is unknown to the car in the first lap, therefore it will not have any cone positions except for the cones detected by the perception pipeline.

Passing Detected Cones to the 

From the previous :doc:`tutorial<tut_perception>`, we can return the detected cones from the update function of the LiDAR:

.. code-block:: python
   
   def update(self, Track=None):
      ..
      return self.cones
