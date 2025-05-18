LiDAR-Based Perception Pipeline: The Laser-Focused Cone Seeker
==============================================================

While camera-based systems give you color and context, LiDAR-based perception brings raw, unfiltered truth — point clouds don't lie (they just occasionally bounce off the grass). In the high-speed, high-stakes domain of FSAE Driverless, LiDAR plays the role of the sharp-eyed, distance-measuring sentinel that turns raw spatial data into precise 3D maps of cone-dotted circuits.

.. rubric:: Point Cloud Acquisition: Painting the World in Dots

LiDAR (usually 3D, rotating, multi-layer units like Velodyne or Ouster) generates point clouds at high frequency — often 10–20 Hz. Each scan returns a dense array of points with XYZ coordinates (and sometimes intensity), effectively forming a 3D snapshot of the surroundings. Think of it as an echolocation party with lasers — except no bats, just math.

.. rubric:: Preprocessing: Cleaning Up the Noise

LiDAR data is accurate but not always clean. So we do some digital janitorial work:

- Ground Plane Removal: Using RANSAC or plane-fitting to subtract the track surface from the scene — cones are exciting, the ground is not.
- Downsampling: Voxel grid filters reduce data size without losing essential structure.
- ROI Filtering: Keep only the points within a reasonable forward and lateral field — no need to process trees 50 meters away.

.. rubric:: Cone Detection: Extracting the Spikes from the Cloud

This is where the system shifts from cloudy to clear — identifying individual cones from the remaining 3D points. Typical pipeline involves:

- Clustering: Euclidean clustering or DBSCAN is applied to group nearby points into object candidates.
- Feature Extraction: For each cluster, features like height, width, shape, and number of points are computed.
- Classification:
    - Rule-based: If it quacks like a cone (i.e., ~30 cm tall and not moving), it probably is.
    - ML-based: Use of classifiers like Random Forests or CNNs trained on 3D features (e.g. PointNet or projection-based approaches).

.. rubric:: Cone Classification: Adding Color Without a Camera

Since LiDAR doesn’t “see” color, classification into blue/yellow/orange becomes a detective game. Solutions include:

- Sensor Fusion: Project LiDAR detections into the camera frame and fetch the color label from the vision pipeline.
- Spatial Inference: If fusion fails, cones may be classified heuristically based on position (e.g., left vs. right side of track).

This is where LiDAR and camera must play nice — a fusion of talents like Sherlock and Watson, but with more calibration headaches.

WIP