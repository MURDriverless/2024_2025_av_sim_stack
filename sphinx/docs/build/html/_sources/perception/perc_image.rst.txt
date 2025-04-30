##################################################################
Camera-Based Perception Pipeline: A Visually-Guided Cone-Whisperer
##################################################################

In the high-speed world of FSAE Driverless, where autonomous race cars must navigate circuits defined by nothing more than colored traffic cones, a camera-based perception pipeline plays the role of both artist and analyst — painting the scene in pixels and extracting structure from chaos. While LiDAR may have the spatial flair, it’s the humble camera that brings the color and context.

.. rubric:: Image Acquisition: A Frame-Worthy Start

The process begins with RGB cameras (usually global shutter to avoid motion artifacts), capturing high-frequency images of the scene ahead. These cameras are mounted to provide a wide, strategically-angled field of view to maximize cone visibility — because missing a cone in FSAE is like missing a gate in slalom skiing: embarrassing and point-costly.

.. rubric:: Preprocessing

Raw images undergo a series of essential grooming steps:
- Undistortion: Correcting for fisheye or barrel distortion using known camera intrinsics.
- Normalization: Adjusting illumination and contrast for visual consistency under ever-treacherous lighting conditions.
- Region of Interest (ROI): Cropping out sky and other irrelevancies — cones don’t fly, so neither should your pixels.

.. rubric:: Detection & Classification: Where Vision Meets Intelligence

At this stage, the pipeline identifies cones and tells them apart — not just as blobs, but as team players (blue = left, yellow = right, orange = start/finish). Two primary approaches coexist (sometimes contentiously):
- Traditional CV Methods: Color segmentation in HSV space combined with contour detection and morphological operations. Effective, if slightly old-school — think of it as the vinyl of perception.
- Deep Learning Methods: Convolutional Neural Networks (e.g., YOLO, SSD, or Faster R-CNN) trained on labeled FSAE datasets deliver robust bounding boxes and class probabilities. These models thrive on GPU-powered hardware and caffeine-fueled training marathons.

Either way, the output is a set of 2D bounding boxes, complete with cone classification, timestamped and ready for dimensional resurrection.

.. rubric:: From Flatland to Trackland

Now comes the geometric magic — lifting detections from the 2D image plane into the 3D world:
- Stereo Vision: Disparity maps from synchronized stereo cameras yield triangulated depth estimates.
- Monocular Depth Estimation: For the brave or the hardware-constrained, monocular depth prediction using deep nets or size-prior heuristics can provide usable (if shakier) results.
- Sensor Fusion with LiDAR: In many FSAE setups, the camera is not alone. LiDAR can corroborate or refine detections — an inter-sensor handshake that’s particularly helpful in uncertain terrain.

.. rubric:: Cone Tracking and Map Construction: Herding the Pixels

Cone detections across frames are fused and filtered using algorithms such as Kalman filters or Hungarian matching. This ensures consistency, avoids duplication, and builds a short-term memory of the environment — essentially, a cone map annotated with color and position. This is handed off to the SLAM pipeline.

WIP