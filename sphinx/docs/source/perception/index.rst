Perception
==========

What is perception in autonomous vehicles, or in general autonomous systems? It refers to how they "see" and interpret the world around them. These systems might perceive their environment in ways similar to—or very different from—how humans do. For instance, a camera might serve as a rough equivalent to human eyes, but cameras come in many forms, such as thermal and infrared, which capture light beyond the range of human vision. Additionally, these perception systems are not restricted to cameras. In our case, we also intend to implement the use of **Light Detection and Ranging (LiDAR)** which uses lasers to create a point cloud of the environment it scans.

The overall perception pipeline for MUR Autonomous will be divided into two pipelines:

1. The image-based pipeline (camera-based).
2. The LiDAR-based pipeline.

The reasoning for these two pipelines is due to redundancy and safety. Sure you can run one of the pipelines, however, should one fail, you autonomous vehicle is now running blind. Ofcourse the emergency braking system should kick in, but having redundancy is essential in the robustness and safety of the system.