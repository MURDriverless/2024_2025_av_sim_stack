Ouster OS1-128 LiDAR Setup and Tutorial
========================================

Environment Setup & Real-Vehicle LiDAR Data Collection
-------------------------------------------------------

Welcome to the Environment Setup & Data Collection tutorial.
By the end of this tutorial, you’ll understand:

- How to build and launch both CSV- and PCAP-based ROS 2 pipelines
- How to arrange real-vehicle traffic-cone tests (Straight & Curve lanes)
- Why and when to choose CSV vs. PCAP + JSON formats

What You’ll Need
----------------

- WSL (Ubuntu 22.04) in an Oracle VM
- ROS 2 Humble
- Ouster-SDK (LiDAR drivers)
- A test vehicle with LiDAR mount

Setting Up Your Workspace
-------------------------

.. code-block:: bash

    # Ensure you are in your home directory, clone our repo and enter it
    git clone https://github.com/YourOrg/2024_2025_av_sim_stack.git
    cd 2024_2025_av_sim_stack

    # Ensure Ouster-SDK lives in src/ or{ your project folder name}/ 
    git clone https://github.com/ouster-lidar/ouster-sdk.git src/ouster-sdk
    colcon build --symlink-install
    source install/setup.bash


Launching the CSV-Based Pipeline/ or PCAP-Based Pipeline
--------------------------------------------------------

.. code-block:: bash

    colcon build --packages-select csv_to_pointcloud_node --cmake-clean-cache

.. code-block:: bash

    colcon build --packages-select lidar_integration --cmake-clean-cache

.. code-block:: bash

    source install/setup.bash
    ros2 launch csv_to_pointcloud_node full_pipeline.launch.py
    ros2 launch lidar_integration lidar_pipeline_launch.py

After this, you will see:

# insert picture here

And it will automatically launch the rviz2

Inspecting ROS 2 Topics
-----------------------

By the end of this step, you’ll know how to discover active topics and view live message data via the CLI.

What You’ll Need
^^^^^^^^^^^^^^^^^^^^^

- A running ROS 2 launch (e.g. full_pipeline.launch.py)
- Your terminal with `source install/setup.bash` active


Listing All Topics
^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    ros2 topic list

should output:

.. code-block:: bash

    /pointcloud
    /filtered_pointcloud
    /cone_markers
    /rosout
    ...

This shows every topic currently being published or subscribed.

Viewing Topic Data (“Echo”)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    ros2 topic echo /cone_markers

Example:

.. code-block:: bash

    ros2 topic echo --once /pointcloud

Filtering Echo Output
^^^^^^^^^^^^^^^^^^^^^^

First 5 messages only:

.. code-block:: bash

    ros2 topic echo /pointcloud --field "width"

.. rubric:: Try This:

.. code-block:: bash

    ros2 bag record /cone_positions

Record bag of cone positions for later analysis:

With these commands, you can inspect the flow of data through your pipelines, debug message contents, and verify that each node publishes exactly what you expect.

Collecting Straight & Curve Lane Data - Real-Vehicle Cone Lane Tests
----------------------------------------------------------------------

- Mount Design: Create or adapt a sturdy custom bracket to secure the Ouster OS-1 to your vehicle’s frame.
- Power Supply: Provide a reliable auxiliary battery pack (e.g. 12 V lead-acid or LiPo).
- Software Setup: Install and configure Ouster Studio to verify sensor connectivity and adjust scanning parameters before each run.
- Procedure: Mount LiDAR → drive through each lane → record both CSV and PCAP + JSON

Data Outputs & Formats
----------------------

After each run, you will have:

- CSV files: processed, downsampled point clouds
- PCAP + JSON: raw packet captures with full metadata

These are then fed into C++ detection nodes that cluster, fit, and extract cone centers. The full detection workflow sits under your LiDAR Perception Pipeline tutorial.

CSV vs. PCAP + JSON: Pros & Cons
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

CSV
""""

.. code-block:: diff

    + ✔ Lightweight & easy parse 
    + ✔ Fast load times 
    + ✔ Ideal for quick prototyping 
    - ✘ Limited metadata 
    - ✘ Loses packet-level timing  

PCAP + JSON
"""""""""""""

.. code-block:: diff

    + ✔ Raw packets + precise timestamps 
    + ✔ Full sensor calibration & metadata 
    + ✔ Enables exact sensor replay and advanced debugging 
    - ✘ Large files → slower load 
    - ✘ Requires complex PCAP + JSON parsing logic  

You’re now ready to build, launch, and collect LiDAR data in both CSV and PCAP + JSON formats.

Tutorial: LiDAR Data Acquisition & Preprocessing
------------------------------------------------

Welcome to the LiDAR Data Acquisition & Preprocessing tutorial.
By the end of this tutorial, you’ll understand:

- How to physically mount and power the Ouster OS-1 sensor
- How to collect both CSV and PCAP + JSON datasets on a moving vehicle
- How to preprocess raw outputs to skip invalid points and synchronize frames

What You’ll Need
^^^^^^^^^^^^^^^^

- Test Vehicle & Mounting Hardware
- Power Supply: Auxiliary 24 V battery pack (20 W peak)
- Software: Ouster Studio for health checks & parameter tuning
- ROS 2 Humble with lidar_integration (Pcap) and csv_to_pointcloud_node (CSV)

Mounting & Sensor Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Secure the Ouster OS-1 on your vehicle roof
- Verify mechanical stability at speeds up to 40 km/h
- Connect via gigabit Ethernet and auxiliary power
- Launch Ouster Studio to confirm UDP connectivity and set:

  - Channels: 128
  - Horizontal resolution: 1024
  - Frame rate: 10 Hz (or 20 Hz)

Driving Tests & Data Recording
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Arrange traffic cones:

  - Spacing: 5 m between cones
  - Lane width: 11 m

- Perform two runs:

  - Straight lane
  - Curve lane

- While driving, launch both pipelines to record:

  - CSV output (pre-processed point clouds)
  - PCAP + JSON (raw UDP packets + metadata)

Preprocessing CSV Data
^^^^^^^^^^^^^^^^^^^^^^

LiDAR records a distance of 0 for invalid returns, resulting in phantom points at the sensor’s origin.

Transform spherical to Cartesian:

.. code-block:: none

    x = ρ · sin φ · cos θ
    y = ρ · sin φ · sin θ
    z = ρ · cos φ

Where:

- ρ (range): radial distance
- θ (azimuth): angle in the xy plane
- φ (elevation): angle from +z toward xy plane

Python Snippet:

.. code-block:: python

    import pandas as pd

    df = pd.read_csv('data/straight or curve.csv')
    df = df[df['range'] > 0]
    df.to_csv('points_filtered.csv', index=False)

Benefits of removing zero-range points:

- Eliminates Origin Spikes
- Improves Clustering & Detection
- Speeds Up Downstream Processing

Preprocessing PCAP + JSON Data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

No explicit “no-return” flag in PCAP packets. To filter:

1. **Skip Initial Frames**

   Use `skip_time` parameter:

   .. code-block:: python

       from launch import LaunchDescription
       from launch.actions import DeclareLaunchArgument

       json_path = LaunchConfiguration('json_path')
       pcap_path = LaunchConfiguration('pcap_path')
       skip_time = LaunchConfiguration('skip_time')

   In `lidar_pipeline_launch.py` and `total_reader.cpp`:

   .. code-block:: cpp

       double skip_time = this->declare_parameter("skip_time");
       if (current_time < skip_time) continue;

2. **Filter by UDP Port**

   .. code-block:: cpp

       if (pkt_info.dst_port == lidar_port) {
           LidarPacket pkt(static_cast<int>(pkt_info.packet_size));
           std::memcpy(pkt.buf.data(), data, pkt_info.packet_size);
       }

Benefits:

- Lower Parsing Overhead
- Improved Detection Reliability

Tutorial: Lidar Perception Pipeline & Preprocessing 2: CSV Reader
-----------------------------------------------------------------

Welcome to the detailed File Reference and pipeline overview for our CSV-based perception nodes.
By the end of this tutorial, you will have:

- A clear visual of the CSV→PointCloud2 processing pipeline
- Concise summaries of core functions in the CSV reader node

Pipeline Stages
^^^^^^^^^^^^^^^

1. **CsvToPointCloudNode (lidar.cpp)**  
   Publishes raw clouds and sphere markers on `/pointcloud` (optional `/raw_point_markers`) after reading X/Y/Z, signal, and reflectivity from CSV.

2. **CropBoxFilterNode (crop_box.cpp)**  
   Subscribes to `/pointcloud`, applies a 3D CropBox, republishes on `/cropped_pointcloud`, adds markers on `/crop_box_markers`.

3. **ConeDetection (cone_detection.cpp)**  
   Listens on `/cropped_pointcloud`, downsamples and clusters, then publishes on `/filtered_pointcloud` and `/cone_marker_array`. Logs positions to CSV.

4. **PointCloudToMarkers (pointcloud_to_markers.cpp)**  
   Converts reflectivity into colored spheres. Subscribes to `/filtered_pointcloud` or `/pointcloud`, outputs `/visualization_marker_array`.

Lidar.cpp Node
^^^^^^^^^^^^^^

Wall Timer at 10 Hz:

.. code-block:: cpp

    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&CsvToPointCloudNode::publish_frame, this)
    );

This ensures a 10 Hz loop, matching Ouster OS-1 spin rate.

frame_id for 360° Visualization:

.. code-block:: cpp

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "lidar_frame";
    msg.header.stamp    = this->get_clock()->now();

Ensures all points are assigned to the correct coordinate frame in RViz2.

Batch Size = 1024 × 128:

.. code-block:: cpp

    size_t batch_size = 1024 * 128;
    size_t end_index  = std::min(current_index_ + batch_size, points_.size());
    auto   frame      = vector<PointXYZIR>(
                           points_.begin() + current_index_,
                           points_.begin() + end_index
                        );
    current_index_    = end_index;

This reflects Ouster OS-1 resolution: 1024 azimuth steps × 128 channels = 131,072 points.

