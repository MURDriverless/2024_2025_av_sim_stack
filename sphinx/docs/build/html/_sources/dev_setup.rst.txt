Development Environment Setup
=============================

This guide walks you through setting up a development environment that includes Python libraries commonly used for algorithm development, as well as the full desktop version of ROS2 (including RViz2), tailored for FSAE autonomous vehicle development.

.. contents::
   :local:
   :depth: 2

------------
Requirements
------------

- At least Ubuntu 22.04 (recommended for compatibility with ROS2 Jazzy) OR Windows Subsystem for Linux (WSL2) with the Ubuntu distribution
- Internet connection
- Basic familiarity with the terminal

-----------------------------
Step 1: System Preparation
-----------------------------

1. **Update system packages**

   .. code-block:: bash

      sudo apt update && sudo apt upgrade -y

2. **Install essential tools**

   .. code-block:: bash

      sudo apt install -y build-essential curl wget git python3-pip python3-venv

---------------------------------------
Step 2: Setup Python Development Env
---------------------------------------

1. **Create a Python virtual environment**

   .. code-block:: bash

      python3 -m venv fsae_env
      source fsae_env/bin/activate

2. **Upgrade pip and install packages**

   .. code-block:: bash

      pip install --upgrade pip
      pip install numpy matplotlib cvxpy

3. **Verify installation**

   .. code-block:: bash

      python -c "import numpy, matplotlib, cvxpy; print('All packages installed successfully')"

-----------------------------------
Step 3: Install ROS2 Jazzy (LTS)
-----------------------------------

At the time of writing (May 2025), ROS2 Jazzy is a supported LTS version.

1. **Set up sources**

   .. code-block:: bash

      sudo apt install software-properties-common
      sudo add-apt-repository universe
      sudo apt update && sudo apt install curl -y
      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc > /dev/null
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/trusted.gpg.d/ros.asc] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

2. **Install the full desktop version of ROS2**

   .. code-block:: bash

      sudo apt update
      sudo apt install ros-jazzy-desktop -y

3. **Environment setup**

   .. code-block:: bash

      echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
      source ~/.bashrc

4. **Install development tools and dependencies**

   .. code-block:: bash

      sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete

5. **Initialize `rosdep`**

   .. code-block:: bash

      sudo rosdep init
      rosdep update

---------------------------
Step 4: Test the Setup
---------------------------

1. **Open a new terminal**

2. **Source the ROS2 environment**

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash

3. **Launch a test visualization**

   .. code-block:: bash

      rviz2

   You should see the RViz2 GUI launch successfully.

-------------------------
Optional: Docker Setup
-------------------------

Prefer a containerized setup? Use Docker:

.. code-block:: bash

   docker run -it --rm \
     --net=host \
     --privileged \
     --env="DISPLAY=$DISPLAY" \
     --env="QT_X11_NO_MITSHM=1" \
     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
     osrf/ros:jazzy-desktop

Make sure to run ``xhost +local:docker`` on your host machine to allow GUI forwarding.

--------------------------
You're Good to Go!
--------------------------

You now have a fully functional FSAE-focused dev environment with Python scientific libraries and ROS2. Start coding, simulating, and visualizing!
