# rviz_cone_test
Test importing a traffic cone STL model on Rviz2

Ensure Rviz2 and ROS2 Jazzy is installed with colcon build dependencies

To run:
```
# Step 1, assuming you're in your home directory, ~/
git clone repo

# Step 2
cd rviz_cone_test/ros2_ws                                               # This is your root ROS2 workspace

# Step 3, assuming your in the root ROS2 workspace
colcon build && source install/setup.bash                               # This creates build/ install/ and log/ directories

# Step 4
ros2 run track_visualizer_pkg track_visualizer

# Step 5, run rviz2
rviz2

# Step 6
# In rviz, click add, click by topic, double click 'Marker' under /visualization_marker
```
Remember to edit the path for fs_track.csv (line 22) in track_visualizer.py to its absolute path