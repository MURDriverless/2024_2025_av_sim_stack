import numpy as np
import matplotlib.pyplot as plt
import sys
import os

import wheel

# TO-DO: Need to fix directory structure for modules
sys.path.append(os.path.abspath("/home/ab/git/2024_2025_av_sim_stack/simulation/track_generator"))

from track import Track
from vehicle import Vehicle
from PurePursuit import PurePursuitController

# Simulation config
DT = 0.1  # time step (s)
SIM_TIME = 30  # total sim time (s)

# Create track and controller
track = Track(num_waypoints=15)
track.generate_track()
path = track.centerline

# Init vehicle and controller
# Ensures that the vehicle yaw angle is pointing in the right direction
start = path[0]
next_pt = path[1]
dx, dy = next_pt[0] - start[0], next_pt[1] - start[1]
yaw = np.arctan2(dy, dx)
vehicle = Vehicle(wheelbase=1.7, x=start[0], y=start[1], yaw=yaw, velocity=0.0)
controller = PurePursuitController(lookahead_distance=7.0)

# Log for plotting
trajectory = []

# Sim loop
for _ in range(int(SIM_TIME / DT)):
    state = vehicle.state()
    target = controller.find_target_point(path, state)
    steer = controller.compute_control(state, target, vehicle.wheelbase)
    throttle = 1.0  # constant throttle for now

    vehicle.update(throttle, steer, DT)
    trajectory.append((vehicle.x, vehicle.y))

# Visualization
track_data = track.get_track_data()
center = np.array(track_data['centerline'])
left = track_data['left_cones']
right = track_data['right_cones']
traj = np.array(trajectory)

plt.figure(figsize=(8, 8))
plt.plot(center[:, 0], center[:, 1], 'k--', label='Centerline')
plt.plot(traj[:, 0], traj[:, 1], 'g-', linewidth=2, label='Vehicle Path')

for cone in left:
    plt.scatter(cone['x'], cone['y'], color=cone['color'], s=10)
for cone in right:
    plt.scatter(cone['x'], cone['y'], color=cone['color'], s=10)

# Start/finish line
plt.plot([left[0]['x'], right[0]['x']], [left[0]['y'], right[0]['y']], 'k-', linewidth=2, label='Start/Finish')

# Final vehicle state
last_state = vehicle.state()

# Heading arrow (from vehicle position)
arrow_length = 2.0
arrow_dx = arrow_length * np.cos(last_state['yaw'])
arrow_dy = arrow_length * np.sin(last_state['yaw'])

plt.arrow(
    last_state['x'], last_state['y'],
    arrow_dx, arrow_dy,
    head_width=0.5, head_length=0.7, fc='black', ec='black', label='Heading'
)

# Show lookahead point
final_target = controller.find_target_point(path, last_state)
plt.scatter(final_target[0], final_target[1], c='red', s=40, label='Lookahead Point', edgecolors='black')


plt.axis('equal')
plt.title("Vehicle Simulation with Pure Pursuit Controller")
plt.legend()
plt.grid(True)
plt.show()
