# Test vehicle physics

import sys
import os

# Append track module path
sys.path.append(os.path.abspath("/home/ab/git/2024_2025_av_sim_stack/simulation/track_generator"))

from track import Track
from vehicle import Vehicle
import matplotlib.pyplot as plt

# Init track and vehicle
track = Track()
track.generate_track()
path = track.centerline
car = Vehicle(wheelbase=1.7)
car_state = []
dt = 0.1

# Sim loop
for i in range(1000):
    car.update(throttle=0.1, steering_angle=0.1, dt=dt)
    car_state.append(car.state().copy())

# Plot vehicle trajectory
car_x = []
car_y = []
for i in car_state:
    if "x" in i:
        car_x.append(i["x"])
    if "y" in i:
        car_y.append(i["y"])

plt.figure(figsize=(8, 8))
plt.plot([i["x"] for i in car_state if "x" in i], [i["y"] for i in car_state if "y" in i], color='orange', label='Car Path')
plt.plot(path[:,0], path[:,1], 'k--', label="Track Centerline")
plt.legend()
plt.grid(True)
plt.show()