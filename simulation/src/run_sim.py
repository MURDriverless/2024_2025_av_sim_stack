import os
import csv
import threading
import subprocess
import numpy as np
import rclpy
from scipy.spatial import QhullError

from world import World
from track import Track
from state import State
from vehicle import Car
from control import PurePursuit
from lidar import Lidar
from camera import Stereo
from visualiser import Rviz


def load_cones(csv_path):
    """Load cone positions and colors from CSV, skipping black cones."""
    cones = []
    with open(csv_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # skip header
        for row in reader:
            if row:
                x, y, _, color = row
                if color != 'black':
                    cones.append([[float(x), float(y)], color])
    return cones


def setup_car(world, track):
    """Configure the car with initial state, sensors, and controller."""
    initial_state = State([0, 0, np.deg2rad(45), 0.001, 0, 0, 0])
    car = Car(world, track, "Test Car", initial_state)
    car.add_lidar(Lidar())
    car.add_camera(Stereo())
    car.add_controller(PurePursuit(
        lookahead_distance=4, u_max=40, k_speed_c=150, k_throttle=6000
    ))
    return car


def launch_rviz(config_path="~/Documents/src/my_simulation_config.rviz"):
    """Launch RViz2 with a predefined config file."""
    return subprocess.Popen(["rviz2", "-d", os.path.expanduser(config_path)])


def main():
    # Load track and create simulation world
    cones = load_cones('fs_track.csv')
    world = World("Earth")
    track = Track(world, "Test Track", cones)
    car = setup_car(world, track)

    # Initialize ROS and visualization
    rclpy.init()
    visualizer = Rviz(car.gpu)
    rviz_thread = threading.Thread(target=visualizer.start)
    rviz_thread.start()

    rviz_proc = launch_rviz()

    try:
        car.start_controller()
    except QhullError as e:
        print(f"[QhullError] {e}")
        car.gpu.plotting()
    else:
        print("SIMULATION COMPLETE")
    finally:
        rclpy.shutdown()
        rviz_thread.join()
        rviz_proc.terminate()


if __name__ == "__main__":
    main()


