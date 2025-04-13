# Physics based on bicycle model

import numpy as np

class Vehicle:
    def __init__(self, wheelbase=2.5, x=0.0, y=0.0, yaw=0.0, velocity=0.0):
        self.wheelbase = wheelbase  # meters
        self.x = x                  # position x (meters)
        self.y = y                  # position y (meters)
        self.yaw = yaw              # orientation (radians)
        self.velocity = velocity    # current speed (m/s)

    def update(self, throttle, steering_angle, dt):
        # Clamp steering angle to physical limits if desired
        max_steering = np.radians(30)  # 30 deg max steer
        steering_angle = np.clip(steering_angle, -max_steering, max_steering)

        # Kinematic bicycle model
        self.x += self.velocity * np.cos(self.yaw) * dt
        self.y += self.velocity * np.sin(self.yaw) * dt
        self.yaw += (self.velocity / self.wheelbase) * np.tan(steering_angle) * dt
        self.yaw = self._normalize_angle(self.yaw)

        self.velocity += throttle * dt  # Basic acceleration model

    def _normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def state(self):
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "velocity": self.velocity
        }
