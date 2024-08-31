PI = 3.14
DT = 0.1
VELOCITY = 1.0
STEERING_ANGLE = 30 * (PI / 180)
WHEELBASE = 2.0

# Function to update the position and orientation 
def update_position(x, y, theta, velocity, steering_angle, dt)
    # Bicycle model equations
    x += velocity * cos(theta) * dt
    y += velocity * sin(theta) * dt
    theta += (velocity / WHEELBASE) * tan(steering_angle) * dt

if __name__ == "__main__":
    # Initial position and orientation of the circle
    x = 0.0
    y = 0.0
    theta = 0.0

    # Move forward for 10 meters
    distance_traveled = 0.0
    while (distance_traveled < 10):
        update_position(x, y, theta, VELOCITY, 0.0, DT)
        distance_traveled += VELOCITY * DT
        print(f"")
