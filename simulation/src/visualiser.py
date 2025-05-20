import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster


class Rviz(Node):
    def __init__(self, gpu):
        super().__init__('visualizer_node')
        self.gpu = gpu

        self.marker_pub = self.create_publisher(MarkerArray, '/cone_markers', 20)
        self.slam_marker_pub = self.create_publisher(MarkerArray, '/slam_landmarks', 20)
        self.car_marker_pub = self.create_publisher(Marker, '/car_model_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cone_timer = self.create_timer(0.1, self.publish_all)  # 10 Hz
        self.car_timer = self.create_timer(0.02, self.publish_car)  # 50 Hz

    def start(self):
        rclpy.spin(self)
        self.destroy_node()
        rclpy.shutdown()

    def publish_all(self):
        if self.gpu.lidar_data: 
            self.publish_cones()
        if hasattr(self.gpu, 'particles') and self.gpu.particles:
            self.publish_slam_landmarks()
        if self.gpu.car.state:
            self.publish_speed_text()

    def publish_car(self):
        if self.gpu.car.state:
            self.publish_car_pose()
            self.publish_target_point()

        

    def publish_cones(self):
        marker_array = MarkerArray()
        state = self.gpu.car.state
        timestamp = self.get_clock().now().to_msg()

        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)

        for i, cone in enumerate(self.gpu.lidar_data):
            local_x, local_y = cone.x, cone.y
            global_x = cos_yaw * local_x - sin_yaw * local_y + state.x
            global_y = sin_yaw * local_x + cos_yaw * local_y + state.y

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = timestamp
            marker.ns = "cones"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = global_x
            marker.pose.position.y = global_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color = self.get_cone_color(cone.color)
            marker.lifetime.sec = 1

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def publish_target_point(self):
        if not hasattr(self.gpu.controller, 'target') or self.gpu.controller.target is None:
            return

        state = self.gpu.car.state
        target = self.gpu.controller.target  # [x_local, y_local]

        # Transform to world frame
        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)
        x = cos_yaw * target[0] - sin_yaw * target[1] + state.x
        y = sin_yaw * target[0] + cos_yaw * target[1] + state.y

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "controller_target"
        marker.id = 99
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # red

        # X shape points
        size = 0.3
        marker.points = [
            Point(x=x - size, y=y - size, z=0.1),
            Point(x=x + size, y=y + size, z=0.1),
            Point(x=x - size, y=y + size, z=0.1),
            Point(x=x + size, y=y - size, z=0.1),
        ]

        self.car_marker_pub.publish(marker)


    def publish_car_pose(self):
        state = self.gpu.car.state
        timestamp = self.get_clock().now().to_msg()

        # Add 180° yaw to car yaw (rotate mesh to face "backward")
        corrected_yaw = state.yaw + math.pi

        # Quaternion for dynamic yaw
        sin_yaw = math.sin(corrected_yaw / 2.0)
        cos_yaw = math.cos(corrected_yaw / 2.0)
        yaw_q = [0.0, 0.0, sin_yaw, cos_yaw]  # [x, y, z, w]

        # Quaternion for 90° roll (about X axis)
        sin_roll = math.sin(math.pi / 4)
        cos_roll = math.cos(math.pi / 4)
        roll_q = [sin_roll, 0.0, 0.0, cos_roll]  # [x, y, z, w]

        # Multiply yaw_q * roll_q
        x1, y1, z1, w1 = yaw_q
        x2, y2, z2, w2 = roll_q
        qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

        # TF for sensors (just dynamic yaw, no roll or 180 flip)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "car_pose"
        tf_msg.transform.translation.x = state.x
        tf_msg.transform.translation.y = state.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = math.sin(state.yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(state.yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf_msg)

        # STL mesh marker (yaw + 180° + roll)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = timestamp
        marker.ns = "car"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "file:///home/mur/Documents/src/car_model.stl"
        marker.pose.position.x = state.x
        marker.pose.position.y = state.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001

        marker.color.r = 0.6
        marker.color.g = 0.6
        marker.color.b = 0.6
        marker.color.a = 1.0

        self.car_marker_pub.publish(marker)

    def publish_speed_text(self):
        state = self.gpu.car.state
        timestamp = self.get_clock().now().to_msg()

        marker = Marker()
        marker.header.frame_id = "car_pose"
        marker.header.stamp = timestamp
        marker.ns = "speed_text"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.5  # hover above car
        marker.scale.z = 0.4  # text height
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = f"Speed: {state.u*3.6:.2f} km/h"

        self.car_marker_pub.publish(marker)




    def publish_slam_landmarks(self):
        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()
        landmarks = self.compute_average_landmarks()

        for i, (x, y) in enumerate(landmarks):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = timestamp
            marker.ns = "slam_landmarks"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
            marker.lifetime.sec = 0

            marker_array.markers.append(marker)

        self.slam_marker_pub.publish(marker_array)

    def compute_average_landmarks(self):
        particles = self.gpu.particles
        if not particles:
            return []

        num_landmarks = max(len(p.lm) for p in particles)
        avg_landmarks = []

        for lm_id in range(num_landmarks):
            sum_x = sum_y = total_w = 0.0
            for p in particles:
                if lm_id < len(p.lm):
                    x, y = p.lm[lm_id]
                    w = p.w
                    sum_x += w * x
                    sum_y += w * y
                    total_w += w
            if total_w > 0:
                avg_landmarks.append((sum_x / total_w, sum_y / total_w))
            else:
                avg_landmarks.append((0.0, 0.0))

        return avg_landmarks

    def get_cone_color(self, color_name):
        return {
            "blue": ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
            "yellow": ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
            "orange": ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
        }.get(color_name, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))  # default white






