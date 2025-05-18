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
        self.car_pub = self.create_publisher(PoseStamped, '/car_pose', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.publish_all)  # 10 Hz

    def start(self):
        rclpy.spin(self)
        self.destroy_node()
        rclpy.shutdown()

    def publish_all(self):
        if self.gpu.lidar_data:
            self.publish_cones()
        if self.gpu.car.state:
            self.publish_car_pose()
        if hasattr(self.gpu, 'particles') and self.gpu.particles:
            self.publish_slam_landmarks()

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
            marker.lifetime.sec = 0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def publish_car_pose(self):
        state = self.gpu.car.state
        timestamp = self.get_clock().now().to_msg()

        sin_half_yaw = math.sin(state.yaw / 2.0)
        cos_half_yaw = math.cos(state.yaw / 2.0)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = timestamp
        pose_msg.pose.position.x = state.x
        pose_msg.pose.position.y = state.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.z = sin_half_yaw
        pose_msg.pose.orientation.w = cos_half_yaw

        self.car_pub.publish(pose_msg)

        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = "car_pose"
        tf_msg.transform.translation.x = state.x
        tf_msg.transform.translation.y = state.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = pose_msg.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

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






