import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import csv
import os

class PointPublisher(Node):
    def __init__(self, left_cones=None, right_cones=None):
        super().__init__('track_visualizer')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_cones)

        self.left_cones = []
        self.right_cones = []
        self.orange_cones = []

        self.load_cones_from_csv('fs_track.csv')

    def load_cones_from_csv(self, filename):
        try:
            path = "/projects/ros2_ws/src/track_visualizer_pkg/track_visualizer_pkg/fs_track.csv"
            with open(path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader)
                for row in reader:
                    print(row)
                    x, y, obj_type, color = row
                    if color == 'blue':
                        self.left_cones.append((float(x), float(y)))
                    elif color == 'yellow':
                        self.right_cones.append((float(x), float(y)))
                    elif color == 'orange':
                        self.orange_cones.append((float(x), float(y)))


        except Exception as e:
            self.get_logger().error(f"Failed to read cone CSV: {e}")

    def create_cone_mesh_marker(self, x, y, ns, marker_id, color, mesh_path):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0  # No rotation

        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001

        marker.mesh_resource = mesh_path
        marker.mesh_use_embedded_materials = False  # STL has no color

        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        return marker


    def publish_cones(self):
        marker_id = 0
        mesh_path = 'package://track_visualizer_pkg/meshes/traffic_cone.stl'
        # mesh_path = '/projects/rviz_test/my_visualizer/my_visualizer/resources/traffic_cone.stl'

        for x, y in self.left_cones:
            marker = self.create_cone_mesh_marker(x, y, "left_cones", marker_id,
                                                  (0.0, 0.0, 1.0), mesh_path)
            self.publisher.publish(marker)
            marker_id += 1

        for x, y in self.right_cones:
            marker = self.create_cone_mesh_marker(x, y, "right_cones", marker_id,
                                                  (1.0, 1.0, 0.0), mesh_path)
            self.publisher.publish(marker)
            marker_id += 1

    # def publish_cones(self):
    #     marker = Marker()
    #     marker.header.frame_id = "map"
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = "test"
    #     marker.id = 999
    #     marker.type = Marker.CUBE
    #     marker.action = Marker.ADD
    #     marker.pose.position.x = 0.0
    #     marker.pose.position.y = 0.0
    #     marker.pose.position.z = 0.5
    #     marker.scale.y = 0.1
    #     marker.scale.z = 0.1
    #     marker.scale.x = 0.1
    #     marker.color.a = 1.0
    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0
    #     self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

















































# import rclpy
# from rclpy.node import Node
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point
# import csv
# import os

# class PointPublisher(Node):
#     def __init__(self, left_cones=None, right_cones=None):
#         super().__init__('track_visualizer')
#         self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
#         self.timer = self.create_timer(1.0, self.publish_cones)

#         self.left_cones = []
#         self.right_cones = []
#         self.orange_cones = []

#         self.load_cones_from_csv('fs_track.csv')

#     def load_cones_from_csv(self, filename):
#         try:
#             path = "/projects/ros2_ws/src/track_visualizer_pkg/track_visualizer_pkg/fs_track.csv"
#             with open(path, 'r') as csvfile:
#                 reader = csv.reader(csvfile)
#                 next(reader)
#                 for row in reader:
#                     print(row)
#                     x, y, obj_type, color = row
#                     if color == 'blue':
#                         self.left_cones.append((float(x), float(y)))
#                     elif color == 'yellow':
#                         self.right_cones.append((float(x), float(y)))
#                     elif color == 'orange':
#                         self.orange_cones.append((float(x), float(y)))


#         except Exception as e:
#             self.get_logger().error(f"Failed to read cone CSV: {e}")

#     def create_cone_marker(self, cone_points, color, ns, marker_id):
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = ns
#         marker.id = marker_id
#         marker.type = Marker.POINTS
#         marker.action = Marker.ADD
#         marker.scale.x = 0.2
#         marker.scale.y = 0.2
#         marker.color.a = 1.0
#         marker.color.r = color[0]
#         marker.color.g = color[1]
#         marker.color.b = color[2]

#         for x, y in cone_points:
#             pt = Point()
#             pt.x = x
#             pt.y = y
#             pt.z = 0.0
#             marker.points.append(pt)

#         return marker

#     def publish_cones(self):
#         left_marker = self.create_cone_marker(self.left_cones, (0.0, 0.0, 1.0), "left_cones", 0)
#         right_marker = self.create_cone_marker(self.right_cones, (1.0, 1.0, 0.0), "right_cones", 1)
#         orange_marker = self.create_cone_marker(self.orange_cones, (1.0, 0.44, 0.0), "orange_cones", 2)

#         self.publisher.publish(left_marker)
#         self.publisher.publish(right_marker)
#         self.publisher.publish(orange_marker)

# def main(args=None):
#     rclpy.init(args=args)
#     node = PointPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
