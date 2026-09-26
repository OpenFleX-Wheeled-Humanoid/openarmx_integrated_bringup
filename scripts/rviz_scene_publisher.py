#!/usr/bin/env python3
"""
RViz Scene Publisher for OpenArmX Integrated Robot Testing

Publishes MarkerArray to /scene_markers for visualizing a test environment:
- Ground plane (8x8m)
- 4 walls (opaque, 2.5m high)
- 2 box obstacles + 2 cylinder obstacles
- 1 low railing

Usage:
    ros2 run openarmx_integrated_bringup rviz_scene_publisher
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


class RVizScenePublisher(Node):
    def __init__(self):
        super().__init__('rviz_scene_publisher')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('enable_walls', True)
        self.declare_parameter('enable_obstacles', True)

        self.frame_id = self.get_parameter('frame_id').value

        self.publisher = self.create_publisher(MarkerArray, '/scene_markers', 1)

        # Build the message once (scene is static)
        self.scene_msg = self._build_scene()

        # 1Hz publish
        self.timer = self.create_timer(1.0, self.publish_scene)
        self.marker_id = 0

        self.get_logger().info(
            f'Scene publisher started (frame={self.frame_id}, '
            f'{len(self.scene_msg.markers)} markers, 1Hz)')

    def _make_marker(self, marker_type, x, y, z, sx, sy, sz, r, g, b, a=1.0):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = 'scene'
        m.id = self.marker_id
        self.marker_id += 1
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = sx
        m.scale.y = sy
        m.scale.z = sz
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = a
        m.lifetime = Duration(sec=0, nanosec=0)
        return m

    def _build_ground(self):
        return [self._make_marker(
            Marker.CUBE, 0.0, 0.0, -0.005,
            8.0, 8.0, 0.01,
            0.85, 0.85, 0.85, 1.0)]

    def _build_walls(self):
        markers = []
        wall_h = 2.5
        wall_t = 0.08
        half_h = wall_h / 2.0
        room = 8.0
        half = room / 2.0
        c = (0.6, 0.65, 0.7, 1.0)

        markers.append(self._make_marker(
            Marker.CUBE, 0.0, half, half_h,
            room + wall_t, wall_t, wall_h, *c))
        markers.append(self._make_marker(
            Marker.CUBE, 0.0, -half, half_h,
            room + wall_t, wall_t, wall_h, *c))
        markers.append(self._make_marker(
            Marker.CUBE, half, 0.0, half_h,
            wall_t, room + wall_t, wall_h, *c))
        markers.append(self._make_marker(
            Marker.CUBE, -half, 0.0, half_h,
            wall_t, room + wall_t, wall_h, *c))
        return markers

    def _build_obstacles(self):
        markers = []
        obstacle_color = (0.55, 0.45, 0.35, 0.9)

        markers.append(self._make_marker(
            Marker.CUBE, 2.5, 2.5, 0.3,
            0.6, 0.5, 0.6, *obstacle_color))
        markers.append(self._make_marker(
            Marker.CUBE, 2.5, -2.5, 0.35,
            0.7, 0.5, 0.7, *obstacle_color))

        markers.append(self._make_marker(
            Marker.CYLINDER, -2.5, 2.5, 0.5,
            0.3, 0.3, 1.0,
            0.5, 0.5, 0.5, 0.9))
        markers.append(self._make_marker(
            Marker.CYLINDER, -2.5, -2.5, 0.5,
            0.4, 0.4, 1.0,
            0.5, 0.5, 0.5, 0.9))

        markers.append(self._make_marker(
            Marker.CUBE, 1.5, 1.5, 0.1,
            2.0, 0.05, 0.2,
            0.7, 0.7, 0.7, 0.9))

        return markers

    def _build_scene(self):
        self.marker_id = 0
        msg = MarkerArray()
        msg.markers.extend(self._build_ground())
        if self.get_parameter('enable_walls').value:
            msg.markers.extend(self._build_walls())
        if self.get_parameter('enable_obstacles').value:
            msg.markers.extend(self._build_obstacles())
        return msg

    def publish_scene(self):
        self.publisher.publish(self.scene_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RVizScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
