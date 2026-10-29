#!/usr/bin/env python3
"""defect_mapper: subscriber -> publisher.

SUBSCRIBES: /tap/classification (tapmap_interfaces/TapResult)
PUBLISHES : /defect_grid   (nav_msgs/OccupancyGrid)  -- the live heat map
            /defect_markers(visualization_msgs/MarkerArray) -- red/green dots

View both in RViz2 (fixed frame: odom).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tapmap_interfaces.msg import TapResult

RES = 0.05          # m / cell
W, H = 40, 40       # 2 m x 2 m grid


class DefectMapper(Node):
    def __init__(self):
        super().__init__('defect_mapper')
        self.grid = [-1] * (W * H)
        self.markers = MarkerArray()
        self.n = 0
        self.create_subscription(TapResult, 'tap/classification', self.tap_cb, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'defect_grid', 1)
        self.mark_pub = self.create_publisher(MarkerArray, 'defect_markers', 1)
        self.create_timer(1.0, self.publish_grid)
        self.get_logger().info('defect_mapper up: 2x2 m grid @ 5 cm')

    def tap_cb(self, msg: TapResult):
        cx, cy = int(msg.x / RES), int(msg.y / RES)
        if 0 <= cx < W and 0 <= cy < H:
            self.grid[cy * W + cx] = int(100 * msg.confidence) if msg.label == 'hollow' else 0
        m = Marker()
        m.header.frame_id = 'odom'
        m.ns, m.id, m.type, m.action = 'taps', self.n, Marker.CYLINDER, Marker.ADD
        self.n += 1
        m.pose.position.x, m.pose.position.y, m.pose.position.z = msg.x, msg.y, 0.005
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.04
        m.scale.z = 0.01
        if msg.label == 'hollow':
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.15, 0.1, 0.95
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.3, 0.6
        self.markers.markers.append(m)
        self.mark_pub.publish(self.markers)

    def publish_grid(self):
        g = OccupancyGrid()
        g.header.frame_id = 'odom'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = RES
        g.info.width, g.info.height = W, H
        g.info.origin.orientation.w = 1.0
        g.data = self.grid
        self.grid_pub.publish(g)


def main():
    rclpy.init()
    rclpy.spin(DefectMapper())


if __name__ == '__main__':
    main()
