#!/usr/bin/env python3
"""coverage_planner: the mission brain. Hosts the /scan_area ACTION.

ACTION SERVER: /scan_area (tapmap_interfaces/ScanArea)
SUBSCRIBES   : /odom (nav_msgs/Odometry), /tap/classification (TapResult)
PUBLISHES    : /cmd_vel (geometry_msgs/Twist)
CALLS        : /tapper/fire (std_srvs/Trigger)

Sequence per goal: build boustrophedon grid -> for each waypoint:
drive -> settle -> fire tap -> await classification -> next.
Finally return to (0,0) and report the return-position error
(the built-in odometry self-audit).
"""
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tapmap_interfaces.action import ScanArea
from tapmap_interfaces.msg import TapResult


def boustrophedon(width, height, spacing):
    pts, y, left_to_right = [], 0.0, True
    while y <= height + 1e-6:
        xs = [i * spacing for i in range(int(width / spacing) + 1)]
        if not left_to_right:
            xs = xs[::-1]
        pts += [(x, y) for x in xs]
        left_to_right = not left_to_right
        y += spacing
    return pts


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        cb = ReentrantCallbackGroup()
        self.pose = (0.0, 0.0, 0.0)
        self.tap_event = threading.Event()
        self.last_tap = None
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 20, callback_group=cb)
        self.create_subscription(TapResult, 'tap/classification', self.tap_cb, 10, callback_group=cb)
        self.fire = self.create_client(Trigger, 'tapper/fire', callback_group=cb)
        self.server = ActionServer(self, ScanArea, 'scan_area', self.execute, callback_group=cb)
        self.get_logger().info('coverage_planner up: send a ScanArea goal to begin')

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        th = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, th)

    def tap_cb(self, msg):
        self.last_tap = msg
        self.tap_event.set()

    # -- simple drive-to-point controller (rotate, then translate) --
    def goto(self, gx, gy, tol=0.015):
        rate = 0.05
        while True:
            x, y, th = self.pose
            dx, dy = gx - x, gy - y
            d = math.hypot(dx, dy)
            if d < tol:
                break
            heading = math.atan2(dy, dx)
            err = math.atan2(math.sin(heading - th), math.cos(heading - th))
            cmd = Twist()
            if abs(err) > 0.08:
                cmd.angular.z = max(-1.5, min(1.5, 2.5 * err))
            else:
                cmd.linear.x = min(0.12, 1.2 * d)
                cmd.angular.z = max(-1.0, min(1.0, 1.5 * err))
            self.cmd_pub.publish(cmd)
            time.sleep(rate)
        self.cmd_pub.publish(Twist())          # full stop

    def execute(self, goal_handle):
        g = goal_handle.request
        pts = boustrophedon(g.width, g.height, g.spacing)
        fb = ScanArea.Feedback()
        defects = 0
        self.get_logger().info(f'scan started: {len(pts)} tap points')

        for i, (gx, gy) in enumerate(pts):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.cmd_pub.publish(Twist())
                return ScanArea.Result()
            self.goto(gx, gy)
            time.sleep(0.3)                     # settle before tap
            self.tap_event.clear()
            self.fire.call_async(Trigger.Request())
            if self.tap_event.wait(timeout=2.0) and self.last_tap.label == 'hollow':
                defects += 1
            fb.percent_complete = 100.0 * (i + 1) / len(pts)
            fb.taps_done = i + 1
            fb.defects_found = defects
            fb.current_x, fb.current_y = float(self.pose[0]), float(self.pose[1])
            goal_handle.publish_feedback(fb)

        result = ScanArea.Result()
        result.total_taps = len(pts)
        result.defect_cells = defects
        if g.return_home:
            self.goto(0.0, 0.0)
            result.return_error = float(math.hypot(self.pose[0], self.pose[1]))
            self.get_logger().info(f'returned home, error = {result.return_error*100:.1f} cm')
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = CoveragePlanner()
    rclpy.spin(node, executor=MultiThreadedExecutor())


if __name__ == '__main__':
    main()
