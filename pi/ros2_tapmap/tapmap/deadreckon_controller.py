#!/usr/bin/env python3
"""deadreckon_controller: odometry for an ENCODER-LESS 4-wheel chassis.

Drop-in replacement for base_controller when the drive motors have NO
encoders (the yellow TT-motor 4-wheel chassis). Instead of measuring how
far the wheels turned, it integrates the COMMANDED velocity over time
(open-loop dead reckoning):

    distance  <- commanded linear.x  * speed_scale * dt
    heading   <- real gyro if it is streaming, else commanded angular.z
                 * turn_scale * dt

SUBSCRIBES: /cmd_vel      (geometry_msgs/Twist)  -- what the planner asked for
            /imu/data_raw (sensor_msgs/Imu)      -- gyro z, OPTIONAL
PUBLISHES : /odom (nav_msgs/Odometry) + TF odom->base_link
SERVES    : /reset_odom (std_srvs/Trigger) -- zero the pose at the start corner

Because the planner closes its own loop on this /odom, the scan pattern
always completes. To make the PHYSICAL robot match the map you calibrate
two numbers with a stopwatch/ruler (see config/tapmap.yaml):
  speed_scale : real metres travelled / metres the odom thinks it moved
  turn_scale  : real degrees turned  / degrees the odom thinks it turned
Both default to 1.0; tune them once on the floor.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DeadReckonController(Node):
    def __init__(self):
        super().__init__('deadreckon_controller')
        p = self.declare_parameter
        p('speed_scale', 1.0)      # real metres / commanded metres  (stopwatch cal)
        p('turn_scale', 1.0)       # real radians / commanded radians (protractor cal)
        p('use_imu', True)         # fuse real gyro heading if it is streaming
        p('cmd_timeout', 0.5)      # s: no cmd_vel for this long -> treat as stopped
        p('rate_hz', 50.0)         # integration rate

        self.speed_scale = float(self.P('speed_scale'))
        self.turn_scale = float(self.P('turn_scale'))
        self.use_imu = bool(self.P('use_imu'))
        self.cmd_timeout = float(self.P('cmd_timeout'))

        self.x = self.y = self.th = 0.0
        self.cmd_v = self.cmd_w = 0.0
        self.cmd_t = None
        self.gz = 0.0
        self.gz_t = None
        self.last_t = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 20)
        self.create_subscription(Imu, 'imu/data_raw', self.imu_cb, 50)
        self.create_service(Trigger, 'reset_odom', self.reset_cb)
        self.timer = self.create_timer(1.0 / float(self.P('rate_hz')), self.step)
        self.get_logger().info(
            f'deadreckon_controller up (encoder-less mode): '
            f'speed_scale={self.speed_scale}, turn_scale={self.turn_scale}, '
            f'use_imu={self.use_imu}')

    def P(self, name):
        return self.get_parameter(name).value

    def reset_cb(self, req, res):
        self.x = self.y = self.th = 0.0
        res.success = True
        res.message = 'odometry zeroed'
        return res

    def cmd_cb(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.cmd_t = self.get_clock().now()

    def imu_cb(self, msg: Imu):
        self.gz = msg.angular_velocity.z
        self.gz_t = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0 or dt > 0.5:              # clock jump / long pause: skip
            return

        # command is only valid if it arrived recently (deadman)
        fresh_cmd = (self.cmd_t is not None and
                     (now - self.cmd_t).nanoseconds < self.cmd_timeout * 1e9)
        v = self.cmd_v * self.speed_scale if fresh_cmd else 0.0
        w_cmd = self.cmd_w * self.turn_scale if fresh_cmd else 0.0

        # heading: prefer the real gyro when it is streaming, else the command
        imu_fresh = (self.use_imu and self.gz_t is not None and
                     (now - self.gz_t).nanoseconds < 0.3e9)
        w = self.gz if imu_fresh else w_cmd

        th_mid = self.th + 0.5 * w * dt         # midpoint integration
        self.x += v * math.cos(th_mid) * dt
        self.y += v * math.sin(th_mid) * dt
        self.th = wrap(self.th + w * dt)
        self.publish(now, v, w)

    def publish(self, now, v, w):
        stamp = now.to_msg()
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = w
        self.odom_pub.publish(od)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = DeadReckonController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
