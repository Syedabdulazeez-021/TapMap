#!/usr/bin/env python3
"""sim_firmware: stands in for the ESP32 (micro-ROS) node.

PUBLISHES : /odom            (nav_msgs/Odometry)      50 Hz  + TF odom->base_link
            /audio/tap_window(std_msgs/Float32MultiArray)    on every tap
SUBSCRIBES: /cmd_vel         (geometry_msgs/Twist)
SERVES    : /tapper/fire     (std_srvs/Trigger)

On the real robot, the ESP32 publishes wheel ticks + IMU and a base_controller
computes odometry; here the simulator integrates a unicycle model directly and
synthesises the tap acoustics: a solid surface rings ~2 kHz with fast decay,
a hollow (debonded) region rings ~500 Hz with slow decay -- matching real
percussion physics closely enough to exercise the whole pipeline.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

SAMPLE_RATE = 16000
WINDOW_S = 0.10


class SimFirmware(Node):
    def __init__(self):
        super().__init__('sim_firmware')
        # hollow region of the virtual tile panel (m, in odom frame)
        self.declare_parameter('hollow_min_x', 0.40)
        self.declare_parameter('hollow_max_x', 0.60)
        self.declare_parameter('hollow_min_y', 0.15)
        self.declare_parameter('hollow_max_y', 0.35)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.audio_pub = self.create_publisher(Float32MultiArray, 'audio/tap_window', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.create_service(Trigger, 'tapper/fire', self.fire_cb)
        self.tf = TransformBroadcaster(self)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.get_logger().info('sim_firmware up: virtual panel with one hollow region')

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def step(self):
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt
        self.th = math.atan2(math.sin(self.th + self.w * self.dt),
                             math.cos(self.th + self.w * self.dt))
        now = self.get_clock().now().to_msg()

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = math.sin(self.th / 2.0)
        od.pose.pose.orientation.w = math.cos(self.th / 2.0)
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        self.odom_pub.publish(od)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = od.pose.pose.orientation.z
        t.transform.rotation.w = od.pose.pose.orientation.w
        self.tf.sendTransform(t)

    def in_hollow(self):
        p = self.get_parameter
        return (p('hollow_min_x').value <= self.x <= p('hollow_max_x').value and
                p('hollow_min_y').value <= self.y <= p('hollow_max_y').value)

    def fire_cb(self, req, res):
        n = int(SAMPLE_RATE * WINDOW_S)
        t = np.arange(n) / SAMPLE_RATE
        if self.in_hollow():                    # drum-like: low freq, slow decay
            sig = np.sin(2 * np.pi * 500 * t) * np.exp(-t / 0.030)
        else:                                   # solid tick: high freq, fast decay
            sig = np.sin(2 * np.pi * 2000 * t) * np.exp(-t / 0.008)
        sig += 0.02 * np.random.randn(n)        # ambient noise floor
        msg = Float32MultiArray()
        msg.data = sig.astype(np.float32).tolist()
        self.audio_pub.publish(msg)
        res.success = True
        res.message = 'tap fired at (%.2f, %.2f)' % (self.x, self.y)
        return res


def main():
    rclpy.init()
    rclpy.spin(SimFirmware())


if __name__ == '__main__':
    main()
