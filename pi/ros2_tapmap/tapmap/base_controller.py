#!/usr/bin/env python3
"""base_controller: wheel ticks (+ gyro) -> /odom + TF odom->base_link.

SUBSCRIBES: /wheel_ticks  (std_msgs/Int32MultiArray [left, right], cumulative,
                           positive = forward — signs fixed in serial_bridge)
            /imu/data_raw (sensor_msgs/Imu) — gyro z, optional
PUBLISHES : /odom (nav_msgs/Odometry) + TF odom->base_link
SERVES    : /reset_odom (std_srvs/Trigger) — zero the pose at the start corner

Distance comes from the encoders, heading from the gyro whenever it is
streaming. Encoders are good at distance but poor at heading (a tiny
wheel-size mismatch compounds into a large angle error); the gyro is the
opposite. Fusing the two keeps scan rows straight. If no gyro data has
arrived in the last 0.3 s it falls back to pure wheel odometry.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        p = self.declare_parameter
        p('wheel_diameter', 0.043)   # m  (frozen chassis value)
        p('track', 0.122)            # m  (frozen chassis value)
        p('ticks_per_rev', 360.0)    # MEASURE: counts for one full wheel turn
        p('use_imu', True)

        self.x = self.y = self.th = 0.0
        self.prev = None
        self.last_t = None
        self.gz = 0.0
        self.gz_t = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Int32MultiArray, 'wheel_ticks', self.ticks_cb, 50)
        self.create_subscription(Imu, 'imu/data_raw', self.imu_cb, 50)
        self.create_service(Trigger, 'reset_odom', self.reset_cb)
        self.get_logger().info(
            f"base_controller up: D={self.P('wheel_diameter')} m, "
            f"track={self.P('track')} m, ticks/rev={self.P('ticks_per_rev')}")

    def P(self, name):
        return self.get_parameter(name).value

    def reset_cb(self, req, res):
        self.x = self.y = self.th = 0.0
        self.prev = None
        res.success = True
        res.message = 'odometry zeroed'
        return res

    def imu_cb(self, msg: Imu):
        self.gz = msg.angular_velocity.z
        self.gz_t = self.get_clock().now()

    def ticks_cb(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            return
        now = self.get_clock().now()
        l, r = int(msg.data[0]), int(msg.data[1])
        if self.prev is None:
            self.prev, self.last_t = (l, r), now
            return

        dl_t, dr_t = l - self.prev[0], r - self.prev[1]
        self.prev = (l, r)
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0 or dt > 0.5:          # stalled stream / clock jump: skip
            return

        m_per_tick = math.pi * float(self.P('wheel_diameter')) / float(self.P('ticks_per_rev'))
        dl, dr = dl_t * m_per_tick, dr_t * m_per_tick
        dc = 0.5 * (dl + dr)

        imu_fresh = (bool(self.P('use_imu')) and self.gz_t is not None and
                     (now - self.gz_t).nanoseconds < 0.3e9)
        dth = self.gz * dt if imu_fresh else (dr - dl) / float(self.P('track'))

        th_mid = self.th + 0.5 * dth        # midpoint integration
        self.x += dc * math.cos(th_mid)
        self.y += dc * math.sin(th_mid)
        self.th = wrap(self.th + dth)
        self.publish(now, dc / dt, dth / dt)

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
    node = BaseController()
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
