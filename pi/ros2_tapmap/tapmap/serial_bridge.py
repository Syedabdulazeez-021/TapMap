#!/usr/bin/env python3
"""serial_bridge: the only node that talks to the ESP32 (USB serial).

Replaces sim_firmware on the real robot. Keeps the same ROS 2 contract,
so coverage_planner / audio_classifier / defect_mapper are unchanged.

READS from ESP32   "ENC,<l>,<r>,<gz>"  -> /wheel_ticks  (std_msgs/Int32MultiArray)
                                        -> /imu/data_raw (sensor_msgs/Imu, gyro z)
                   "AUDIO,<s0>,<s1>.." -> /audio/tap_window (std_msgs/Float32MultiArray)
SUBSCRIBES         /cmd_vel (geometry_msgs/Twist) -> sends "M,<l_pwm>,<r_pwm>"
SERVES             /tapper/fire (std_srvs/Trigger) -> sends "TAP", waits for AUDIO

Calibration parameters (set in config/tapmap.yaml):
  left_motor_dir / right_motor_dir : +1 or -1 so that +cmd drives FORWARD
  left_enc_dir   / right_enc_dir   : +1 or -1 so that forward motion counts UP
  min_pwm  : smallest PWM that actually turns the wheels (deadband)
"""
import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        p = self.declare_parameter
        p('port', '/dev/ttyUSB0')
        p('baud', 115200)
        p('track', 0.122)            # m, wheel separation
        p('v_at_full_pwm', 0.30)     # m/s that PWM 255 roughly produces
        p('min_pwm', 60)             # deadband: below this the motor stalls
        p('max_pwm', 255)            # speed cap: lower this to make the robot slower
        p('left_motor_dir', 1)
        p('right_motor_dir', 1)
        p('left_enc_dir', 1)
        p('right_enc_dir', 1)
        p('tap_timeout_s', 4.0)

        g = lambda n: self.get_parameter(n).value
        self.track = float(g('track'))
        self.vfull = float(g('v_at_full_pwm'))
        self.min_pwm = int(g('min_pwm'))
        self.max_pwm = int(g('max_pwm'))
        self.lmd, self.rmd = int(g('left_motor_dir')), int(g('right_motor_dir'))
        self.led, self.red = int(g('left_enc_dir')), int(g('right_enc_dir'))
        self.tap_timeout = float(g('tap_timeout_s'))

        port, baud = g('port'), int(g('baud'))
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.lock = threading.Lock()

        self.ticks_pub = self.create_publisher(Int32MultiArray, 'wheel_ticks', 20)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 20)
        self.audio_pub = self.create_publisher(Float32MultiArray, 'audio/tap_window', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.create_service(Trigger, 'tapper/fire', self.fire_cb)

        self._audio_evt = threading.Event()
        self._last_audio = []
        self.imu_seen = False

        threading.Thread(target=self._read_loop, daemon=True).start()
        self.get_logger().info(f'serial_bridge up on {port} @ {baud}')

    # ------------------------------------------------------------------ out
    def _to_pwm(self, v):
        pwm = v / self.vfull * 255.0
        if abs(pwm) < 1.0:
            return 0
        mag = max(self.min_pwm, min(self.max_pwm, int(abs(pwm))))   # deadband + speed cap
        return int(math.copysign(mag, pwm))

    def cmd_cb(self, msg: Twist):
        v, w = msg.linear.x, msg.angular.z
        vl = v - w * self.track / 2.0
        vr = v + w * self.track / 2.0
        lp = self.lmd * self._to_pwm(vl)
        rp = self.rmd * self._to_pwm(vr)
        self._send(f'M,{lp},{rp}')

    def fire_cb(self, request, response):
        self._audio_evt.clear()
        self._last_audio = []
        self._send('TAP')
        got = self._audio_evt.wait(timeout=self.tap_timeout)
        if got and len(self._last_audio) > 64:
            m = Float32MultiArray()
            m.data = [float(x) for x in self._last_audio]
            self.audio_pub.publish(m)
            response.success = True
            response.message = f'tap captured ({len(self._last_audio)} samples)'
        else:
            response.success = False
            response.message = 'tap fired but no audio (mic not connected?)'
        return response

    def _send(self, line: str):
        with self.lock:
            try:
                self.ser.write((line + '\n').encode())
            except serial.SerialException as e:
                self.get_logger().error(f'serial write failed: {e}')

    # ------------------------------------------------------------------- in
    def _read_loop(self):
        while rclpy.ok():
            try:
                raw = self.ser.readline().decode(errors='ignore').strip()
            except Exception:
                continue
            if not raw:
                continue
            if raw.startswith('ENC,'):
                self._handle_enc(raw)
            elif raw.startswith('AUDIO'):
                parts = raw.split(',')[1:]
                try:
                    self._last_audio = [int(x) for x in parts]
                except ValueError:
                    self._last_audio = []
                self._audio_evt.set()
            elif raw.startswith('TapMap') or raw.startswith('IMU'):
                self.get_logger().info(f'ESP32: {raw}')

    def _handle_enc(self, raw):
        p = raw.split(',')
        if len(p) < 3:
            return
        try:
            l = self.led * int(p[1])
            r = self.red * int(p[2])
        except ValueError:
            return
        m = Int32MultiArray()
        m.data = [l, r]
        self.ticks_pub.publish(m)

        if len(p) >= 4 and p[3] != 'X':
            try:
                gz_dps = float(p[3])
            except ValueError:
                return
            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = 'imu_link'
            imu.angular_velocity.z = math.radians(gz_dps)
            imu.orientation_covariance[0] = -1.0     # orientation not provided
            self.imu_pub.publish(imu)
            if not self.imu_seen:
                self.imu_seen = True
                self.get_logger().info('IMU gyro streaming — heading fusion active')


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send('M,0,0')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
