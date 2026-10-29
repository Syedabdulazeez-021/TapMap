#!/usr/bin/env python3
"""wifi_bridge: talk to the ESP32 over WiFi (TCP) instead of USB serial.

Drop-in replacement for serial_bridge. Same ROS 2 contract, so
deadreckon_controller / coverage_planner / audio_classifier / defect_mapper
are all unchanged. The ONLY difference is the transport: a TCP socket to
the ESP32's IP instead of a /dev/ttyUSB* serial port.

The ESP32 (WiFi firmware) runs a TCP SERVER on its IP:port; this node is
the client. Put the ESP32's IP (shown on its OLED at boot) into
config/tapmap.yaml -> wifi_bridge: esp32_ip.

READS from ESP32   "ENC,<l>,<r>[,<gz>]" -> /wheel_ticks (Int32MultiArray)
                                          -> /imu/data_raw (Imu, if gz present)
                   "DIST,<mm>"           -> /range_mm (Float32, front VL53)
                   "AUDIO,<s0>,..."      -> /audio/tap_window (Float32MultiArray)
SUBSCRIBES         /cmd_vel -> "M,<l_pwm>,<r_pwm>"
SERVES             /tapper/fire -> "TAP", waits for AUDIO
"""
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class WifiBridge(Node):
    def __init__(self):
        super().__init__('wifi_bridge')
        p = self.declare_parameter
        p('esp32_ip', '192.168.1.50')   # SET to the IP shown on the ESP32 OLED
        p('esp32_port', 8266)
        p('track', 0.122)
        p('v_at_full_pwm', 0.30)
        p('min_pwm', 90)
        p('max_pwm', 255)            # speed cap: lower this to make the robot slower
        p('left_motor_dir', 1)
        p('right_motor_dir', 1)
        p('left_enc_dir', 1)
        p('right_enc_dir', 1)
        p('tap_timeout_s', 4.0)

        g = lambda n: self.get_parameter(n).value
        self.ip = str(g('esp32_ip'))
        self.port = int(g('esp32_port'))
        self.track = float(g('track'))
        self.vfull = float(g('v_at_full_pwm'))
        self.min_pwm = int(g('min_pwm'))
        self.max_pwm = int(g('max_pwm'))
        self.lmd, self.rmd = int(g('left_motor_dir')), int(g('right_motor_dir'))
        self.led, self.red = int(g('left_enc_dir')), int(g('right_enc_dir'))
        self.tap_timeout = float(g('tap_timeout_s'))

        self.sock = None
        self.lock = threading.Lock()
        self._buf = b''

        self.ticks_pub = self.create_publisher(Int32MultiArray, 'wheel_ticks', 20)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 20)
        self.audio_pub = self.create_publisher(Float32MultiArray, 'audio/tap_window', 10)
        self.range_pub = self.create_publisher(Float32, 'range_mm', 10)   # front VL53 distance
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.create_service(Trigger, 'tapper/fire', self.fire_cb)

        self._audio_evt = threading.Event()
        self._last_audio = []
        self.imu_seen = False

        threading.Thread(target=self._read_loop, daemon=True).start()
        self.get_logger().info(f'wifi_bridge up: connecting to ESP32 at {self.ip}:{self.port}')

    # ---------------------------------------------------------------- socket
    def _connect(self):
        while rclpy.ok():
            try:
                s = socket.create_connection((self.ip, self.port), timeout=5.0)
                s.settimeout(0.3)
                with self.lock:
                    self.sock = s
                self.get_logger().info(f'connected to ESP32 at {self.ip}:{self.port}')
                return
            except OSError as e:
                self.get_logger().warn(f'connect failed ({e}); retrying in 2s...')
                time.sleep(2.0)

    # ------------------------------------------------------------------- out
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
            if self.sock is None:
                return
            try:
                self.sock.sendall((line + '\n').encode())
            except OSError as e:
                self.get_logger().error(f'socket write failed: {e}')
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None

    # ------------------------------------------------------------------- in
    def _readline(self):
        while b'\n' not in self._buf:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                return None
            except OSError:
                return b''        # signal broken socket
            if chunk == b'':
                return b''
            self._buf += chunk
        line, self._buf = self._buf.split(b'\n', 1)
        return line

    def _read_loop(self):
        self._connect()
        while rclpy.ok():
            if self.sock is None:
                self._buf = b''
                self._connect()
                continue
            line = self._readline()
            if line is None:
                continue
            if line == b'':
                self.get_logger().warn('ESP32 disconnected; reconnecting...')
                with self.lock:
                    try:
                        self.sock.close()
                    except OSError:
                        pass
                    self.sock = None
                continue
            raw = line.decode(errors='ignore').strip()
            if not raw:
                continue
            if raw.startswith('ENC,'):
                self._handle_enc(raw)
            elif raw.startswith('DIST,'):
                try:
                    mm = float(raw.split(',')[1])
                except (ValueError, IndexError):
                    continue
                self.range_pub.publish(Float32(data=mm))
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
            imu.orientation_covariance[0] = -1.0
            self.imu_pub.publish(imu)
            if not self.imu_seen:
                self.imu_seen = True
                self.get_logger().info('IMU gyro streaming — heading fusion active')


def main():
    rclpy.init()
    node = WifiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send('M,0,0')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
