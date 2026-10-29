#!/usr/bin/env python3
"""
wander.py - autonomous MARCH-and-TAP explorer for TapMap (obstacle-aware).

Run AFTER launching the robot (ros2 launch tapmap robot_wifi.launch.py), in a
second SOURCED terminal:

    python3 ~/wander.py                 # defaults: 90 s run
    python3 ~/wander.py 120             # run for 120 seconds
    python3 ~/wander.py 120 0.22 400    # run_time  step_speed  obstacle_mm

Behaviour (the stop-tap-move cycle, like the grid scan):
  STEP   - drive forward a SHORT burst (STEP_TIME), then stop
  TAP    - stop, fire the tapper, record + classify the sound
  then it senses the front VL53 and either steps again or, if something is
  within OBSTACLE_MM, it STOPS WELL SHORT, backs up and TURNS before moving on.
Because it stops to sense before and during every step, it cannot drive into a
wall. After RUN_TIME seconds it stops and exits. Ctrl-C stops it anytime (the
ESP32 deadman also halts the motors ~0.4 s after the last command).

Uses only /cmd_vel, /range_mm and the /tapper/fire service - no package rebuild.
"""
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tapmap_interfaces.msg import TapResult   # /tap/classification (hollow / solid)


class Wander(Node):
    def __init__(self, run_time, step_speed, obstacle_mm):
        super().__init__('wander_controller')

        # ------------------------- tunables -------------------------
        self.RUN_TIME      = float(run_time)        # seconds, then stop
        self.STEP_SPEED    = float(step_speed)      # forward speed during a step (m/s)
        self.STEP_TIME     = 0.6                     # how long each forward step lasts (s)
        self.BACKUP_SPEED  = 0.22                    # reverse speed while backing off (m/s)
        self.TURN          = 4.0                     # turn command (rad/s): high so the bridge
                                                     #   saturates both sides to max_pwm for the
                                                     #   full-power 4-wheel point turn
        self.OBSTACLE_MM   = float(obstacle_mm)      # if something is closer, DON'T step - turn
        self.HARD_STOP_MM  = 180.0                   # if this close mid-step, stop instantly
        self.CLEAR_MM      = self.OBSTACLE_MM + 200  # treat as clear once farther than this
        self.BACKUP_TIME   = 0.8                     # back up this long before turning (clearance)
        self.TURN_MIN_TIME = 1.0                     # commit to turning at least this long
        self.TURN_MAX_TIME = 2.6                     # give up turning after this and step again
        self.BLOCK_HITS    = 2                       # blocked reads in a row before reacting (ignore blips)
        self.TAP           = True                    # tap at the end of every step
        # ------------------------------------------------------------

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Float32, 'range_mm', self._range_cb, 10)
        self.create_subscription(TapResult, 'tap/classification', self._result_cb, 10)
        self.fire = self.create_client(Trigger, 'tapper/fire')

        self.n_hollow = 0
        self.n_solid = 0

        self.dist = -1.0            # latest distance mm (-1 = no / invalid reading)
        self.state = 'STEP'         # STEP | TAP | BACKUP | TURN
        self.step_start = self.now()
        self.backup_start = 0.0
        self.turn_start = 0.0
        self.block_count = 0
        self.start = self.now()
        self.fire_future = None

        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz
        self.get_logger().info(
            'wander (march+tap) up: run %.0fs, step %.2f m/s, avoid<%.0fmm'
            % (self.RUN_TIME, self.STEP_SPEED, self.OBSTACLE_MM))

    # ----------------- helpers -----------------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _range_cb(self, msg):
        self.dist = msg.data

    def _result_cb(self, msg):
        if msg.label == 'hollow':
            self.n_hollow += 1
        else:
            self.n_solid += 1
        total = self.n_hollow + self.n_solid
        self.get_logger().info(
            '  >>> %-6s  conf=%.2f  at (%.2f, %.2f)   [total %d | hollow %d | solid %d]'
            % (msg.label.upper(), msg.confidence, msg.x, msg.y,
               total, self.n_hollow, self.n_solid))

    def send(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.pub.publish(t)

    def blocked(self):
        return 0.0 < self.dist < self.OBSTACLE_MM

    def too_close(self):
        return 0.0 < self.dist < self.HARD_STOP_MM

    def is_clear(self):
        return self.dist < 0.0 or self.dist > self.CLEAR_MM

    def start_step(self, t):
        self.state = 'STEP'
        self.step_start = t
        self.block_count = 0

    def start_avoid(self, t):
        self.send(0.0, 0.0)
        self.state = 'BACKUP'
        self.backup_start = t
        self.block_count = 0

    def begin_tap(self, t):
        self.send(0.0, 0.0)
        if self.TAP and self.fire.service_is_ready():
            self.fire_future = self.fire.call_async(Trigger.Request())
            self.state = 'TAP'
        else:
            self.start_step(t)     # no tapper available - just keep marching

    # ----------------- 10 Hz loop -----------------
    def tick(self):
        t = self.now()

        # time limit -> stop and quit
        if t - self.start > self.RUN_TIME:
            self.send(0.0, 0.0)
            self.get_logger().info('run time reached - stopping')
            self.timer.cancel()
            rclpy.shutdown()
            return

        # TAP: hold still until the tap finishes, then sense and decide
        if self.state == 'TAP':
            self.send(0.0, 0.0)
            if self.fire_future is not None and self.fire_future.done():
                try:
                    res = self.fire_future.result()
                    self.get_logger().info('tap: %s' % res.message)
                except Exception as e:
                    self.get_logger().warn('tap failed: %s' % e)
                self.fire_future = None
                if self.blocked():
                    self.get_logger().info('path blocked (%.0f mm) - turning' % self.dist)
                    self.start_avoid(t)
                else:
                    self.start_step(t)
            return

        # BACKUP: reverse briefly to make room before spinning
        if self.state == 'BACKUP':
            self.send(-self.BACKUP_SPEED, 0.0)
            if (t - self.backup_start) > self.BACKUP_TIME:
                self.state = 'TURN'
                self.turn_start = t
            return

        # TURN: spin in place at full power, committed for a time then re-check
        if self.state == 'TURN':
            self.send(0.0, self.TURN)
            turned = t - self.turn_start
            if turned > self.TURN_MAX_TIME or (turned > self.TURN_MIN_TIME and self.is_clear()):
                self.start_step(t)
            return

        # STEP: short forward burst, but bail out the instant anything is close
        if self.state == 'STEP':
            if self.too_close():                 # emergency: stop and turn NOW
                self.get_logger().info('too close (%.0f mm) - stopping' % self.dist)
                self.start_avoid(t)
                return
            if self.blocked():
                self.block_count += 1
            else:
                self.block_count = 0
            if self.block_count >= self.BLOCK_HITS:   # confirmed obstacle ahead
                self.get_logger().info('obstacle (%.0f mm) - stopping short, turning' % self.dist)
                self.start_avoid(t)
                return
            if (t - self.step_start) > self.STEP_TIME:   # step done -> stop and tap
                self.begin_tap(t)
                return
            self.send(self.STEP_SPEED, 0.0)
            return

        # safety net: unknown state -> step
        self.start_step(t)


def main():
    run_time = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    step     = float(sys.argv[2]) if len(sys.argv) > 2 else 0.22
    obstacle = float(sys.argv[3]) if len(sys.argv) > 3 else 400.0

    rclpy.init()
    node = Wander(run_time, step, obstacle)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send(0.0, 0.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
