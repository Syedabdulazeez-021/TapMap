#!/usr/bin/env python3
"""
chassis_test.py - quick drive test for the 4-wheel chassis on a Raspberry Pi + L298N.

Wiring (BCM GPIO):
    LEFT  motors (OUT1/OUT2):  IN1=GPIO17, IN2=GPIO27, ENA=GPIO18
    RIGHT motors (OUT3/OUT4):  IN3=GPIO23, IN4=GPIO24, ENB=GPIO13
    L298N GND  ->  Pi GND (pin 6)        (common ground - required)
    12V battery -> L298N +12V            (motors only; Pi on its own USB-C)
    Remove the ENA/ENB jumpers so speed control works (this script PWMs them).

Run:
    python3 chassis_test.py

It will: forward -> back -> spin left -> spin right -> stop.
Press Ctrl-C at any time to stop immediately.

If a wheel spins the wrong way, swap that motor's two wires on the L298N
output (or swap this script's forward/backward pins for that side).
"""
import time
from gpiozero import Motor

# ---- pins (BCM) ----
left  = Motor(forward=17, backward=27, enable=18, pwm=True)   # IN1, IN2, ENA
right = Motor(forward=23, backward=24, enable=13, pwm=True)   # IN3, IN4, ENB

SPEED = 0.6          # 0.0 - 1.0  (medium). Lower if it moves too fast.

def drive(l, r, seconds, label):
    """l, r in [-1..1]: + = that side forward, - = backward, 0 = stop."""
    print(label)
    # left side
    if   l > 0: left.forward(abs(l))
    elif l < 0: left.backward(abs(l))
    else:       left.stop()
    # right side
    if   r > 0: right.forward(abs(r))
    elif r < 0: right.backward(abs(r))
    else:       right.stop()
    time.sleep(seconds)

def stop():
    left.stop()
    right.stop()

try:
    print("Chassis test starting - keep clear of the wheels.")
    time.sleep(1)

    drive( SPEED,  SPEED, 2.0, "Forward")
    drive( 0,      0,     1.0, "Stop")
    drive(-SPEED, -SPEED, 2.0, "Backward")
    drive( 0,      0,     1.0, "Stop")
    drive(-SPEED,  SPEED, 1.5, "Spin LEFT")
    drive( 0,      0,     1.0, "Stop")
    drive( SPEED, -SPEED, 1.5, "Spin RIGHT")
    stop()
    print("Done.")

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    stop()
