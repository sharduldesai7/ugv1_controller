#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

# ── Configuration ─────────────────────────────────────────────────────────────
PORT          = "/dev/ugv_motor"   # Stable name from scripts/99-ugv1-usb.rules
BAUD_RATE     = 9600
START_BYTE    = 0xAA
BASE_SPEED    = 50               # Max speed scaling (0–255)

STRUCT_FORMAT = "<4B4h"
STRUCT_SIZE   = struct.calcsize(STRUCT_FORMAT)

# ── Motor directions ──────────────────────────────────────────────────────────
FORWARD  = 2
BACKWARD = 1
STOP     = 3

# ── Wheel layout ──────────────────────────────────────────────────────────────
#
#        FRONT
#   [M1]       [M2]
#   (FL)       (FR)
#
#   [M4]       [M3]
#   (BL)       (BR)
#        BACK


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def invert_direction(direction):
    """Swap FORWARD and BACKWARD, leave STOP unchanged."""
    if direction == FORWARD:
        return BACKWARD
    elif direction == BACKWARD:
        return FORWARD
    return STOP


class MotorControlNode(Node):

    def __init__(self):
        super().__init__("motor_control_node")

        # ── Serial setup ──────────────────────────────────────────
        try:
            self.ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"Serial connected on {PORT} @ {BAUD_RATE} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # ── Subscribe to /cmd_vel ─────────────────────────────────
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("Subscribed to /cmd_vel — ready to receive commands")

        # ── Watchdog ──────────────────────────────────────────────
        # If /cmd_vel goes quiet while the motors are running (browser closed,
        # Wi-Fi dropped, publisher crashed), stop them. Set cmd_timeout to 0
        # to disable. Anything that drives the rover must publish faster than
        # this (the web page sends about 20 per second while driving).
        self.declare_parameter("cmd_timeout", 0.5)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self._last_cmd_time = None      # None = nothing received since the last stop
        self._moving = False            # True while the last command was non-zero
        self._last_logged = None
        if self.cmd_timeout > 0:
            self.create_timer(0.05, self.watchdog)
        self.get_logger().info("Run: ros2 run teleop_twist_keyboard teleop_twist_keyboard")

    def watchdog(self):
        if self._last_cmd_time is None:
            return
        if time.monotonic() - self._last_cmd_time > self.cmd_timeout:
            self._last_cmd_time = None
            if not self._moving:
                return                  # already told to stop; nothing to do
            self._moving = False
            self._last_logged = None    # log the next command even if unchanged
            self.get_logger().warn(
                f"No /cmd_vel for {self.cmd_timeout:.1f} s — stopping all motors")
            self.send_motor_data(0, 0, 0, 0, STOP, STOP, STOP, STOP)

    def cmd_vel_callback(self, msg):
        self._last_cmd_time = time.monotonic()
        linear  = msg.linear.x    # +ve = forward, -ve = backward
        angular = msg.angular.z   # +ve = turn left, -ve = turn right

        # ── Differential drive mixing ─────────────────────────────
        left_speed  = linear - (angular / 2.0)
        right_speed = linear + (angular / 2.0)

        # Scale to motor speed range and clamp
        left_speed  = int(clamp(left_speed  * BASE_SPEED, -255, 255))
        right_speed = int(clamp(right_speed * BASE_SPEED, -255, 255))

        # ── Split magnitude and direction ─────────────────────────
        dir_left  = FORWARD if left_speed  > 0 else BACKWARD if left_speed  < 0 else STOP
        dir_right = FORWARD if right_speed > 0 else BACKWARD if right_speed < 0 else STOP

        spd_left  = abs(left_speed)
        spd_right = abs(right_speed)
        self._moving = bool(spd_left or spd_right)

        # ── Send to Arduino ───────────────────────────────────────
        # M1 = FL (wires reversed, so invert direction)
        # M2 = FR, M3 = BR, M4 = BL
        self.send_motor_data(
            speed1=spd_left,   speed2=spd_right,  # M1 (FL), M2 (FR)
            speed3=spd_right,  speed4=spd_left,   # M3 (BR), M4 (BL)
            direction1=invert_direction(dir_left), # M1 inverted
            direction2=dir_right,
            direction3=dir_right,
            direction4=dir_left
        )

        # Log only when the command changes: a driver may publish 20 times a second
        summary = (spd_left, spd_right, dir_left, dir_right)
        if summary != self._last_logged:
            self._last_logged = summary
            self.print_status(spd_left, spd_right, dir_left, dir_right, linear, angular)

    def send_motor_data(self, speed1, speed2, speed3, speed4,
                        direction1, direction2, direction3, direction4):
        try:
            payload = struct.pack(
                STRUCT_FORMAT,
                speed1, speed2, speed3, speed4,
                direction1, direction2, direction3, direction4
            )
            packet = bytes([START_BYTE]) + payload
            self.ser.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def print_status(self, spd_left, spd_right, dir_left, dir_right, linear, angular):
        dir_str = lambda d: "FWD" if d == FORWARD else ("BWD" if d == BACKWARD else "STP")

        # Note: print shows logical/intended direction, not the inverted value sent
        self.get_logger().info(
            f"linear={linear:+.2f} angular={angular:+.2f} | "
            f"M1-FL:{spd_left:3}({dir_str(dir_left)}*) "   # * = inverted on wire
            f"M2-FR:{spd_right:3}({dir_str(dir_right)}) "
            f"M3-BR:{spd_right:3}({dir_str(dir_right)}) "
            f"M4-BL:{spd_left:3}({dir_str(dir_left)})"
        )

    def destroy_node(self):
        """Send stop command and close serial on shutdown."""
        self.get_logger().info("Shutting down — stopping all motors")
        self.send_motor_data(0, 0, 0, 0, STOP, STOP, STOP, STOP)
        self.ser.close()
        super().destroy_node()


# ── Main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    try:
        node = MotorControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
