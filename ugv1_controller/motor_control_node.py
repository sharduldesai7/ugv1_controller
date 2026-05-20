#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

# ── Configuration ─────────────────────────────────────────────────────────────
PORT          = "/dev/ttyUSB0"   # Change to your port
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
        self.get_logger().info("Run: ros2 run teleop_twist_keyboard teleop_twist_keyboard")

    def cmd_vel_callback(self, msg):
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
