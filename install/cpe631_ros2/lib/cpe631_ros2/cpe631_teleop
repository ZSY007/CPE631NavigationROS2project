#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.1

MSG = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

ERR_MSG = """
Communications Failed
"""


def get_key(settings, input_stream):
    tty.setraw(input_stream.fileno())
    rlist, _, _ = select.select([input_stream], [], [], 0.1)
    if rlist:
        key = input_stream.read(1)
    else:
        key = ''
    termios.tcsetattr(input_stream, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return f"currently:\tlinear vel {target_linear_vel}\t angular vel {target_angular_vel}"


def make_simple_profile(output, input_value, slop):
    if input_value > output:
        output = min(input_value, output + slop)
    elif input_value < output:
        output = max(input_value, output - slop)
    return output


def constrain(input_value, low, high):
    return max(min(input_value, high), low)


def check_linear_limit_velocity(model, vel):
    if model == "burger":
        return constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    if model in ["waffle", "waffle_pi"]:
        return constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    return constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)


def check_angular_limit_velocity(model, vel):
    if model == "burger":
        return constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    if model in ["waffle", "waffle_pi"]:
        return constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    return constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)


class TeleopNode(Node):
    def __init__(self):
        super().__init__('cpe631_teleop')
        self.declare_parameter('model', 'burger')
        self.model = self.get_parameter('model').get_parameter_value().string_value
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0

    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.control_linear_vel
        msg.twist.angular.z = self.control_angular_vel
        self.publisher.publish(msg)

    def run(self):
        input_stream = sys.stdin
        if not sys.stdin.isatty():
            try:
                input_stream = open('/dev/tty', 'r')
            except OSError:
                print('No TTY available for teleop input.')
                return
        settings = termios.tcgetattr(input_stream)
        try:
            print(MSG)
            status = 0
            while rclpy.ok():
                key = get_key(settings, input_stream)
                if key == 'w':
                    self.target_linear_vel = check_linear_limit_velocity(
                        self.model, self.target_linear_vel + LIN_VEL_STEP_SIZE
                    )
                    status += 1
                    print(vels(self.target_linear_vel, self.target_angular_vel))
                elif key == 's':
                    self.target_linear_vel = check_linear_limit_velocity(
                        self.model, self.target_linear_vel - LIN_VEL_STEP_SIZE
                    )
                    status += 1
                    print(vels(self.target_linear_vel, self.target_angular_vel))
                elif key == 'a':
                    self.target_angular_vel = check_angular_limit_velocity(
                        self.model, self.target_angular_vel + ANG_VEL_STEP_SIZE
                    )
                    status += 1
                    print(vels(self.target_linear_vel, self.target_angular_vel))
                elif key == 'd':
                    self.target_angular_vel = check_angular_limit_velocity(
                        self.model, self.target_angular_vel - ANG_VEL_STEP_SIZE
                    )
                    status += 1
                    print(vels(self.target_linear_vel, self.target_angular_vel))
                elif key == ' ' or key == 'x':
                    self.target_linear_vel = 0.0
                    self.target_angular_vel = 0.0
                    print(vels(self.target_linear_vel, self.target_angular_vel))
                else:
                    if key == '\x03':
                        break

                if status == 20:
                    print(MSG)
                    status = 0

                self.control_linear_vel = make_simple_profile(
                    self.control_linear_vel, self.target_linear_vel, LIN_VEL_STEP_SIZE / 2.0
                )
                self.control_angular_vel = make_simple_profile(
                    self.control_angular_vel, self.target_angular_vel, ANG_VEL_STEP_SIZE / 2.0
                )
                self.publish_twist()

        except Exception as ex:
            print(ERR_MSG)
            print(ex)
        finally:
            self.control_linear_vel = 0.0
            self.control_angular_vel = 0.0
            self.publish_twist()
            termios.tcsetattr(input_stream, termios.TCSADRAIN, settings)
            if input_stream is not sys.stdin:
                input_stream.close()


def main():
    if os.name == 'nt':
        print('This teleop node requires a POSIX terminal.')
        return
    rclpy.init()
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
