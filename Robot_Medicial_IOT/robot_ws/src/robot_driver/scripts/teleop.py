#!/usr/bin/env python3

import os
import select
import sys

from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# Simple velocity limits for differential drive robot
MAX_LIN_VEL = 0.5
MAX_ANG_VEL = 2.0

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.2

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\1xb':
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key2 = sys.stdin.read(2)
                if key2 == "[A":
                    return 'up'
                elif key2 == '[B':
                    return 'down'
                elif key2 =='[D':
                    return 'left'
                elif key2 == '[C':
                    return 'right'
        else:
            return key
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0:.2f}\t angular velocity {1:.2f} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output_vel, input_vel, slop):
    if input_vel > output_vel:
        output_vel = min(input_vel, output_vel + slop)
    elif input_vel < output_vel:
        output_vel = max(input_vel, output_vel - slop)
    else:
        output_vel = input_vel

    return output_vel


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, '/cmd_vel', qos)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key == 'w' or key =='W':
                target_linear_velocity = check_linear_limit_velocity(
                    target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x' or key =='X':
                target_linear_velocity = check_linear_limit_velocity(
                    target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a' or key =='A':
                target_angular_velocity = check_angular_limit_velocity(
                    target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd' or key =='D':
                target_angular_velocity = check_angular_limit_velocity(
                    target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's' or key == 'S':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == "left":
                target_angular_velocity = check_angular_limit_velocity(MAX_ANG_VEL)
            elif key == "right":
                target_angular_velocity = check_angular_limit_velocity(-MAX_ANG_VEL)
            elif key == "up":
                target_linear_velocity = check_linear_limit_velocity(MAX_LIN_VEL)
            elif key == "down":
                target_linear_velocity = check_linear_limit_velocity(-MAX_LIN_VEL)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist = Twist()
            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()