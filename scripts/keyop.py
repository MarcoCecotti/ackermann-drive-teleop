#!/usr/bin/env python

'''
ackermann_drive_keyop.py:
    A ros keyboard teleoperation script for ackermann steering based robots
'''

__author__ = 'Marco Cecotti'
__license__ = 'GPLv3'
__maintainer__ = 'Marco Cecotti'

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import sys, select, termios, tty

control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09'}

key_bindings = {
    '\x41' : ( 1.0 , 0.0),
    '\x42' : (-1.0 , 0.0),
    '\x43' : ( 0.0 ,-1.0),
    '\x44' : ( 0.0 , 1.0),
    '\x20' : ( 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0)}


class AckermannDriveKeyop:

    def __init__(self, args):
        if len(args) == 1:
            max_speed = float(args[0])
            max_steering_angle = float(args[0])
        elif len(args) >= 2:
            max_speed = float(args[0])
            max_steering_angle = float(args[1])
        else:
            max_speed = float(0.2)
            max_steering_angle = float(0.7)

        if len(args) > 2:
            cmd_topic = '/' + args[2]
        else:
            cmd_topic = 'cmd_ack'

        self.settings = termios.tcgetattr(sys.stdin)

        self.speed_range = [-max_speed, max_speed]
        self.steering_angle_range = [-max_steering_angle,
                                     max_steering_angle]
        for key in key_bindings:
            key_bindings[key] = \
                    (key_bindings[key][0] * max_speed / 5,
                     key_bindings[key][1] * max_steering_angle / 5)

        self.speed = float(0)
        self.steering_angle = float(0)
        self.motors_pub = rospy.Publisher(
            cmd_topic, AckermannDriveStamped, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        self.print_state()
        self.key_loop()

    def pub_callback(self, event):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.steering_angle = self.steering_angle
        ackermann_cmd_msg.drive.steering_angle_velocity = float(-0.2)
        ackermann_cmd_msg.drive.speed = self.speed
        ackermann_cmd_msg.drive.acceleration = float(-0.4)
        ackermann_cmd_msg.drive.jerk = float(-0.5)
        self.motors_pub.publish(ackermann_cmd_msg)

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse arrows to change speed and steering angle')
        rospy.loginfo('\x1b[1M\rUse space to brake and tab to align wheels')
        rospy.loginfo('\x1b[1M\rPress <ctrl-c> or <q> to exit')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteer Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)
        rospy.loginfo('\x1b[1M\r%s %s', type(self.speed), type(self.steering_angle))

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def clip(self, val, val_min, val_max):
        if val < val_min:
            val = val_min
        if val > val_max:
            val = val_max
        return val

    def key_loop(self):
        while 1:
            key = self.get_key()
            if key in key_bindings.keys():
                if key == control_keys['space']:
                    self.speed = float(0)
                elif key == control_keys['tab']:
                    self.steering_angle = float(0)
                else:
                    self.speed = self.speed + key_bindings[key][0]
                    self.steering_angle = \
                            self.steering_angle + key_bindings[key][1]
                    self.speed = self.clip(
                        self.speed, self.speed_range[0], self.speed_range[1])
                    self.steering_angle = self.clip(
                        self.steering_angle,
                        self.steering_angle_range[0],
                        self.steering_angle_range[1])
                self.print_state()
            elif key == '\x03' or key == '\x71':  # ctr-c or q
                break
            else:
                continue
        self.finalize()

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.steering_angle = float(0)
        ackermann_cmd_msg.drive.steering_angle_velocity = float(0)
        ackermann_cmd_msg.drive.speed = float(0)
        ackermann_cmd_msg.drive.acceleration = float(0)
        ackermann_cmd_msg.drive.jerk = float(0)
        self.motors_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_keyop_node')
    keyop = AckermannDriveKeyop(sys.argv[1:len(sys.argv)])
