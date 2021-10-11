#!/usr/bin/env python
import rospy
import time
import numpy as np
import argparse

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy


def parse_args():

    parser = argparse.ArgumentParser(description='Inputs for JetBot')
    parser.add_argument("--left_forward_speed", default=0.75)
    parser.add_argument("--right_forward_speed", default=0.65)
    parser.add_argument("--left_turn_speed", default=0.57)
    parser.add_argument("--right_turn_speed", default=0.55)
    args = parser.parse_args()
    print(args)
    return args


def move_forward(distance, left_speed, right_speed, forward_rate=0.415):
    duration = round(distance/forward_rate * 10)

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, -right_speed]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctr_pub.publish(msg)
    time.sleep(1.0)


def right_turn(turn, left_speed, right_speed, turn_rate=3.02):
    duration = round(turn/turn_rate * 10)

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, right_speed]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctr_pub.publish(msg)
    time.sleep(1.0)


def left_turn(turn, left_speed, right_speed, turn_rate=1.87):

    duration = round(turn/turn_rate * 10)

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, left_speed, -right_speed]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctr_pub.publish(msg)
    time.sleep(1.0)


def calculate_distance(x1, x2, y1, y2):

    return np.sqrt(abs(x1-x2)**2 + abs(y1-y2)**2)


# initialization
if __name__ == '__main__':
    # Get inputs
    args = parse_args()

    # Replace with reading in waypoints
    x = [0, 1, 1, 2, 2, 1, 0]
    y = [0, 0, 1, 1, 2, 1, 0]
    theta = [0, 0, 1.57, 0, -1.57, -0.73, 0]

    # setup ros node
    rospy.init_node('jetbot_test')
    ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)

    move = 0.0
    stop = 1.0

    # 0 -> 1
    move_forward(1, args.left_forward_speed, args.right_forward_speed)
    time.sleep(3.0)

    # 1 -> 2
    left_turn(1.57, args.left_turn_speed, args.right_turn_speed)
    move_forward(1, args.left_forward_speed, args.right_forward_speed)
    time.sleep(3.0)

    # 2 -> 3
    right_turn(1.57, args.left_turn_speed, args.right_turn_speed)
    move_forward(1, args.left_forward_speed, args.right_forward_speed)
    time.sleep(3.0)

    # 3 -> 4
    left_turn(1.57, args.left_turn_speed, args.right_turn_speed)
    move_forward(1, args.left_forward_speed, args.right_forward_speed)
    left_turn(3.14, args.left_turn_speed, args.right_turn_speed)
    time.sleep(3.0)

    # 4 -> 5
    right_turn(0.785, args.left_turn_speed, args.right_turn_speed)
    move_forward(1.41, args.left_forward_speed, args.right_forward_speed)
    left_turn(1.57, args.left_turn_speed, args.right_turn_speed)

    # 5 -> 0
    right_turn(2.355, args.left_turn_speed, args.right_turn_speed)
    move_forward(1.41, args.left_forward_speed, args.right_forward_speed)
    left_turn(2.355, args.left_turn_speed, args.right_turn_speed)
