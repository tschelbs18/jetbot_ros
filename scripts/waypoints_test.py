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
    duration = int(round(distance/forward_rate * 10))

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, -right_speed]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg = Float32MultiArray()
    msg.data = [stop, left_speed, right_speed]
    ctr_pub.publish(msg)
    time.sleep(1.0)


def right_turn(turn, left_speed, right_speed, turn_rate=3.02):
    duration = int(round(turn/turn_rate * 10))

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, right_speed]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg = Float32MultiArray()
    msg.data = [stop, left_speed, right_speed]
    ctr_pub.publish(msg)
    time.sleep(1.0)


def left_turn(turn, left_speed, right_speed, turn_rate=1.87):

    duration = int(round(turn/turn_rate * 10))

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, left_speed, -right_speed]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg = Float32MultiArray()
    msg.data = [stop, left_speed, right_speed]
    ctr_pub.publish(msg)
    time.sleep(1.0)


def calculate_distance(x1, x0, y1, y0):

    return np.sqrt(abs(x1-x0)**2 + abs(y1-y0)**2)


def calculate_orientation(x1, x0, y1, y0):

    desired_orientation = round(np.arctan2((y1-y0), (x1-x0)), 2)
    if desired_orientation < 0:
        desired_orientation = 6.28 + desired_orientation

    return desired_orientation


def orientation_difference(theta1, theta0):

    return theta1 - theta0


def move_and_pose_waypoint(x, y, theta, current_x, current_y, current_orientation):
    desired_orientation = calculate_orientation(x, current_x, y, current_y)
    turn_needed = desired_orientation - current_orientation
    if turn_needed > 0:
        left_turn(abs(turn_needed), args.left_turn_speed,
                  args.right_turn_speed)
    elif turn_needed < 0:
        right_turn(abs(turn_needed), args.left_turn_speed,
                   args.right_turn_speed)

    current_orientation = (current_orientation +
                           (desired_orientation-current_orientation)) % 6.28

    distance = calculate_distance(x, current_x, y, current_y)
    move_forward(distance, args.left_forward_speed, args.right_forward_speed)
    current_x, current_y = current_x + \
        (x - current_x), current_y + (y - current_y)

    turn_needed = orientation_difference(theta, current_orientation)
    if turn_needed > 0:
        left_turn(abs(turn_needed), args.left_turn_speed,
                  args.right_turn_speed)
    elif turn_needed < 0:
        right_turn(abs(turn_needed), args.left_turn_speed,
                   args.right_turn_speed)

    current_orientation = (current_orientation + turn_needed) % 6.28

    return current_x, current_y, round(current_orientation, 2)


# initialization
if __name__ == '__main__':
    # Get inputs
    args = parse_args()

    # Read in waypoints
    x = []
    y = []
    theta = []
    with open('data/waypoints.txt') as file:
        lines = file.readlines()

    for line in lines:
        _x, _y, _theta = line.split(',')
        x.append(int(_x))
        y.append(int(_y))
        _theta = float(_theta[:-1])
        if _theta < 0:
            _theta = 6.28 + _theta
        theta.append(round(_theta, 2))

    # setup ros node
    rospy.init_node('jetbot_test')
    ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)

    move = 0.0
    stop = 1.0

    current_x = 0
    current_y = 0
    current_orientation = 0

    for _x, _y, _theta in zip(x, y, theta):
        move_and_pose_waypoint(_x, _y, _theta, current_x,
                               current_y, current_orientation)
