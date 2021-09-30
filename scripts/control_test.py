#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy


# initialization
if __name__ == '__main__':

        # setup ros node
        rospy.init_node('jetbot_test')
        ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)

        reverse = 0.0
        forward = 1.0
        right = 2.0
        left = 3.0
        stop = 4.0
        speed = 0.2

        # reverse 
        for i in range(10):
            msg = Float32MultiArray()
            msg.data = [reverse, speed]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed]
        ctr_pub.publish(msg)
        time.sleep(1.0)

        # forward
        for i in range(10):
            msg = Float32MultiArray()
            msg.data = [forward, speed]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed]
        ctr_pub.publish(msg)
        time.sleep(1.0)


        # right
        for i in range(10):
            msg = Float32MultiArray()
            msg.data = [right, speed]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed]
        ctr_pub.publish(msg)
        time.sleep(1.0)

        # left
        for i in range(10):
            msg = Float32MultiArray()
            msg.data = [left, speed]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed]
        ctr_pub.publish(msg)
        time.sleep(1.0)


