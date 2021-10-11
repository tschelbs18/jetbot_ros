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

        move = 0.0
        stop = 1.0


        # 0->1
        speed_l = -0.75
        speed_r = -0.65
        for i in range(24):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(5.0)

        # 1->2
        speed_l = 0.62
        speed_r = -0.60
        for i in range(8):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(2.0)

        speed_l = -0.74
        speed_r = -0.70
        for i in range(24):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(5.0)

        # 2->3
        speed_l = -0.62
        speed_r = 0.60
        for i in range(5):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(2.0)
 
        speed_l = -0.74
        speed_r = -0.70
        for i in range(24):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(5.0)

        # 3->4
        speed_l = 0.62
        speed_r = -0.60
        for i in range(8):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(2.0)
 
        speed_l = -0.74
        speed_r = -0.70
        for i in range(24):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(2.0)
 
        speed_l = 0.62
        speed_r = -0.60
        for i in range(17):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(5.0)
 
         # 5->6
        speed_l = -0.62
        speed_r = 0.60
        for i in range(3):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(2.0)
 
        speed_l = -0.74
        speed_r = -0.70
        for i in range(34):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(2.0)
 
        speed_l = 0.62
        speed_r = -0.60
        for i in range(8):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(5.0)
 
         # 5->6
        speed_l = -0.62
        speed_r = 0.60
        for i in range(5):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(2.0)
 
        speed_l = -0.74
        speed_r = -0.70
        for i in range(34):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(2.0)
 
        speed_l = 0.62
        speed_r = -0.60
        for i in range(13):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(5.0)
 




