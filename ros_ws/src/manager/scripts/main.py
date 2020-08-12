#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import threading

lane_vel = Twist()
angularScale = 6  # 180/30
servodata = 0
traffic_light_data = 0


def lanecallback(msg):
    global lane_vel
    lane_vel = msg
    _servoCmdMsg = msg.angular.z * angularScale + 90
    global servodata
    servodata = min(max(0, _servoCmdMsg), 180)
    servodata = 100 - servodata * 100 / 180


def lightcallback(data):
    global traffic_light_data
    traffic_light_data = data.data


def realmain():
    pub_m = rospy.Publisher('/bluetooth/received/manual', Int32, queue_size=10)
    pub_d = rospy.Publisher('/auto_driver/send/direction', Int32, queue_size=10)
    pub_s = rospy.Publisher('/auto_driver/send/speed', Int32, queue_size=10)
    pub_g = rospy.Publisher('/auto_driver/send/gear', Int32, queue_size=10)

    manul = 0  # 0 - Automatic
    speed = 20  # SPEED
    direction = 50  # 0-LEFT-50-RIGHT-100
    gear = 1  # 1 - Drive, 2 - Stop

    cmd_vel = Twist()
    flag = 0
    p_flag = 1
    servodata_list = []
    n_loop = 1

    rospy.init_node('manager', anonymous=True)

    add_thread = threading.Thread(target=lambda: rospy.spin())

    add_thread.start()

    rate = rospy.Rate(10)
    rospy.Subscriber("/lane_det", Twist, lanecallback)
    rospy.Subscriber("/sign_det", Int32, lightcallback)

    rospy.loginfo(rospy.is_shutdown())
    # n = 1
    # servodata_list = n * [servodata]
    while not rospy.is_shutdown():
        # KINEMATIC CONTROL CODE HERE
        # servodata_list[0:n - 1] = servodata_list[1:n]
        # servodata_list[n - 1] = servodata
        #servodata_mean = np.mean(servodata_list)*n
        # servoSum = 0
        # for i in servodata_list:
        #     servoSum += i

        # servodata_mean = servoSum / n

        # WRITE YOUR CONDITION STATEMENT HERE, PLS FINISH
        # USE (traffic_light_data)
        # TO CHANGE: GEAR, DIRECTION(IF DRIVE, USE servodata_mean)
        # GEAR: 1 - D(RIVE); 2 - N(EUTRAL).
        manul = 1

        pub_m.publish(manul)
        # pub_d.publish(direction)
        # pub_s.publish(speed)
        # pub_g.publish(gear)
        rate.sleep()


if __name__ == '__main__':
    realmain()
