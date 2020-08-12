#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import threading

################################################################################ STATE
state_manul = 0  # 0 - Automatic
state_speed = 0  # SPEED
state_direction = 50  # 0-LEFT-50-RIGHT-100
state_gear = 1  # 1 - Drive, 2 - Stop

################################################################################ Publishers
pub_m = rospy.Publisher('/bluetooth/received/manual', Int32, queue_size=10)
pub_d = rospy.Publisher('/auto_driver/send/direction', Int32, queue_size=10)
pub_s = rospy.Publisher('/auto_driver/send/speed', Int32, queue_size=10)
pub_g = rospy.Publisher('/auto_driver/send/gear', Int32, queue_size=10)


def applyState():
    print('current state -> ', state_manul, state_direction, state_speed, state_gear)
    pub_m.publish(state_manul)
    pub_d.publish(state_direction)
    pub_s.publish(state_speed)
    pub_g.publish(state_gear)


################################################################################ Callbacks
# data: int
def laneCb(data):
    global state_direction
    state_direction = data.data


# data: int, 0 for stop, 1 for slow, 2 for fast
speeds = [0, 5, 15] # TODO Use real speeds


def signCb(data):
    global state_speed
    state_speed = speeds[data.data]


def realmain():
    global state_manul
    global state_direction
    global state_speed
    global state_gear
    rospy.init_node('manager', anonymous=True)
    threading.Thread(target=lambda: rospy.spin()).start()
    rate = rospy.Rate(10)
    rospy.Subscriber("/lane_det", Int32, laneCb)
    rospy.Subscriber("/sign_det", Int32, signCb)
    # cmd_vel = Twist()
    # flag = 0
    # p_flag = 1
    # servodata_list = []
    # n_loop = 1
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
        applyState()
        rate.sleep()


if __name__ == '__main__':
    realmain()
