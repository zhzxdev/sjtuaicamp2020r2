#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import threading
import os

################################################################################ DEBUG
debug_disable_hilens = os.getenv('DEBUG_NOHILENS') == '1'
debug_disable_lanecam = os.getenv('DEBUG_NOLANECAM') == '1'
debug_fakerun = os.getenv('DEBUG_FAKERUN') == '1'
debug_enable_pause = True
debug_default_speed = 35
print('################################################################################')
print('HiLens       : ' + str(not debug_disable_hilens))
print('LaneCam      : ' + str(not debug_disable_lanecam))
print('FakeRun      : ' + str(debug_fakerun))
print('Pause        : ' + str(debug_enable_pause))
print('DefaultSpeed : ' + str(debug_default_speed))
print('################################################################################')

################################################################################ STATE
state_manul = 0  # 0 - Automatic
state_speed = debug_default_speed if debug_disable_hilens else 0  # SPEED
state_direction = 49  # 0-LEFT-50-RIGHT-100
state_gear = 1  # 1 - Drive, 2 - Stop
state_onpesd = False
state_paused = False

################################################################################ Publishers
pub_m = rospy.Publisher('/bluetooth/received/manual', Int32, queue_size=10)
pub_d = rospy.Publisher('/auto_driver/send/direction', Int32, queue_size=10)
pub_s = rospy.Publisher('/auto_driver/send/speed', Int32, queue_size=10)
pub_g = rospy.Publisher('/auto_driver/send/gear', Int32, queue_size=10)


def applyState():
    print('--> ', state_manul, state_direction, state_speed, state_gear, state_paused)
    if debug_fakerun:
        return
    pub_m.publish(state_manul)
    pub_d.publish(state_direction)
    pub_s.publish(state_speed)
    pub_g.publish(state_gear)


################################################################################ Callbacks
# data: int
def laneCb(data):
    if state_paused:
        return
    global state_direction
    state_direction = data.data


# data: int, 0 for stop, 1 for slow, 2 for fast
speeds = [0, 25, 35]  # TODO Use real speeds
speed_shift = 10
last_speed = debug_default_speed
cheat_state = 0
cheat_type = 0 # 0 for slow 1 for fast

def signCb(data):
    if state_paused:
        return
    data = data.data
    speed = data & 3
    onpesd = (data & 4) == 4
    global state_speed
    global state_onpesd
    global last_speed
    global cheat_state
    global cheat_type
    state_onpesd = onpesd
    state_speed = speeds[speed] - speed_shift if state_onpesd else speeds[speed]
    if cheat_state == 0:
        if state_speed < last_speed:
            cheat_type = 0
            cheat_state = 1
            state_speed = 0
            cheat_state += 1
        if state_speed > last_speed:
            cheat_type = 1
            cheat_state = 1
            state_speed = 50
            cheat_state += 1
    elif cheat_state == 2:
        last_speed = state_speed
        cheat_type = 0
    else:
        cheat_state += 1
        state_speed = 0 if cheat_state == 0 else 50


def pauseCb(data):
    global state_paused
    global state_speed
    if data.data == 1:
        state_paused = True
        state_speed = 0
    else:
        state_paused = False
        state_speed = debug_default_speed


################################################################################ Main
def realmain():
    global state_manul
    global state_direction
    global state_speed
    global state_gear
    rospy.init_node('manager', anonymous=True)
    threading.Thread(target=lambda: rospy.spin()).start()
    rate = rospy.Rate(20)
    if not debug_disable_lanecam:
        rospy.Subscriber("/lane_det", Int32, laneCb)
    if not debug_disable_hilens:
        rospy.Subscriber("/sign_det", Int32, signCb)
    if debug_enable_pause:
        rospy.Subscriber("/debug/pause", Int32, pauseCb)
    while not rospy.is_shutdown():
        applyState()
        rate.sleep()


if __name__ == '__main__':
    realmain()
