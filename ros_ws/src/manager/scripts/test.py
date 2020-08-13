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
    pub_m.publish(state_manul)
    pub_d.publish(state_direction)
    pub_s.publish(state_speed)
    pub_g.publish(state_gear)

################################################################################ Main
def realmain():
    global state_manul
    global state_direction
    global state_speed
    global state_gear
    rospy.init_node('manager', anonymous=True)
    threading.Thread(target=lambda: rospy.spin()).start()
    rate = rospy.Rate(20)
    state_speed = 35
    count = 0
    while not rospy.is_shutdown():
      count += 1
      if count == 50:
        state_speed -= 10
      applyState()
      rate.sleep()


if __name__ == '__main__':
    realmain()
