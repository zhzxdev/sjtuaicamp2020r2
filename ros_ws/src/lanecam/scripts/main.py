#!/usr/bin/env python

import rospy as r
from std_msgs.msg import Int32

def realmain():
    pub = r.Publisher('/lane_det', Int32)
    r.init_node('lanecam', anonymous=True)
    rate = r.Rate(10)
    while not r.is_shutdown():
        pub.publish(50)
        rate.sleep()


if __name__ == '__main__':
    realmain()
