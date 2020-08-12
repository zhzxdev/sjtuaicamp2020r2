#!/usr/bin/env python

import rospy as r


def realmain():
    r.init_node('lanecam', anonymous=True)
    rate = r.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    realmain()
