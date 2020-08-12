#!/usr/bin/env python

import rospy as r


def realmain():
    pub = r.Publisher('/sign_det', Int32)
    r.init_node('lanecam', anonymous=True)
    rate = r.Rate(10)
    while not r.is_shutdown():
        pub.publish(50)
        rate.sleep()


if __name__ == '__main__':
    realmain()
