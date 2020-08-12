#!/usr/bin/env python

import rospy as r
from std_msgs.msg import Int32
import socket as S
import os
import struct

sock = S.socket(S.AF_INET, S.SOCK_DGRAM)
sock.bind(('192.168.2.110', 1926))


def realmain():
    # print("Module HiLens")
    pub = r.Publisher('/sign_det', Int32, queue_size=10)
    r.init_node('hilens', anonymous=True)
    # sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    while not r.is_shutdown():
        data, addr = sock.recvfrom(1)
        data = struct.unpack('B', data)[0]
        pub.publish(data)


if __name__ == "__main__":
    realmain()
