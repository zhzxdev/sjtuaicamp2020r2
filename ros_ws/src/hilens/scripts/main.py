import rospy as r
import socket
import os

host = '192.168.2.111'
port = 7777


def realmain():
    # print("Module HiLens")
    pub = r.Publisher('/sign_det')
    r.init_node('hilens', anonymous=True)
    rate = r.Rate(10)
    while not r.is_shutdown():
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        data = s.recv(8)
        data = int(data)
        print(data)
        pub.publish(data)
        s.close()
        rate.sleep()


if __name__ == "__main__":
    realmain()
