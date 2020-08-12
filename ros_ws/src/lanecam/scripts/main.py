import rospy as r


def realmain():
    r.init_node('lanecam', anonymous=True)
    rate = r.Rate(10)
    pass


if __name__ == '__main__':
    realmain()
