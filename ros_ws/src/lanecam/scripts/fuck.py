#!/usr/bin/env python

import cv2
import math
import numpy as np
import rospy as r
from std_msgs.msg import Int32

################################################################################ DEBUG
DEBUG = True
################################################################################ PUBLISHERS
pub_r = r.Publisher('/lane_det', Int32, queue_size=10)
pub_p = r.Publisher('/debug/pause', Int32, queue_size=10)
################################################################################ CONST
dirs = [51, 0, 70]
gap = 24
pesd_shift = 0
################################################################################ HOTSPOTS
margin = 30
h_start = 645 - margin
h_end = 455 - margin
h_hyper = 400 - margin

r_start = (1279, h_start)
r_end = (915, h_end)
r_k, r_b = np.polyfit([r_start[1], r_end[1]], [r_start[0], r_end[0]], 1)
r_hyper = (int(r_k * h_hyper + r_b), h_hyper)
# print(r_k, r_b)
l_start = (0, h_start)
l_end = (365, h_end)
l_k, l_b = np.polyfit([l_start[1], l_end[1]], [l_start[0], l_end[0]], 1)
l_hyper = (int(l_k * h_hyper + l_b), h_hyper)
# print(l_k, l_b)
################################################################################ MAGICS
thr_common_base = .3
thr_common_min = 30
thr_binary_v = 175
thr_binary_s = 50
thr_pesd_area = int((h_start - h_end) * (1280 - gap - gap - l_end[0]) * .7 * .5)
thr_pesd_base = .3
thr_pesd_min = 5
thr_pesd_force = int((h_start - h_end) * (1280 - r_end[0]) * .35 * .5)


class camera:
    def __init__(self):
        self.cap = cv2.VideoCapture("/dev/video10")
        self.cap.set(3, 1280)
        self.cap.set(4, 720)
        # self.cap = cv2.VideoCapture("C:\\Users\\Zhang\\Downloads\\Telegram Desktop\\challenge_video_2 2.mp4")

    def __del__(self):
        self.cap.release()

    def spin(self):
        ret, img = self.cap.read()
        if ret:
            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            img_s = img_hsv[:, :, 1]
            img_v = img_hsv[:, :, 2]
            step1 = np.zeros_like(img)
            step1[(img_v >= thr_binary_v) & (img_s <= thr_binary_s)] = 255

            step2 = cv2.erode(step1, np.ones((3, 3), np.uint8), iterations=1)
            step2 = cv2.dilate(step2, np.ones((3, 3), np.uint8), iterations=1)
            rl, rr = 0, 0
            for i in range(h_end, h_start):
                rl += np.sum(step2[i][:int(l_k * i + l_b)]) / 255
                rr += np.sum(step2[i][int(r_k * i + r_b):]) / 255
            sl = np.sum(step2[h_end:h_start, :640 - gap]) / 255 - rl
            sr = np.sum(step2[h_end:h_start, 640 + gap:]) / 255 - rr
            realthr_common = int(max(max(sl, sr) * thr_common_base, thr_common_min))
            state = 0 if abs(sl - sr) < realthr_common else 1 if sl < sr else 2
            is_pesd = False
            shift = 0
            if sl > thr_pesd_area and sr > thr_pesd_area:
                is_pesd = True
                if rr < thr_pesd_force:
                    state = 1
                    cv2.putText(step2, 'F L(' + str(rr) + '/' + str(thr_pesd_force) + ')', (50, 200), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 5)
                elif rl < thr_pesd_force:
                    state = 2
                    cv2.putText(step2, 'F R(' + str(rl) + '/' + str(thr_pesd_force) + ')', (50, 200), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 5)
                else:
                    psl = np.sum(step2[h_hyper:h_end, :640 - gap]) / 255
                    psr = np.sum(step2[h_hyper:h_end, 640 + gap:]) / 255
                    for i in range(h_hyper, h_end):
                        psl -= np.sum(step2[i][:int(l_k * i + l_b)]) / 255
                        psr -= np.sum(step2[i][int(r_k * i + r_b):]) / 255
                    realthr_pesd = int(max(max(psl, psr) * thr_pesd_base, thr_pesd_min))
                    state = 0 if abs(psl - psr) < realthr_pesd else 1 if psl < psr else 2
                    if DEBUG:
                        cv2.putText(step2, str(realthr_pesd), (50, 200), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
                        cv2.line(step2, l_hyper, (640 - gap, h_hyper), (127, 127, 0), 5)
                        cv2.line(step2, (640 - gap, h_hyper), (640 - gap, h_end), (127, 127, 0), 5)
                        cv2.line(step2, l_end, l_hyper, (127, 0, 0), 5)
                        cv2.line(step2, r_hyper, (640 + gap, h_hyper), (0, 127, 127), 5)
                        cv2.line(step2, (640 + gap, h_hyper), (640 + gap, h_end), (0, 127, 127), 5)
                        cv2.line(step2, r_end, r_hyper, (0, 127, 0), 5)
                        cv2.putText(step2, str(psl), l_hyper, cv2.FONT_HERSHEY_COMPLEX, 1, (127, 0, 0), 5)
                        cv2.putText(step2, str(psr), r_hyper, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 127, 0), 5)
                        cv2.putText(step2, str(abs(psl - psr)), (50, 250), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
            if DEBUG:
                cv2.line(step2, l_end, (640 - gap, h_end), (255, 255, 0), 5)
                cv2.line(step2, (640 - gap, h_end), (640 - gap, 719), (255, 255, 0), 5)
                cv2.line(step2, l_start, l_end, (255, 0, 0), 5)
                cv2.line(step2, r_end, (640 + gap, h_end), (0, 255, 255), 5)
                cv2.line(step2, (640 + gap, h_end), (640 + gap, 719), (0, 255, 255), 5)
                cv2.line(step2, r_start, r_end, (0, 255, 0), 5)
                cv2.putText(step2, str(sl), l_end, cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 5)
                cv2.putText(step2, str(sr), r_end, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
                cv2.putText(step2, str(state) + ',' + str(is_pesd), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 5)
                cv2.putText(step2, str(realthr_common), (50, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
                cv2.putText(step2, str(abs(sl - sr)), (50, 150), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
                step2 = cv2.resize(step2, (0, 0), fx=.5, fy=.5)
                cv2.imshow('s2', step2)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    exit(0)
                elif key == ord(' '):
                    pub_p.publish(1)
                    cv2.waitKey(0)
                    pub_p.publish(0)
            return max(min(dirs[state] + shift, 100), 0)
        else:
            raise "Fuck!!!"


def realmain():
    r.init_node('lanecam', anonymous=True)
    rate = r.Rate(10)
    cam = camera()
    while not r.is_shutdown():
        pub_r.publish(cam.spin())
        rate.sleep()


if __name__ == '__main__':
    realmain()
