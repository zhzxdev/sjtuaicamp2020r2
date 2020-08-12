#!/usr/bin/env python

import rospy as r
from std_msgs.msg import Int32

import cv2
import math
import numpy as np

delta = 150  # 平移量
margin = 90  # 窗口半宽
minpix = 25  # 车道线最小像素数
nwindows = 12  # 窗口个数
eps = 1e-4
modifier = 0.3

output_name = 'output.avi'

class camera:
    def __init__(self):
        self.d = 50.0
        self.x = 0.0
        self.last_my_theta = 0
        self.cap = cv2.VideoCapture('chanllenge_video.mp4')
        self.aP = [0., 0.]
        self.lastP = [0., 0.]
        # 录像
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)  # 获取帧率
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))  # 一定要转int 否则是浮点数
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.size = (self.width, self.height)
        self.VWirte = cv2.VideoWriter(output_name, cv2.VideoWriter_fourcc('I', '4', '2', '0'), self.fps,
                                 self.size)  # 初始化文件写入 文件名 编码解码器 帧率 文件大小

    def __del__(self):
        self.cap.release()

    def spin(self):
        ret, img = self.cap.read()
        if ret:
            # 录像
            success, frame = self.cap.read()
            if success:
                self.VWirte.write(frame)

            cv2.waitKey(1)
            gray_blur = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            kernel = np.ones((3, 3), np.uint8)
            gray_blur = cv2.erode(gray_blur, kernel, iterations=1)
            origin_thr = np.zeros_like(gray_blur)
            origin_thr[(gray_blur >= 125)] = 255
            src_points = np.array([[3, 570], [387, 460], [906, 452], [1041, 485]], dtype='float32')
            dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype='float32')
            M = cv2.getPerspectiveTransform(src_points, dst_points)
            binary_warped = cv2.warpPerspective(origin_thr, M, (1280, 720), cv2.INTER_LINEAR)
            # 找左峰
            histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0)  # Left Part
            lane_base = (list(filter(lambda x: histogram_x[x] > 30000, range(binary_warped.shape[0]))))[0]
            # 窗口生长算法
            window_height = int(binary_warped.shape[0] / nwindows)
            nonzero = binary_warped.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            lane_current = lane_base
            lane_inds = []
            for window in range(nwindows):
                win_y_low = binary_warped.shape[0] - (window + 1) * window_height
                win_y_high = binary_warped.shape[0] - window * window_height
                win_x_low = lane_current - margin
                win_x_high = lane_current + margin
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                        nonzerox < win_x_high)).nonzero()[0]
                lane_inds.append(good_inds)
                if len(good_inds) > minpix:
                    lane_current = int(np.mean(nonzerox[good_inds]))
                elif window >= 3:
                    break
            lane_inds = np.concatenate(lane_inds)
            pixel_x = nonzerox[lane_inds]
            pixel_y = nonzeroy[lane_inds]
            # calculate the aimPoint
            if pixel_x.size == 0:
                return
            front_distance = np.argsort(pixel_y)[int(len(pixel_y) / 8)]
            aim_lane_p = [pixel_x[front_distance], pixel_y[front_distance]]

            last_x = sorted(list(nonzerox[i] for i in range(len(nonzeroy)) if nonzeroy[i] == aim_lane_p[1]))[-1]
            if last_x - aim_lane_p[0] > 100:
                self.aP[0] = aim_lane_p[0] + delta
            else:
                self.aP[0] = aim_lane_p[0] - delta
            self.aP[1] = aim_lane_p[1]
            my_k = (self.lastP[1] - self.aP[1]) / (self.lastP[0] - self.aP[0])
            self.lastP = self.aP[:]
            my_theta = math.atan(my_k)
            # 矫正转角
            if my_theta * self.last_my_theta < 0:
                self.x = 0
                self.d = 50
            elif my_theta > 0:
                self.x += modifier
                self.d += self.x
            elif my_theta < 0:
                self.x -= modifier
                self.d += self.x
            self.last_my_theta = my_theta

            # 构造输出图像
            cv2.putText(binary_warped, str(my_theta), (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            cv2.circle(binary_warped, (int(aim_lane_p[0]), int(aim_lane_p[1])), 24, (0, 0, 0), 1)
            cv2.circle(binary_warped, (int(self.aP[0]), int(self.aP[1])), 24, (255, 255, 255), 1)
            binary_warped = cv2.resize(binary_warped, (0, 0), fx=.5, fy=.5)
            cv2.imshow('pic', binary_warped)


def realmain():
    pub = r.Publisher('/lane_det', Int32, queue_size=10)
    r.init_node('lanecam', anonymous=True)
    rate = r.Rate(10)
    cam = camera()
    while not r.is_shutdown():
        pub.publish(cam.spin())
        rate.sleep()


if __name__ == '__main__':
    realmain()
