import cv2
import numpy as np
import math

x_cmPerPixel = 90 / 665.00
y_cmPerPixel = 81 / 680.00
roadWidth = 665

y_offset = 50.0  # cm

I = 58.0
D = 18.0
k = -19

p1 = (0, 465)
p2 = (421, 350)
p3 = (910, 350)
p4 = (1279, 465)

class camera:
    def __init__(self):

        self.camMat = []
        self.camDistortion = []

        self.cap = cv2.VideoCapture('test3.mp4')

        self.aP = [0, 0]
        self.lastP = [0, 0]
        self.Timer = 0
        self.angularScale = 6

    def __del__(self):
        self.cap.release()

    def spin(self):
        ret, img = self.cap.read()
        if ret == True:
            gray_Blur = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            kernel = np.ones((3, 3), np.uint8)
            gray_Blur = cv2.erode(gray_Blur, kernel, iterations=1)
            origin_thr = np.zeros_like(gray_Blur)
            origin_thr[(gray_Blur >= 125)] = 255

            src_points = np.array([p1, p2, p3, p4], dtype="float32")
            dst_points = np.array([[0, 719], [0, 0], [1279, 0], [1279, 719]], dtype="float32")
            M = cv2.getPerspectiveTransform(src_points, dst_points)
            binary_warped = cv2.warpPerspective(origin_thr, M, (1280, 720), cv2.INTER_LINEAR)

            histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0)  # 1/2

            histogram_y = np.sum(binary_warped[0:binary_warped.shape[0], :], axis=1)
            midpoint_y = 320
            upper_half_histSum = np.sum(histogram_y[0:midpoint_y])
            lower_half_histSum = np.sum(histogram_y[midpoint_y:])
            try:
                hist_sum_y_ratio = (upper_half_histSum) / (lower_half_histSum)
            except:
                hist_sum_y_ratio = 1

            lane_base = np.argmax(histogram_x)
            midpoint = int(histogram_x.shape[0] / 2)

            nwindows = 10
            window_height = int(binary_warped.shape[0] / nwindows)
            nonzero = binary_warped.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            lane_current = lane_base
            margin = 100
            minpix = 25

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

            pixelX = nonzerox[lane_inds]
            pixelY = nonzeroy[lane_inds]

            # calculate the aimPoint
            if (pixelX.size == 0):
                return 50

            a2, a1, a0 = np.polyfit(pixelY, pixelX, 2)
            aveX = np.average(pixelX)

            frontDistance = np.argsort(pixelY)[int(len(pixelY) / 8)]
            aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]

            lanePk = 2 * a2 * aimLaneP[0] + a1
            if (abs(lanePk) < 0.1):
                if lane_base >= midpoint:
                    LorR = -1.25
                else:
                    if hist_sum_y_ratio < 0.1:
                        LorR = -1.25
                    else:
                        LorR = 0.8
                self.aP[0] = aimLaneP[0] + LorR * roadWidth / 2
                self.aP[1] = aimLaneP[1]
            else:
                if 2 * a2 * aveX + a1 > 0:
                    if a2 > 0:
                        x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)

                    else:
                        x_intertcept = (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)

                else:
                    if a2 > 0:
                        x_intertcept = (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)

                    else:
                        x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)

                if x_intertcept > 599:
                    LorR = -1.4
                else:
                    LorR = 0.8

                k_ver = - 1 / lanePk

                theta = math.atan(k_ver)
                self.aP[0] = aimLaneP[0] + math.cos(theta) * LorR * roadWidth / 2
                self.aP[1] = aimLaneP[1] + math.sin(theta) * LorR * roadWidth / 2
            # Ruo liang ci mubiaodian xiangcha taida, ze hulue gai mubiaodian
            # Fangzhi bei ganraowu qipian
            if self.lastP[0] == 0 and self.lastP[1] == 0:
                if (((self.aP[0] - self.lastP[0]) ** 2 + (
                        self.aP[1] - self.lastP[1]) ** 2 > 2500) and self.Timer < 2):
                    self.aP = self.lastP[:]
                    self.Timer += 1
                else:
                    self.Timer = 0

            self.lastP = self.aP[:]

            img = cv2.resize(img, (0, 0), fx=.5, fy=.5)
            cv2.imshow('real_world', img)
            binary_warped = cv2.resize(binary_warped, (0, 0), fx=.5, fy=.5)
            cv2.imshow('pic', binary_warped)
            if cv2.waitKey(50) == ord(' '):
                cv2.waitKey(0)


if __name__ == '__main__':
    cam = camera()
    while True:
        cam.spin()


