import cv2
import math
import numpy as np

x_cmPerPixel = 90 / 665.  # cm -> pixel
y_cmPerPixel = 81 / 680.
delta = 150
y_offset = 50.  # cm
I = 58.  # 轴间距
D = 18.  # 摄像头坐标系与车中心间距
k = -19  # 计算cmdSteer的系数
margin = 100  # 窗口半宽
minpix = 25  # 车道线最小像素数
nwindows = 10  # 窗口个数


class camera:
    def __init__(self):
        self.camMat = []
        self.camDistortion = []
        self.cap = cv2.VideoCapture('chanllenge_video.mp4')
        self.aP = [0., 0.]
        self.lastP = [0., 0.]
        self.Timer = 0
        self.angularScale = 6

    def __del__(self):
        self.cap.release()

    def spin(self):
        ret, img = self.cap.read()
        if ret == True:
            cv2.waitKey(1)
            gray_Blur = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            kernel = np.ones((3, 3), np.uint8)
            gray_Blur = cv2.erode(gray_Blur, kernel, iterations=1)
            origin_thr = np.zeros_like(gray_Blur)
            origin_thr[(gray_Blur >= 125)] = 255

            src_points = np.array([[3, 570], [387, 460], [906, 452], [1041, 485]], dtype="float32")
            dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")
            M = cv2.getPerspectiveTransform(src_points, dst_points)
            binary_warped = cv2.warpPerspective(origin_thr, M, (1280, 720), cv2.INTER_LINEAR)

            histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0)  # Left /2
            histogram_y = np.sum(binary_warped[0:binary_warped.shape[0], :], axis=1)
            midpoint_y = 320  # int(histogram.shape[0]/2)
            upper_half_histSum = np.sum(histogram_y[0:midpoint_y])
            lower_half_histSum = np.sum(histogram_y[midpoint_y:])
            try:
                hist_sum_y_ratio = upper_half_histSum / lower_half_histSum
            except:
                hist_sum_y_ratio = 1

            lane_base = (list(filter(lambda x: histogram_x[x] > 30000, range(binary_warped.shape[0]))))[0]
            # lane_base = np.argmax(histogram_x)
            midpoint = int(histogram_x.shape[0] / 2)

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
                    lane_current = int(np.mean(nonzerox[good_inds]))  ####
                elif window >= 3:
                    break

            lane_inds = np.concatenate(lane_inds)

            pixelX = nonzerox[lane_inds]
            pixelY = nonzeroy[lane_inds]

            # calculate the aimPoint
            if (pixelX.size == 0):
                return

            a2, a1, a0 = np.polyfit(pixelY, pixelX, 2)
            aveX = np.average(pixelX)
            # 区分左右车道线,以计算截距

            frontDistance = np.argsort(pixelY)[int(len(pixelY) / 8)]
            aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]

            # 计算aimLaneP处斜率，从而得到目标点的像素坐标
            lanePk = 2 * a2 * aimLaneP[0] + a1
            if (abs(lanePk) < 0.1):
                if lane_base >= midpoint:
                    LorR = -1.25
                else:
                    if hist_sum_y_ratio < 0.1:
                        LorR = -1.25
                    else:
                        LorR = 0.8
                self.aP[0] = aimLaneP[0] + LorR * delta
                self.aP[1] = aimLaneP[1]
            else:
                if (2 * a2 * aveX + a1) > 0:  # 斜率大于0
                    if a2 > 0:
                        x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)  # 求截距
                    else:
                        x_intertcept = (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)

                else:  # 斜率小于0
                    if a2 > 0:
                        x_intertcept = (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)
                    else:
                        x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)

                if (x_intertcept > 599):
                    LorR = -1.4  # RightLane
                else:
                    LorR = 0.8  # LeftLane

                k_ver = - 1 / lanePk
                theta = math.atan(k_ver)
                self.aP[0] = aimLaneP[0] + math.cos(theta) * LorR * delta
                self.aP[1] = aimLaneP[1] + math.sin(theta) * LorR * delta

            myK = (self.lastP[1] - self.aP[1]) / (self.lastP[0] - self.aP[0])
            self.lastP = self.aP[:]
            myTheta = math.atan(myK)
            cv2.putText(binary_warped, str(myTheta), (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            cv2.circle(binary_warped, (int(aimLaneP[0]), int(aimLaneP[1])), 24, (0, 0, 0), 1)
            cv2.circle(binary_warped, (int(self.aP[0]), int(self.aP[1])), 24, (255, 255, 255), 1)
            binary_warped = cv2.resize(binary_warped, (0, 0), fx=.5, fy=.5)
            cv2.imshow('pic', binary_warped)


if __name__ == '__main__':
    cam = camera()
    while True:
        cam.spin()
