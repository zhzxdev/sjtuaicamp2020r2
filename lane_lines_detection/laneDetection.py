import cv2
import math
import numpy as np

delta = 120  # 平移量
margin = 100  # 窗口半宽
minpix = 25  # 车道线最小像素数
nwindows = 10  # 窗口个数


class camera:
    def __init__(self):
        self.cap = cv2.VideoCapture('chanllenge_video.mp4')
        self.aP = [0., 0.]
        self.lastP = [0., 0.]

    def __del__(self):
        self.cap.release()

    def spin(self):
        ret, img = self.cap.read()
        if ret == True:
            cv2.waitKey(1)

            # Convert Image
            gray_Blur = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            kernel = np.ones((3, 3), np.uint8)
            gray_Blur = cv2.erode(gray_Blur, kernel, iterations=1)
            origin_thr = np.zeros_like(gray_Blur)
            origin_thr[(gray_Blur >= 125)] = 255
            src_points = np.array([[3, 570], [387, 460], [906, 452], [1041, 485]], dtype='float32')
            dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype='float32')
            M = cv2.getPerspectiveTransform(src_points, dst_points)
            binary_warped = cv2.warpPerspective(origin_thr, M, (1280, 720), cv2.INTER_LINEAR)

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
            pixelX = nonzerox[lane_inds]
            pixelY = nonzeroy[lane_inds]

            # calculate the aimPoint
            if (pixelX.size == 0):
                return

            frontDistance = np.argsort(pixelY)[int(len(pixelY) / 8)]
            aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]

            self.aP[0] = aimLaneP[0] + delta
            self.aP[1] = aimLaneP[1]
            myK = (self.lastP[1] - self.aP[1]) / (self.lastP[0] - self.aP[0])
            self.lastP = self.aP[:]
            myTheta = math.atan(myK)
            # 构造输出图像
            cv2.putText(binary_warped, str(myTheta), (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            cv2.circle(binary_warped, (int(aimLaneP[0]), int(aimLaneP[1])), 24, (0, 0, 0), 1)
            cv2.circle(binary_warped, (int(self.aP[0]), int(self.aP[1])), 24, (255, 255, 255), 1)
            binary_warped = cv2.resize(binary_warped, (0, 0), fx=.5, fy=.5)
            cv2.imshow('pic', binary_warped)


if __name__ == '__main__':
    cam = camera()
    while True:
        cam.spin()
