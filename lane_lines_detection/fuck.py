import cv2
import math
import numpy as np

h_start = 645
h_end = 455
r_start = (1279, h_start)
r_end = (915, h_end)
r_cast = (915, 719)
r_orig = (1279, 719)
r_k, r_b = np.polyfit([r_start[1], r_end[1]], [r_start[0], r_end[0]], 1)
print(r_k, r_b)
l_start = (0, h_start)
l_end = (365, h_end)
l_cast = (365, 719)
l_orig = (0, 719)
l_k, l_b = np.polyfit([l_start[1], l_end[1]], [l_start[0], l_end[0]], 1)
print(l_k, l_b)
center_thr = .3
center_lim = 30 * 255

dirs = [49, 55, 45]

class camera:
    def __init__(self):
        self.cap = cv2.VideoCapture("C:\\Users\\Zhang\\Downloads\\out1.mp4")
        # self.cap = cv2.VideoCapture("C:\\Users\\Zhang\\Downloads\\Telegram Desktop\\challenge_video_2 2.mp4")

    def __del__(self):
        self.cap.release()

    def spin(self):
        ret, img = self.cap.read()
        if ret:
            cv2.imshow('raw', img)

            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            img_s = img_hsv[:, :, 1]
            img_v = img_hsv[:, :, 2]
            th1 = 175
            th2 = 50
            step1 = np.zeros_like(img)
            step1[(img_v >= th1) & (img_s <= th2)] = 255

            step2 = cv2.erode(step1, np.ones((3, 3), np.uint8), iterations=1)
            step2 = cv2.dilate(step2, np.ones((3, 3), np.uint8), iterations=1)
            suml, sumr = 0, 0
            for i in range(h_end, h_start):
                limitl = int(l_k * i + l_b)
                suml += np.sum(step2[i][limitl:l_end[0]])
                limitr = int(r_k * i + r_b)
                sumr += np.sum(step2[i][r_end[0]:limitr])
            real_thr = int(max(max(suml, sumr) * center_thr, center_lim))
            cv2.polylines(step2, [np.array([l_start, l_end, l_cast, l_orig])], 1, (255, 0, 0), 5)
            cv2.polylines(step2, [np.array([r_start, r_end, r_cast, r_orig])], 1, (0, 255, 0), 5)
            cv2.putText(step2, str(suml), l_end, cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 5)
            cv2.putText(step2, str(sumr), r_end, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
            state = 0 if abs(suml - sumr) < real_thr else 1 if suml < sumr else 2
            cv2.putText(step2, str(state), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 5)
            cv2.putText(step2, str(real_thr), (50, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
            cv2.putText(step2, str(abs(suml - sumr)), (50, 150), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
            cv2.imshow('s2', step2)
            key = cv2.waitKey(1)
            if key == ord('q'):
                exit(0)
            elif key == ord(' '):
                cv2.waitKey(0)
            return dirs[state]


if __name__ == '__main__':
    cam = camera()
    while True:
        print(cam.spin())
