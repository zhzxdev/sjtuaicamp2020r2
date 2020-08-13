import cv2
import math
import numpy as np

################################################################################ CONST
dirs = [49, 55, 45]
gap = 24
################################################################################ HOTSPOTS
margin = 30
h_start = 645 - margin
h_end = 455 - margin

r_start = (1279, h_start)
r_end = (915, h_end)
r_k, r_b = np.polyfit([r_start[1], r_end[1]], [r_start[0], r_end[0]], 1)
# print(r_k, r_b)
l_start = (0, h_start)
l_end = (365, h_end)
l_k, l_b = np.polyfit([l_start[1], l_end[1]], [l_start[0], l_end[0]], 1)
# print(l_k, l_b)
################################################################################ MAGICS
thr_common_base = .3
thr_common_min = 30 * 255
thr_binary_v = 175
thr_binary_s = 50
thr_pesd_area = (h_start - h_end) * (1280 - gap - gap - l_end[0]) * 255 * .7
thr_pesd_base = .4
thr_pesd_min = 10 * 255


class camera:
    def __init__(self):
        self.cap = cv2.VideoCapture("C:\\Users\\Zhang\\Downloads\\testpesd.mp4")
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
            step1 = np.zeros_like(img)
            step1[(img_v >= thr_binary_v) & (img_s <= thr_binary_s)] = 255

            step2 = cv2.erode(step1, np.ones((3, 3), np.uint8), iterations=1)
            step2 = cv2.dilate(step2, np.ones((3, 3), np.uint8), iterations=1)
            dltl, dltr = 0, 0
            for i in range(h_end, h_start):
                limitl = int(l_k * i + l_b)
                dltl += np.sum(step2[i][:limitl])
                limitr = int(r_k * i + r_b)
                dltr += np.sum(step2[i][limitr:])
            suml = np.sum(step2[h_end:h_start, :640 - gap]) - dltl
            sumr = np.sum(step2[h_end:h_start, 640 + gap:]) - dltr
            realthr_common = int(max(max(suml, sumr) * thr_common_base, thr_common_min))
            state = 0 if abs(suml - sumr) < realthr_common else 1 if suml < sumr else 2
            is_pesd = False
            if suml + sumr > thr_pesd_area:
                is_pesd = True
                realthr_pesd = int(max(max(dltl, dltr) * thr_pesd_base, thr_pesd_min))
                state = 0 if abs(dltl - dltr) < realthr_pesd else 1 if dltl > dltr else 2
                cv2.putText(step2, str(realthr_pesd), (50, 200), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
                cv2.putText(step2, str(abs(dltl - dltr)), (50, 250), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)

            cv2.line(step2, (0, h_end), (640 - gap, h_end), (255, 255, 0), 5)
            cv2.line(step2, (640 - gap, h_end), (640 - gap, 719), (255, 255, 0), 5)
            cv2.line(step2, l_start, l_end, (255, 0, 0), 5)
            cv2.line(step2, (1279, h_end), (640 + gap, h_end), (0, 255, 255), 5)
            cv2.line(step2, (640 + gap, h_end), (640 + gap, 719), (0, 255, 255), 5)
            cv2.line(step2, r_start, r_end, (0, 255, 0), 5)
            cv2.putText(step2, str(suml), l_end, cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 5)
            cv2.putText(step2, str(sumr), r_end, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
            cv2.putText(step2, str(state) + ',' + str(is_pesd), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 5)
            cv2.putText(step2, str(realthr_common), (50, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 5)
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
        ans = cam.spin()
        if ans == None:
            break
        print(ans)
