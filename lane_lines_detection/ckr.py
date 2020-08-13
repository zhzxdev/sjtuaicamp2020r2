import cv2
import numpy as np
import time

block3 = 100
RRR = 55
LLL = 40
half_width = 320
half_height = 180

class camera:
    def __init__(self):
        self.d = 50.0
        self.x = 0.0
        self.last_my_theta = 0
        # self.cap = cv2.VideoCapture('/dev/video10')
        self.cap = cv2.VideoCapture('../media/test3.mp4')
        self.aP = [0., 0.]
        self.lastP = [0., 0.]

    def __del__(self):
        self.cap.release()

    def spin(self):
        ret, img = self.cap.read()
        img = cv2.resize(img, (0, 0), fx=.5, fy=.5)
        def show(direction):
            cv2.putText(img, direction, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            cv2.imshow('img', img)
        if ret:
            cv2.waitKey(1)
            gray_blur = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            kernel = np.ones((3, 3), np.uint8)
            gray_blur = cv2.erode(gray_blur, kernel, iterations=1)
            origin_thr = np.zeros_like(gray_blur)
            origin_thr[(gray_blur >= 125)] = 255
            binary_warped = origin_thr[180:360, ]
            binary_warped = cv2.resize(binary_warped, (0, 0), fx=1, fy=2)

            histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0)  # Left Part
            lane_base = list(filter(lambda x: histogram_x[x] > 2500, range(len(histogram_x))))

            if len(lane_base) == 0:
                self.d = 48
                show('center')
                return self.d

            nonzero = list(zip(*list(binary_warped.nonzero())))
            nonzero.sort(cmp=lambda p, q: int(q[0]-p[0] if p[0]!=q[0] else p[1]-q[1]))
            p = np.zeros(len(nonzero), dtype=int)
            cnt = 0

            if 359 - nonzero[0][0] > block3:
                self.d = 48
                show('center')
                return self.d
            hg = 359 - nonzero[0][0]

            for i in range(len(nonzero)):
                if 359 - nonzero[i][0] == hg:
                    cnt += 1
                    p[cnt] = nonzero[i][1]

            p[0] = 0
            for i in range(len(nonzero) - cnt - 1):
                p[i + cnt + 1] = 10000000

            p = sorted(p)
            q = np.zeros_like(p)
            num = 0
            for i in range(1, cnt + 1):
                if p[i] != p[i - 1] + 1 and i != 1:
                    num += 1
                    q[num] = i - 1
            num += 1
            q[num] = cnt

            q[0] = -1
            now = num

            if now >= 2:
                self.d = 48
                show('center')
            else:
                if p[q[1]] < half_width:
                    self.d = RRR
                    show('right')
                else:
                    self.d = LLL
                    show('left')
            return self.d

if __name__ == '__main__':
    cam = camera()
    while True:
        time_s = time.time()
        cam.spin()
        time_t = time.time()
        print 'time: ' + str(time_t - time_s) + 's'
