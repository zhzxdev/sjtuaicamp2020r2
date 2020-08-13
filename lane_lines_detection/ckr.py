import cv2
import numpy as np

LL = 40
RRR = 55
LLL = 40
half_width = 640
half_height = 360

delta = 150
margin = 90
minpix = 25
nwindows = 12
eps = 1e-4
modifier = 0.3
block = 20
block2 = 305
block3 = 100


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
            binary_warped = origin_thr[360:720, ]
            binary_warped = cv2.resize(binary_warped, (0, 0), fx=1, fy=2)

            histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0)  # Left Part
            lane_base = list(filter(lambda x: histogram_x[x] > 5000, range(len(histogram_x))))

            if len(lane_base) == 0:
                self.d = 48
                show('center')
                return self.d

            nonzero = binary_warped.nonzero()
            tmp = []
            for i in range(len(nonzero[0])):
                tmp.append([nonzero[0][i], nonzero[1][i]])
            tmp.sort(lambda p, q: int(q[0] - p[0]))
            nonzeroy = np.array(map(lambda p: p[0], tmp))
            nonzerox = np.array(map(lambda p: p[1], tmp))
            nonzeroy = map(lambda x: 719 - x, nonzeroy)

            if nonzeroy[0] > block3:
                self.d = 48
                show('center')
                return self.d

            hg = nonzeroy[0]

            p = sorted([nonzerox[i] for i in range(len(nonzeroy)) if nonzeroy[i] == hg])
            q = []
            for i in range(1, len(p)):
                if p[i] != p[i - 1] + 1:
                    q.append(i - 1)
            q.append(len(p)-1)
            now = len(q)

            if now >= 2:
                self.d = 48
                show('center')
            else:
                if p[q[0]] < half_width:
                    self.d = RRR
                    show('right')
                else:
                    self.d = LLL
                    show('left')
            return self.d

if __name__ == '__main__':
    cam = camera()
    while True:
        cam.spin()
