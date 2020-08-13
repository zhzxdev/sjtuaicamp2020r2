import cv2
import time
import numpy as np

video_path = 'test4.mp4'
cap = cv2.VideoCapture(video_path)

def save_image(img):
    file_name = 'export' + time.strftime("%Y%m%d%H%M%S", time.localtime()) + '.png'
    cv2.imwrite(file_name, img)

if __name__ == '__main__':
    ret, img = cap.read()
    while ret:
        key = cv2.waitKey(50)
        # img = cv2.resize(img, (0, 0), fx=.5, fy=.5)
        H, W, _ = img.shape
        tmp = cv2.resize(img, (0, 0), fx=.5, fy=.5)
        cv2.imshow('raw', tmp)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_s = img_hsv[:, :, 1]
        img_v = img_hsv[:, :, 2]
        th1 = 175
        th2 = 50
        step2 = np.zeros_like(img)
        step2[(img_v >= th1) & (img_s <= th2)] = 255
        step3 = cv2.erode(step2, np.ones((3, 3), np.uint8), iterations=1)
        step3 = cv2.dilate(step3, np.ones((3, 3), np.uint8), iterations=1)
        # src_points = np.array([[3, 570], [387, 460], [906, 452], [1041, 485]], dtype='float32')
        # dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype='float32')
        # M = cv2.getPerspectiveTransform(src_points, dst_points)
        # step3 = cv2.warpPerspective(step3, M, (W, H), cv2.INTER_LINEAR)
        # cv2.imshow('out', step3)
        if key == ord(' '):
            save_image(img)
        elif key == ord('p'):
            cv2.waitKey(0)
        ret, img = cap.read()
    cap.release()