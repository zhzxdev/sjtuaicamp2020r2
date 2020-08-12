import cv2
import time

video_path = 'test2.mp4'
cap = cv2.VideoCapture(video_path)

def save_image(img):
    file_name = 'export' + time.strftime("%Y%m%d%H%M%S", time.localtime()) + '.png'
    cv2.imwrite(file_name, img)

if __name__ == '__main__':
    ret, img = cap.read()
    while ret:
        key = cv2.waitKey(200)
        img = cv2.resize(img, (0, 0), fx=.5, fy=.5)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow('pic', img)
        if key == ord(' '):
            save_image(img)
        ret, img = cap.read()
    cap.release()