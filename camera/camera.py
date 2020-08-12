import cv2 as cv
import time

output_name = 'output.avi'

if __name__ == '__main__':
    camera = cv.VideoCapture(0)
    fps = camera.get(cv.CAP_PROP_FPS)
    width = int(camera.get(cv.CAP_PROP_FRAME_WIDTH))
    height = int(camera.get(cv.CAP_PROP_FRAME_HEIGHT))
    size = (width, height)
    VWirte = cv.VideoWriter(output_name, cv.VideoWriter_fourcc('I', '4', '2', '0'), fps, size)
    success, frame = camera.read()
    while success:
        VWirte.write(frame)
        success, frame = camera.read()
    time.sleep(1)
    camera.release()
    print('ok')