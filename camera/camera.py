import cv2 as cv
import time

if __name__ == '__main__':
    camera = cv.VideoCapture(0)  # 获取摄像头
    fps = camera.get(cv.CAP_PROP_FPS)  # 获取帧率
    width = int(camera.get(cv.CAP_PROP_FRAME_WIDTH))  # 一定要转int 否则是浮点数
    height = int(camera.get(cv.CAP_PROP_FRAME_HEIGHT))
    size = (width, height)  # 大小
    VWirte = cv.VideoWriter('123asd.avi', cv.VideoWriter_fourcc('I', '4', '2', '0'), fps, size)  # 初始化文件写入 文件名 编码解码器 帧率 文件大小
    success, frame = camera.read()
    numFramesRemaining = 10 * fps;  # z
    while success and numFramesRemaining:
        VWirte.write(frame)
        success, frame = camera.read()
        numFramesRemaining -= 1
    time.sleep(1)  # y延迟一秒关闭摄像头 否则会出现 terminating async callback 异步处理错误
    camera.release()  # 释放摄像头
    print('ok')