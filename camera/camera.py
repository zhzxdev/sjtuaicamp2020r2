import cv2

cap = cv2.VideoCapture('/dev/video10')
width = 1280
ret = cap.set(3, width)
height = 720
ret = cap.set(4, height)
fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
out = cv2.VideoWriter('out.mp4', fourcc, 20, (width, height))
while cap.isOpened():
    ret, frame = cap.read()
    if ret is True:
        out.write(frame)
        cv2.imshow('frame', frame)
    else:
        break
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
