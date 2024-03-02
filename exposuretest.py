import cv2
import numpy as np
import time

#cap = cv2.VideoCapture(1)
codec = 0x47504A4D # MJPG
#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
#cap.set(cv2.CAP_PROP_FOURCC, codec)
#cap.set(cv2.CAP_PROP_EXPOSURE, 70)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#cap.set(cv2.CAP_PROP_FPS, 60)

erosion_kernel = np.ones((6, 6), np.uint8)


start_ts = time.time_ns()
frame = cv2.imread("hi-23.jpeg")
img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
img_threshold = cv2.inRange(img_hsv, (0, 95, 35), (190,255,255))
img_eroded = cv2.erode(img_threshold, erosion_kernel)

contours, hierarchy = cv2.findContours(img_eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

cv2.drawContours(frame, contours, -1, (0,255,0), 3)

largestContour = max(contours, key=cv2.contourArea)
x,y,w,h = cv2.boundingRect(largestContour)
#cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,255),2)
cv2.imshow('frame',frame)
cv2.imshow('threshold', img_eroded)
end_ts = time.time_ns()
total_time = (end_ts - start_ts) / (10 ** 6)
print("time: " + str(total_time))
print("X:" + str(x) + " Y: " + str(y) )
while(True):
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
