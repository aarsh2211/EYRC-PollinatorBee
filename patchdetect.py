import cv2 as cv
import numpy as np

frame  = cv.imread("pictureinput.jpg")
red = cv.inRange(frame, (0,0,0), (150,130,255))
blue = cv.inRange(frame, (0,0,0), (255,0,0))
green = cv.inRange(frame, (0,0,0), (0,255,0))
reder, contoursred, hierarchy = cv.findContours(red,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
cv.drawContours(frame, contoursred, -1, (0,255,0), 3)
cv.namedWindow("red")
cv.imshow("red", frame)

cv.waitKey(0)
cv.destroyAllWindows()
