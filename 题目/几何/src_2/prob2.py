# rgb颜色空间转换到hsv颜色空间
import cv2 as cv
import numpy as np
import math
def houghcircles(img,rgb_image):
    circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 1,
                              minDist=150,
                              param1=100,
                              param2=18,
                              minRadius=10,
                              maxRadius=300)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x,y,r) in circles:
            cv.circle(rgb_image, (x,y), r, (36, 255, 12), 3)
            cv.circle(rgb_image, (x,y), 2, (0, 0, 255), 3)
            print("Center coordinates is",(x, y))
    return (x, y), r

IMGPath = '..\\Object centroid calculation and mechanical arm rotation Angle calculation\\pic_input.png'
rgb_image = cv.imread(IMGPath)
hsv = cv.cvtColor(rgb_image, cv.COLOR_BGR2HSV)


# 红色
low_hsv = np.array([30,30,30])
high_hsv = np.array([400, 400, 400])

# 利用cv2.inRange函数设阈值，提取特征区域
mask = cv.inRange(hsv,lowerb=low_hsv,upperb=high_hsv)

output_1 = cv.bitwise_and(hsv, hsv, mask=mask)

output_gray = cv.cvtColor(output_1, cv.COLOR_BGR2GRAY)
output_img = cv.medianBlur(output_gray,1)
(x, y), r = houghcircles(output_img, rgb_image)


# sp = rgb_image.shape
# print(sp)
# output = (480, 404, 3)

# 画线
# cv.line(rgb_image,(240,404), (x, y),(0, 255, 0), 5)
# cv.line(rgb_image,(240,404), (240, y),(0, 255, 0), 5)
# cv.line(rgb_image,(240,y), (x, y),(0, 255, 0), 5)
# cv.rectangle(rgb_image,(x - r, y - r), (x + r, y + r), (0, 0, 255), 5)
# cv.imshow("rgb", rgb_image)

cv.line(rgb_image,(202, 480), (x, y),(0, 255, 0), 3)
cv.line(rgb_image,(202, 480), (202, y),(0, 255, 0), 3)
cv.line(rgb_image,(202,y), (x, y),(0, 255, 0), 3)
cv.rectangle(rgb_image,(x - r, y - r), (x + r, y + r), (0, 0, 255), 3)
cv.imshow("rgb", rgb_image)

#计算转动角度
temp = math.hypot(x - 202, 480 - y)
angle = math.asin((x - 202)/temp)*180/math.pi
print("the angle is", angle)
cv.waitKey(0)
