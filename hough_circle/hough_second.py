# 有参数可调节窗口
import cv2
import numpy as np

def stackImages(scale, imgArray):
    '''
    图像堆栈，可缩放，按列表排列，不受颜色通道限制
    '''
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver

def empty(a):
    pass

cap = cv2.VideoCapture(0)
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 480)

# Hue Min Trackbar: 调整色调的最小值
cv2.createTrackbar("Hue Min", "TrackBars", 32, 179, empty)
# Hue Max Trackbar: 调整色调的最大值
cv2.createTrackbar("Hue Max", "TrackBars", 127, 179, empty)
# Saturation Min Trackbar: 调整饱和度的最小值
cv2.createTrackbar("Sat Min", "TrackBars", 110, 255, empty)
# Saturation Max Trackbar: 调整饱和度的最大值
cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, empty)
# Value Min Trackbar: 调整亮度的最小值
cv2.createTrackbar("Val Min", "TrackBars", 133, 255, empty)
# Value Max Trackbar: 调整亮度的最大值
cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)
# Param1 Trackbar: 调整HoughCircles的param1参数
cv2.createTrackbar("Param1", "TrackBars", 100, 255, empty)
# Param2 Trackbar: 调整HoughCircles的param2参数
cv2.createTrackbar("Param2", "TrackBars", 120, 255, empty)

while True:
    success, img = cap.read()
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
    param1 = cv2.getTrackbarPos("Param1", "TrackBars")
    param2 = cv2.getTrackbarPos("Param2", "TrackBars")
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHSV, lower, upper)
    imgResult = cv2.bitwise_and(img, img, mask=mask)

    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_img = cv2.GaussianBlur(gray, (3, 3), 0)
        circles = cv2.HoughCircles(image=blur_img, method=cv2.HOUGH_GRADIENT, dp=1, minDist=200, param1=param1, param2=param2, minRadius=0, maxRadius=20)
        circles = np.uint16(np.around(circles))

        for i in circles[0, :]:
            # 画圆
            cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 255), 5)
            # 画圆心
            cv2.circle(img, (i[0], i[1]), 2, (0, 255, 255), 3)
            print('圆心坐标为（%.2f,%.2f）' % (i[0], i[1]))
            a = str(i[0])
            b = str(i[1])
            img = cv2.putText(img, "(" + a + " ," + b + ")", (i[0], i[1]), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
    except:
        print('无法识别到圆')
        imgStack = stackImages(0.6, ([gray, imgHSV, img], [mask, imgResult, img]))
    
    imgStack = stackImages(0.6, ([gray, imgHSV, img], [mask, imgResult, img]))
    cv2.imshow("Stack Images", imgStack)

    if cv2.waitKey(1) & 0xff == ord('q'):
        break

