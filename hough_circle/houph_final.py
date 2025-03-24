import cv2
import numpy as np

# --- 参数调节开关 ---
ENABLE_TRACKBARS = True  # 设置为 True 启用参数调节窗口

def enhance_circle_edges(image, clahe_clip_limit=2.0, clahe_tile_grid_size=(8, 8)):
    """增强圆形边缘"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 2)
    clahe = cv2.createCLAHE(clipLimit=clahe_clip_limit, tileGridSize=clahe_tile_grid_size)
    enhanced = clahe.apply(blurred)
    return enhanced

def detect_circles(frame, dp=1, minDist=50, param1=120, param2=70, minRadius=30, maxRadius=250):
    """核心检测逻辑"""
    edge_enhanced = enhance_circle_edges(frame)

    circles = cv2.HoughCircles(
        edge_enhanced,
        cv2.HOUGH_GRADIENT,
        dp=dp,
        minDist=minDist,
        param1=param1,
        param2=param2,
        minRadius=minRadius,
        maxRadius=maxRadius
    )

    valid_circles = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, r = circle[0], circle[1], circle[2]
            valid_circles.append((x, y, r))
    return valid_circles

# --- 参数调节窗口 ---
cv2.namedWindow("TrackBars") # 始终创建窗口
cv2.resizeWindow("TrackBars", 640, 480)

def empty(a):
    pass

# 创建 Trackbar (始终创建，但只在启用时使用)
cv2.createTrackbar("dp", "TrackBars", 1, 5, empty)
cv2.createTrackbar("minDist", "TrackBars", 50, 200, empty)
cv2.createTrackbar("param1", "TrackBars", 89, 255, empty)
cv2.createTrackbar("param2", "TrackBars", 98, 255, empty)
cv2.createTrackbar("minRadius", "TrackBars", 36, 150, empty)
cv2.createTrackbar("maxRadius", "TrackBars", 250, 400, empty)
cv2.createTrackbar("clahe_clip", "TrackBars", 20, 50, empty)
cv2.createTrackbar("clahe_tile", "TrackBars", 8, 20, empty)


# ---------------------- 主程序 ----------------------
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 获取参数 (如果启用 TrackBars)
        if ENABLE_TRACKBARS:
            dp = cv2.getTrackbarPos("dp", "TrackBars")
            minDist = cv2.getTrackbarPos("minDist", "TrackBars")
            param1 = cv2.getTrackbarPos("param1", "TrackBars")
            param2 = cv2.getTrackbarPos("param2", "TrackBars")
            minRadius = cv2.getTrackbarPos("minRadius", "TrackBars")
            maxRadius = cv2.getTrackbarPos("maxRadius", "TrackBars")
            clahe_clip = cv2.getTrackbarPos("clahe_clip", "TrackBars")/10 # Scale the clip limit
            clahe_tile = cv2.getTrackbarPos("clahe_tile", "TrackBars")
            # 检测圆形 (使用可调参数)
            circles = detect_circles(frame, dp, minDist, param1, param2, minRadius, maxRadius)
            enhanced_frame = enhance_circle_edges(frame, clahe_clip, (clahe_tile, clahe_tile)) # Also enhance with tunable parameters
        else:
            # 检测圆形 (使用默认参数)
            circles = detect_circles(frame)
            enhanced_frame = enhance_circle_edges(frame)

        # 绘制结果
        for x, y, r in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

        cv2.imshow("Circle Detection", frame)
        if ENABLE_TRACKBARS:
            cv2.imshow("Enhanced", enhanced_frame) # show result of parameter changes

        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()