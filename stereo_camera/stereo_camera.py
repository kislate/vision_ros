import cv2

# 初始化两个摄像头
cap_left = cv2.VideoCapture(0)  # 左摄像头
cap_right = cv2.VideoCapture(1)  # 右摄像头
import cv2

# 检查摄像头是否成功打开
if not cap_left.isOpened() or not cap_right.isOpened():
    print("无法打开摄像头")
    exit()

# 创建窗口
cv2.namedWindow("Left Camera", cv2.WINDOW_NORMAL)
cv2.namedWindow("Right Camera", cv2.WINDOW_NORMAL)

try:
    while True:
        # 从两个摄像头读取帧
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()

        # 检查是否成功读取帧
        if not ret_left or not ret_right:
            print("无法读取摄像头帧")
            break

        # 显示帧
        cv2.imshow("Left Camera", frame_left)
        cv2.imshow("Right Camera", frame_right)

        # 按下 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # 释放摄像头资源
    cap_left.release()
    cap_right.release()
    # 关闭所有 OpenCV 窗口
    cv2.destroyAllWindows()

    