import cv2

def main():
    # 打开摄像头
    cap = cv2.VideoCapture(1)  # 使用设备索引0打开摄像头
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 读取一帧
    ret, frame = cap.read()
    if not ret:
        print("无法读取帧")
        cap.release()
        return

    # 获取图像的宽度和高度
    height, width = frame.shape[:2]

    # 打印图像的宽度和高度
    print(f"图像宽度: {width} 像素")
    print(f"图像高度: {height} 像素")

    # 显示图像
    cv2.imshow("Camera Frame", frame)

    # 等待用户按下任意键后关闭窗口
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 释放摄像头资源
    cap.release()

if __name__ == "__main__":
    main()