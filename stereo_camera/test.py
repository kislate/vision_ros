import cv2

def main():
    # 打开摄像头
    cap = cv2.VideoCapture(1)  # 把路径改为0，则读取摄像头
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1440)  # 设置宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置高度

    while True:
        # 读取一帧
        ret, img = cap.read()
        if not ret:
            print("无法读取帧")
            break

        # 显示图像
        cv2.imshow("image", img)

        # 按任意键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头资源并关闭窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()