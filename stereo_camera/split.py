import cv2

# 定义全局变量
WIDTH = 640  # 定义图像宽度
HEIGHT = 480  # 定义图像高度

def main():
    while True:
        # 打开摄像头
        capture = cv2.VideoCapture(1)
        if not capture.isOpened():
            print("can not open the camera")
            input("Press Enter to continue...")
            exit(1)

        # 设置摄像头的分辨率
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

        count = 0

        # 定义左右图像的区域
        leftRect = (0, 0, WIDTH // 2, HEIGHT)  # 左图像区域
        rightRect = (WIDTH // 2, 0, WIDTH // 2, HEIGHT)  # 右图像区域

        while True:
            # 读取一帧
            ret, frame = capture.read()
            if not ret:
                print("can not load the frame")
                break

            count += 1
            if count == 1:
                print(frame.shape[1], frame.shape[0])  # 打印图像的宽度和高度

            # 提取左右图像
            leftImg = frame[:, :WIDTH // 2]
            rightImg = frame[:, WIDTH // 2:]

            # 显示左右图像
            cv2.imshow("left", leftImg)
            cv2.imshow("right", rightImg)

            # 按 ESC 键退出
            if cv2.waitKey(30) & 0xFF == 27:
                break

        # 释放摄像头资源并关闭窗口
        capture.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()