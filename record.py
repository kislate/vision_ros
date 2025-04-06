import cv2
import datetime

def record_video_from_camera(camera_id=1, duration=10, output_folder="."):
    """
    从指定摄像头录制视频并保存到指定文件夹。

    参数:
        camera_id (int): 摄像头的索引号，默认为 1。
        duration (int): 录制时长（秒），默认为 10 秒。
        output_folder (str): 保存视频的文件夹，默认为当前文件夹。
    """
    # 打开摄像头
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 获取摄像头的帧率和分辨率
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 设置视频编码器和输出文件名
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用 MP4 编码格式
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_filename = f"{output_folder}/video_{timestamp}.mp4"
    out = cv2.VideoWriter(output_filename, fourcc, fps, (width, height))

    print(f"开始录制视频，持续时间：{duration} 秒，保存路径：{output_filename}")
    start_time = datetime.datetime.now()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("无法读取帧")
            break

        # 写入帧到视频文件
        out.write(frame)

        # 显示视频窗口（可选）
        cv2.imshow('Recording', frame)

        # 检查是否达到录制时长
        elapsed_time = (datetime.datetime.now() - start_time).total_seconds()
        if elapsed_time >= duration:
            break

        # 按下 'q' 键提前停止录制
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("用户手动停止录制")
            break

    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()

    print(f"视频录制完成，已保存到：{output_filename}")


if __name__ == "__main__":
    # 设置摄像头编号、录制时长和保存路径
    camera_id = 0  # 摄像头编号
    duration = 20  # 录制时长（秒）
    output_folder = "."  # 保存到当前文件夹

    # 调用录制函数
    record_video_from_camera(camera_id, duration, output_folder)