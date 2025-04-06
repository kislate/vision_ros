import cv2
import numpy as np

def create_repeat_video(input_path, output_path, repeat_times):
    # 读取原始视频
    cap = cv2.VideoCapture(input_path)
    
    # 获取视频参数
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # 读取所有帧到内存
    frames = []
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(frame)
    cap.release()
    
    # 生成倒序帧序列
    reversed_frames = frames[::-1]
    
    # 创建视频写入对象
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    # 重复生成正反播放序列
    for _ in range(repeat_times):
        # 正序写入
        for frame in frames:
            out.write(frame)
        # 倒序写入
        for frame in reversed_frames:
            out.write(frame)
    
    out.release()
    print(f"生成完成！总播放次数：{2*repeat_times}次")

# 使用示例
create_repeat_video("demo.mp4", "output.mp4", 20)