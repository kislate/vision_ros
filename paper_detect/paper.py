import cv2
import numpy as np
from collections import deque

def order_points(pts):
    """将四个顶点排序为[左上, 右上, 右下, 左下]"""
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # 左上（x+y最小）
    rect[2] = pts[np.argmax(s)]  # 右下（x+y最大）
    
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # 右上（y-x最小）
    rect[3] = pts[np.argmax(diff)]  # 左下（y-x最大）
    return rect

class PaperTracker:
    def __init__(self, history_sec=5):
        # 维持5秒检测历史（按25FPS计算）
        self.history = deque(maxlen=history_sec * 25)  
        self.stability_threshold = 0.75
        
    def _extract_features(self, cnt):
        """提取轮廓特征"""
        if cnt is None:
            return None
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            return None
        return {
            'contour': cnt,
            'center': (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
            'area': cv2.contourArea(cnt)
        }

    def update(self, cnt):
        """更新检测历史"""
        self.history.append(self._extract_features(cnt))
        
    def get_consensus(self):
        """获取5秒内最稳定检测结果"""
        valid = [f for f in self.history if f is not None]
        if len(valid) < 10:  # 至少需要10帧有效数据
            return None
        
        # 计算位置稳定性（移动平均）
        centers = np.array([f['center'] for f in valid])
        avg_center = np.mean(centers[-50:], axis=0)  # 取最近2秒数据计算平均
        
        # 寻找最接近平均中心的轮廓
        distances = np.linalg.norm(centers - avg_center, axis=1)
        best_match = valid[np.argmin(distances)]
        
        # 面积变化率验证
        recent_areas = [f['area'] for f in valid[-20:]]
        area_cv = np.std(recent_areas) / np.mean(recent_areas)
        
        return best_match['contour'] if area_cv < 0.3 else None

def process_frame(frame, tracker):
    """实时处理帧并标注"""
    # 预处理
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    # 轮廓检测
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:3]  # 取前三轮廓
    
    # 四边形筛选
    paper_contour = None
    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
        if len(approx) == 4:
            paper_contour = approx
            break
    
    # 更新跟踪器
    tracker.update(paper_contour)
    
    # 获取历史共识
    consensus = tracker.get_consensus()
    
    # 可视化标注
    if consensus is not None:
        # 绘制蓝色轮廓
        cv2.drawContours(frame, [consensus], -1, (255, 0, 0), 3)
        
        # 提取四边形顶点
        approx = consensus.reshape(4, 2)
        rect = order_points(approx)
        
        # 转换为整数坐标
        rect = rect.astype("int")
        
        # 绘制对角线（黄色）
        cv2.line(frame, tuple(rect[0]), tuple(rect[2]), (0, 255, 255), 2)  # 主对角线
        cv2.line(frame, tuple(rect[1]), tuple(rect[3]), (0, 255, 255), 2)  # 副对角线
        
        # 计算对角线交点（中心点）
        x = int((rect[0][0] + rect[2][0]) / 2)
        y = int((rect[0][1] + rect[2][1]) / 2)
        
        # 绘制中心红点
        cv2.circle(frame, (x, y), 8, (0, 0, 255), -1)
        
    if paper_contour is not None:
        cv2.drawContours(frame, [paper_contour], -1, (0, 255, 0), 1)
        
    return frame

def video_looper(video_path):
    cap = cv2.VideoCapture(video_path)
    tracker = PaperTracker()
    buffer = []
    
    # 预加载并处理所有帧
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        processed = process_frame(frame.copy(), tracker)
        buffer.append(processed)
    cap.release()
    
    if not buffer:
        print("视频读取失败！")
        return
    
    # 播放控制
    forward = True
    current_frames = buffer.copy()
    
    while True:
        for i in range(len(current_frames)):
            cv2.imshow('Paper Tracking', current_frames[i])
            
            key = cv2.waitKey(25)
            if key == 27:  # ESC退出
                cv2.destroyAllWindows()
                return
            elif key == 32:  # 空格切换方向
                forward = not forward
                current_frames = buffer[::-1] if forward else buffer.copy()
                break
        
        forward = not forward
        current_frames = buffer[::-1] if forward else buffer.copy()

if __name__ == "__main__":
    video_looper("demo.mp4")