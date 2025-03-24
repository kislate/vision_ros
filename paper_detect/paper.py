import cv2
import numpy as np
import argparse

def show_scaled(img, scale=0.2):
    h, w = img.shape[:2]
    resized = cv2.resize(img, (int(w*scale), int(h*scale)), 
                        interpolation=cv2.INTER_AREA)
    cv2.imshow('White Paper Detection', resized)

def adaptive_white_detection(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l_channel = cv2.split(lab)[0]
    
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    enhanced_l = clahe.apply(l_channel)
    
    _, brightness_mask = cv2.threshold(enhanced_l, 200, 255, cv2.THRESH_BINARY)
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    color_mask = cv2.inRange(hsv, 
                           np.array([0, 0, 180]),
                           np.array([255, 80, 255]))
    
    combined_mask = cv2.bitwise_and(brightness_mask, color_mask)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
    processed_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    processed_mask = cv2.morphologyEx(processed_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    
    return processed_mask

def perspective_aware_contour(img, cnt):
    epsilon = 0.03 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    
    if not cv2.isContourConvex(approx):
        approx = cv2.convexHull(approx)
    
    if 4 <= len(approx) <= 6:
        sorted_points = sort_points(approx)
        return sorted_points.reshape(-1,1,2)
    return None

def sort_points(points):
    points = points.reshape(-1,2)
    centroid = np.mean(points, axis=0)
    
    diff = points - centroid
    angles = np.arctan2(diff[:,1], diff[:,0])
    sorted_indices = np.argsort(angles)
    
    return points[sorted_indices]

def detect_white_paper(image_path):
    img = cv2.imread(image_path)
    if img is None:
        print(f"错误：无法读取 {image_path}")
        return

    mask = adaptive_white_detection(img)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        contours = sorted(contours, 
                        key=lambda x: (cv2.contourArea(x), 
                                      cv2.contourArea(x)/(cv2.boundingRect(x)[2]*cv2.boundingRect(x)[3])),
                        reverse=True)
        
        for cnt in contours[:3]:
            if cv2.contourArea(cnt) < 5000:
                continue
            
            processed_cnt = perspective_aware_contour(img, cnt)
            if processed_cnt is not None:
                overlay = img.copy()
                cv2.drawContours(overlay, [processed_cnt], -1, (0,255,0), -1)
                img = cv2.addWeighted(overlay, 0.2, img, 0.8, 0)
                
                cv2.drawContours(img, [processed_cnt], -1, (0,255,0), 3, 
                                lineType=cv2.LINE_AA)
                
                # 计算最小外接矩形
                rect = cv2.minAreaRect(processed_cnt)
                center = rect[0]
                cx, cy = int(center[0]), int(center[1])
                box_points = cv2.boxPoints(rect).astype(int)
                
                # 绘制对角线（黄色）
                cv2.line(img, tuple(box_points[0]), tuple(box_points[2]), (0, 255, 255), 2)
                cv2.line(img, tuple(box_points[1]), tuple(box_points[3]), (0, 255, 255), 2)
                
                # 绘制中心点（红色）
                cv2.circle(img, (cx, cy), 8, (0, 0, 255), -1)
                
                # 显示原始坐标系坐标（左上角原点）
                text = f"Raw Coord: ({cx}, {cy})"
                cv2.putText(img, text, (10, 60), 
                          cv2.FONT_HERSHEY_SIMPLEX, 
                          1.5, 
                          (0, 0, 255), 
                          3,
                          cv2.LINE_AA)
                
                break
    
    show_scaled(img, 0.2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='智能白纸检测')
    parser.add_argument('-i', '--image', required=True, help='输入图片路径')
    args = parser.parse_args()
    detect_white_paper(args.image)