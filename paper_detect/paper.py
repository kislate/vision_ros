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

def line_intersection(line1, line2):
    """精确计算两直线交点"""
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2
    
    denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
    if denom == 0:
        return None  # 平行线
    
    px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denom
    py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denom
    
    return (int(px), int(py))

def enforce_quadrilateral(cnt):
    """强制生成有效四边形轮廓"""
    hull = cv2.convexHull(cnt)
    epsilon = 0.02 * cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, epsilon, True)
    
    if len(approx) != 4:
        rect = cv2.minAreaRect(cnt)
        approx = cv2.boxPoints(rect)
    
    return np.int0(approx).reshape(-1,1,2)

def sort_points(points):
    """改进版顶点排序算法"""
    points = points.reshape(-1,2)
    
    # 计算极角排序的质心
    centroid = np.mean(points, axis=0)
    angles = np.arctan2(points[:,1]-centroid[1], points[:,0]-centroid[0])
    
    # 按极角排序
    sorted_indices = np.argsort(angles)
    sorted_points = points[sorted_indices]
    
    # 确保左上角为首个点
    sum_vals = sorted_points.sum(axis=1)
    start_idx = np.argmin(sum_vals)
    sorted_points = np.roll(sorted_points, -start_idx, axis=0)
    
    return sorted_points

def draw_accurate_center(img, sorted_pts):
    """精确绘制对角线交点"""
    if len(sorted_pts) != 4:
        return img
    
    # 定义两条对角线
    diag1 = (tuple(sorted_pts[0]), tuple(sorted_pts[2]))
    diag2 = (tuple(sorted_pts[1]), tuple(sorted_pts[3]))
    
    # 计算精确交点
    intersect = line_intersection(diag1, diag2)
    
    if intersect:
        # 绘制对角线
        cv2.line(img, diag1[0], diag1[1], (0,255,255), 2, lineType=cv2.LINE_AA)
        cv2.line(img, diag2[0], diag2[1], (0,255,255), 2, lineType=cv2.LINE_AA)
        
        # 绘制交点中心
        cv2.circle(img, intersect, 8, (0,0,255), -1)
        cv2.putText(img, f"Center: {intersect}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)
    return img

def detect_white_paper(image_path):
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Cannot read image {image_path}")
        return

    mask = adaptive_white_detection(img)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # 按面积和长宽比综合排序
        contours = sorted(contours,
                        key=lambda x: (cv2.contourArea(x),
                                      cv2.contourArea(x)/cv2.minAreaRect(x)[1][0]/cv2.minAreaRect(x)[1][1]),
                        reverse=True)
        
        for cnt in contours[:3]:
            if cv2.contourArea(cnt) < 5000:
                continue
            
            processed_cnt = enforce_quadrilateral(cnt)
            sorted_pts = sort_points(processed_cnt)
            
            # 几何有效性验证
            if cv2.contourArea(processed_cnt) / (cv2.minAreaRect(processed_cnt)[1][0]*cv2.minAreaRect(processed_cnt)[1][1]) < 0.7:
                continue
            
            # 绘制轮廓
            overlay = img.copy()
            cv2.drawContours(overlay, [processed_cnt], -1, (0,255,0), -1)
            img = cv2.addWeighted(overlay, 0.2, img, 0.8, 0)
            cv2.drawContours(img, [processed_cnt], -1, (0,255,0), 3, cv2.LINE_AA)
            
            # 绘制精确中心
            img = draw_accurate_center(img, sorted_pts)
            break
    
    show_scaled(img, 0.2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='精准白纸检测')
    parser.add_argument('-i', '--image', required=True, help='输入图片路径')
    args = parser.parse_args()
    detect_white_paper(args.image)