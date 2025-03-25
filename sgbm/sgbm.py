import cv2
import numpy as np

# 加载标定参数
calib_data = np.load("stereo_calib_params.npz")
left_mapx = calib_data['left_mapx']
left_mapy = calib_data['left_mapy']
right_mapx = calib_data['right_mapx']
right_mapy = calib_data['right_mapy']
Q = calib_data['Q']

# 初始化摄像头
cap_left = cv2.VideoCapture(0)   # 左摄像头索引
cap_right = cv2.VideoCapture(1)  # 右摄像头索引

# 配置SGBM参数
def create_sgbm():
    min_disparity = 0
    num_disparities = 64   # 必须是16的整数倍
    block_size = 7         # 推荐3-11的奇数
    
    sgbm = cv2.StereoSGBM_create(
        minDisparity=min_disparity,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8 * 3*block_size**2,  # 控制视差平滑度
        P2=32 * 3*block_size**2,
        disp12MaxDiff=1,
        uniquenessRatio=15,
        speckleWindowSize=100,
        speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    return sgbm

sgbm = create_sgbm()

# 创建显示窗口
cv2.namedWindow('Rectified', cv2.WINDOW_NORMAL)
cv2.namedWindow('Disparity', cv2.WINDOW_NORMAL)

while True:
    # 读取原始帧
    ret_l, frame_l = cap_left.read()
    ret_r, frame_r = cap_right.read()
    
    if not ret_l or not ret_r:
        break
    
    # 应用校正映射
    rectified_l = cv2.remap(frame_l, left_mapx, left_mapy, cv2.INTER_LINEAR)
    rectified_r = cv2.remap(frame_r, right_mapx, right_mapy, cv2.INTER_LINEAR)
    
    # 转换为灰度图用于匹配
    gray_l = cv2.cvtColor(rectified_l, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(rectified_r, cv2.COLOR_BGR2GRAY)
    
    # 计算视差
    disparity = sgbm.compute(gray_l, gray_r).astype(np.float32) / 16.0
    
    # 后处理：空洞填充和滤波
    disparity = cv2.medianBlur(disparity, 5)
    mask = disparity == disparity.min()
    disparity[mask] = np.nan
    
    # 可视化
    vis_disparity = cv2.normalize(disparity, None, alpha=0, beta=255,
                                 norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    vis_disparity = cv2.applyColorMap(vis_disparity, cv2.COLORMAP_JET)
    
    # 绘制水平参考线（验证极线对齐）
    for y in [100, 300, 500]:
        cv2.line(rectified_l, (0,y), (rectified_l.shape[1],y), (0,0,255), 1)
        cv2.line(rectified_r, (0,y), (rectified_r.shape[1],y), (0,0,255), 1)
    
    # 拼接显示
    combined = np.hstack([rectified_l, rectified_r])
    cv2.imshow('Rectified', combined)
    cv2.imshow('Disparity', vis_disparity)
    
    # 按键处理
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    elif key == ord('s'):
        cv2.imwrite("disparity.png", vis_disparity)
        print("视差图已保存")

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()