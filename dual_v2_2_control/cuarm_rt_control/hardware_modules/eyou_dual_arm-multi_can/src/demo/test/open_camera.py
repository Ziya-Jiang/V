import numpy as np
import cv2 as cv
import time  # 用于生成唯一文件名

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# 初始化计数器
save_count = 0

while True:
    # 捕获帧
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    # 显示帧
    cv.imshow('frame', frame)
    
    # 键盘监听
    key = cv.waitKey(1) & 0xFF
    
    # 按q退出
    if key == ord('q'):
        break
    
    # 按s保存当前帧（快速保存）
    elif key == ord('s'):
        timestamp = int(time.time() * 1000)
        filename = f'capture_{save_count}.jpg'
        
        # 保存图像（使用高质量JPEG压缩）
        cv.imwrite(filename, frame, [cv.IMWRITE_JPEG_QUALITY, 95])
        print(f"Saved: {filename}")
        save_count += 1

# 释放资源
cap.release()
cv.destroyAllWindows()