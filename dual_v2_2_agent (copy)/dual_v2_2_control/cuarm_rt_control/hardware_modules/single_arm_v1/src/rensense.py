import cv2

# 打开摄像头（0通常是第一个USB摄像头）
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

while True:
    # 读取一帧
    ret, frame = cap.read()
    
    if not ret:
        print("无法接收帧")
        break
    
    # 显示画面
    cv2.imshow('USB Camera', frame)
    
    # 按'q'键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()