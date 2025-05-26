#!/usr/bin/env python3
import cv2
import numpy as np
import time
import sys

def sort_circles_by_radius_and_position(circles):
    # 先依半徑大小排序（由大到小）
    sorted_by_radius = sorted(circles, key=lambda c: c[2], reverse=True)
    labels = ['a', 'b', 'c']  # 最大圓為a，第二大b，最小c

    # 加上標籤
    labeled = [(labels[i], (int(c[0]), int(c[1])), int(c[2])) for i, c in enumerate(sorted_by_radius[:3])]

    # 依 x 座標排序（由左到右）
    sorted_left_to_right = sorted(labeled, key=lambda x: x[1][0])

    return labeled, sorted_left_to_right

# 開啟攝影機（嘗試多個攝影機索引）
def try_cameras():
    # Try different camera indices (0, 1, 2, etc.)
    for i in range(3):  # Try cameras 0, 1, 2
        print(f"嘗試開啟攝影機 {i}...")
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"✓ 成功開啟攝影機 {i}")
                return cap, i
            cap.release()
    
    # If we're here, no camera worked
    print("❌ 無法開啟任何攝影機")
    print("請確認以下事項:")
    print(" 1. 攝影機已正確連接")
    print(" 2. VMware 已設定將攝影機通過到虛擬機")
    print(" 3. Ubuntu 已取得攝影機權限")
    return None, -1

cap, camera_index = try_cameras()
if cap is None:
    sys.exit(1)

# Set resolution to a lower value if needed
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("按 Enter 鍵擷取圖片並分析")
print("按 q 鍵退出")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 無法讀取影像幀，重試中...")
        time.sleep(1)
        continue

    # 預處理
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 1)
        edges = cv2.Canny(blur, 50, 150)
    except Exception as e:
        print(f"處理影像時發生錯誤: {e}")
        continue

    # 偵測圓形
    try:
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=40,
            param1=100,
            param2=65,
            minRadius=50,
            maxRadius=100
        )
    except Exception as e:
        print(f"偵測圓形時發生錯誤: {e}")
        circles = None

    output = frame.copy()

    if circles is not None:
        circles = np.uint16(np.around(circles[0]))
        for (x, y, r) in circles:
            cv2.circle(output, (x, y), r, (0, 255, 0), 2)
            cv2.circle(output, (x, y), 2, (0, 0, 255), 3)
        
        # Show circle count in the image
        cv2.putText(output, f"Circles: {len(circles)}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # 顯示畫面
    try:
        cv2.imshow("Canny + Hough + Sorted Circles", output)
    except Exception as e:
        print(f"顯示影像時發生錯誤: {e}")

    try:
        key = cv2.waitKey(1) & 0xFF
    except Exception as e:
        print(f"等待按鍵時發生錯誤: {e}")
        key = 255  # No key

    # 按 Enter 鍵輸出排序後的圓資訊與左到右排序
    if key == 13:  # Enter 鍵 ASCII 是 13
        if circles is not None and len(circles) >= 3:
            selected = circles[:3]
            labeled, sorted_left_to_right = sort_circles_by_radius_and_position(selected)

            print("偵測到三個河內塔圓盤，依半徑大小排序：")
            for label, center, radius in labeled:
                print(f"{label}: 位置={center}, 半徑={radius} 像素(px)")

            # 輸出影像中由左到右的標籤排序
            order_str = ' '.join([label for label, _, _ in sorted_left_to_right])
            print(f"影像中左到右排序為：{order_str}")

        else:
            print(f"⚠️ 偵測到的圓形數量不足: {0 if circles is None else len(circles)}")
            print("請重新調整攝影機角度或光線條件")

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("程式已結束")