import cv2
import numpy as np

# ========== 改这里 ==========
SOURCE = "images/test.jpg"      # 图片路径；写 "0" 就切摄像头
# ============================

cap = None
static_img = None

if SOURCE.isdigit():
    cap = cv2.VideoCapture(int(SOURCE), cv2.CAP_V4L2)
    ok, static_img = cap.read()
    if not ok:
        print("❌ 摄像头读取失败"); exit()
else:
    static_img = cv2.imread(SOURCE)
    if static_img is None:
        print("❌ 读不到图片"); exit()

WIN = "Trackbars"
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)

def nothing(x):
    pass

# 6 个滑块：H/S/V 的上下限
# ⚠️ OpenCV 的 H 范围是 0~179（不是 0~360）
cv2.createTrackbar("H min", WIN, 35,  179, nothing)
cv2.createTrackbar("H max", WIN, 85,  179, nothing)
cv2.createTrackbar("S min", WIN, 100, 255, nothing)
cv2.createTrackbar("S max", WIN, 255, 255, nothing)
cv2.createTrackbar("V min", WIN, 100, 255, nothing)
cv2.createTrackbar("V max", WIN, 255, 255, nothing)

print("拖动滑块调阈值")
print("  按 s  → 打印当前阈值（复制到你的脚本里）")
print("  按 q / ESC → 退出")

kernel = np.ones((5, 5), np.uint8)

while True:
    # 取一帧
    if cap is not None:
        ok, img = cap.read()
        if not ok:
            break
    else:
        img = static_img.copy()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 读滑块
    h_min = cv2.getTrackbarPos("H min", WIN)
    h_max = cv2.getTrackbarPos("H max", WIN)
    s_min = cv2.getTrackbarPos("S min", WIN)
    s_max = cv2.getTrackbarPos("S max", WIN)
    v_min = cv2.getTrackbarPos("V min", WIN)
    v_max = cv2.getTrackbarPos("V max", WIN)

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv, lower, upper)

    # 去噪
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 找目标 + 画中心
    result = img.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        if cv2.contourArea(c) < 500:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(result, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv2.circle(result, (cx, cy), 5, (0, 165, 255), -1)

    
    # 把原图、掩码、结果横向拼起来（注意：掩码是单通道，要先转 3 通道）
    mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    combo = np.hstack([mask_3ch, result])
    #cv2.namedWindow("combo", cv2.WINDOW_NORMAL)   # 放在 imshow 之前
    cv2.imshow("All in one", combo)
    '''cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)'''

    key = cv2.waitKey(30) & 0xFF
    if key == ord('q') or key == 27:
        break
    elif key == ord('s'):
        print("\n" + "=" * 40)
        print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"upper = np.array([{h_max}, {s_max}, {v_max}])")
        print("=" * 40 + "\n")

if cap is not None:
    cap.release()
cv2.destroyAllWindows()