import cv2
import numpy as np

img = cv2.imread("images/test.jpg")
if img is None:
    print("❌ 读不到图片")
    exit()

# ── 1. 转 HSV ──
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# ── 2. 定阈值（这里找红色，红跨 0 度所以要两段）──
# lower1, upper1 = np.array([0,   100, 100]), np.array([10,  255, 255])
# lower2, upper2 = np.array([170, 100, 100]), np.array([180, 255, 255])
lower1, upper1 = np.array([35,   100, 100]), np.array([85,  255, 255])

# ── 3. 生成掩码（白色=目标，黑色=背景），两段取并集 ──
# mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
mask = cv2.inRange(hsv, lower1, upper1)

# ── 4. 形态学去噪（去掉小白点，填掉小黑洞）──
kernel = np.ones((5, 5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)   # 开运算：去噪点，去白点
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # 闭运算：补空洞，去黑点

# ── 5. 找轮廓 ──
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(f"找到 {len(contours)} 个候选区域")

result = img.copy()

for c in contours:
    area = cv2.contourArea(c)
    if area < 100:            # 过滤太小的噪点
        continue

    # ── 6. 算中心点（用图像矩）──
    M = cv2.moments(c)
    if M["m00"] == 0:
        continue
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    # ── 7. 画出来 ──
    x, y, w, h = cv2.boundingRect(c)
    cv2.rectangle(result, (x, y), (x + w, y + h), (255, 0, 255), 2)   # 外框
    cv2.circle(result, (cx, cy), 6, (0, 255, 255), -1)                # 中心点
    cv2.putText(result, f"({cx},{cy})", (x, y-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    print(f"  中心: ({cx}, {cy})   面积: {area:.0f}")

# ── 8. 显示 ──
cv2.imshow("Mask", mask)          # 看掩码，调阈值时全靠它！
cv2.imshow("Result", result)
cv2.waitKey(0)
cv2.destroyAllWindows()