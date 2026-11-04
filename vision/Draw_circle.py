import cv2
import numpy as np

# 创建一个 480x640 的黑色画布（注意：高在前，宽在后）
img = np.zeros((480, 640, 3), dtype=np.uint8)

# 画三个圆：注意颜色是 BGR 顺序！
cv2.circle(img, (160, 240), 70, (0, 0, 255), -1)     # 红
cv2.circle(img, (320, 240), 70, (0, 255, 0), -1)     # 绿
cv2.circle(img, (480, 240), 70, (255, 0, 0), -1)     # 蓝

# 加一点噪点，让后面的形态学处理有意义
noise = np.random.randint(0, 255, img.shape, dtype=np.uint8)
img = cv2.addWeighted(img, 0.9, noise, 0.1, 0)

cv2.imwrite("images/test.jpg", img)
print("✅ 已生成 images/test.jpg")