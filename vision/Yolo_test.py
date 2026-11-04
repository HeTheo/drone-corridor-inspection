#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机载电脑上跑 ONNX 推理（不需要 torch）"""
import cv2, numpy as np, time, os

MODEL   = "/home/orangepi/drone/Vision/best.onnx"
IMGSZ   = 320
CONF    = 0.25
NMS_IOU = 0.45
CLASSES = ["Fire_extinguisher", "Fire_hose", "Fire_hydrant",
           "Fire_nozzle", "Rubber_hose", "Valve"]        # ← 改成你的类别名
SAVE_DIR = "/home/orangepi/drone/Vision/frames"
SAVE_EVERY = 30

os.makedirs(SAVE_DIR, exist_ok=True)
net = cv2.dnn.readNetFromONNX(MODEL)

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("❌ 打不开摄像头"); raise SystemExit


def letterbox(img, new=IMGSZ, color=(114, 114, 114)):
    h, w = img.shape[:2]
    r = min(new / h, new / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (nw, nh))
    canvas = np.full((new, new, 3), color, np.uint8)
    dx, dy = (new - nw) // 2, (new - nh) // 2
    canvas[dy:dy+nh, dx:dx+nw] = resized
    return canvas, r, dx, dy


fps, i = 0.0, 0
print("开始循环（Ctrl+C 停止）\n")
try:
    while True:
        ret, frame = cap.read()
        if not ret: break
        t0 = time.time()

        img, r, dx, dy = letterbox(frame)
        blob = cv2.dnn.blobFromImage(img, 1/255.0, (IMGSZ, IMGSZ), swapRB=True)
        net.setInput(blob)
        out = np.squeeze(net.forward()).T          # [N, 4+nc]

        boxes, scores, ids = [], [], []
        for row in out:
            cs = row[4:]
            c = int(np.argmax(cs)); s = float(cs[c])
            if s < CONF: continue
            cx, cy, bw, bh = row[:4]
            x1 = (cx - bw/2 - dx)/r; y1 = (cy - bh/2 - dy)/r
            boxes.append([x1, y1, bw/r, bh/r]); scores.append(s); ids.append(c)

        idx = cv2.dnn.NMSBoxes(boxes, scores, CONF, NMS_IOU)
        n = 0
        if len(idx) > 0:
            for k in idx.flatten():
                x, y, w, h = boxes[k]
                cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(y+h)), (0,255,0), 2)
                cv2.putText(frame, f"{CLASSES[ids[k]]} {scores[k]:.2f}",
                            (int(x), int(y)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                n += 1

        dt = time.time() - t0
        inst = 1.0/dt if dt > 0 else 0
        fps = inst if fps == 0 else fps*0.9 + inst*0.1
        i += 1
        if i % SAVE_EVERY == 0:
            cv2.putText(frame, f"INF {fps:.1f} det {n}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imwrite(f"{SAVE_DIR}/frame_{i:06d}.jpg", frame)
        if i % 30 == 0:
            print(f"帧 {i:5d} | {fps:5.1f} FPS | 检出 {n}")

except KeyboardInterrupt:
    pass
finally:
    cap.release()