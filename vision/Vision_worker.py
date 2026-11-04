#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视觉检测子进程（ONNX + cv2.dnn，不需要 torch / ultralytics）

由 Corridor_mission.py 用 multiprocessing 启动：
    p = mp.Process(target=start_vision_process, args=(q, ev), daemon=True)
"""
import queue
import time

import cv2
import numpy as np

# ---- 参数 ----
MODEL_PATH = "/home/orangepi/drone/Vision/best.onnx"   # ⚠️ 改成实际路径
CAM_INDEX  = 0
IN_W, IN_H = 320, 320
CONF_THRES = 0.20
# ★ 按类别单独设阈值：Fire_nozzle 模型弱，单独放宽
CLASS_CONF = {
    "Fire_nozzle": 0.18,
}
NMS_THRES  = 0.45

CLASSES = ["Fire_extinguisher", "Fire_hose", "Fire_hydrant",
           "Fire_nozzle", "Rubber_hose", "Valve"]


def letterbox(img, nw, nh, color=(114, 114, 114)):
    h, w = img.shape[:2]
    r = min(nw / w, nh / h)
    sw, sh = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (sw, sh), interpolation=cv2.INTER_LINEAR)
    dw, dh = (nw - sw) // 2, (nh - sh) // 2
    out = cv2.copyMakeBorder(resized, dh, nh - sh - dh,
                             dw, nw - sw - dw,
                             cv2.BORDER_CONSTANT, value=color)
    return out, r, dw, dh


def decode(out, r, dx, dy, fw, fh):
    """YOLOv11 ONNX 输出 -> [(cls_name, conf, x1,y1,x2,y2)]"""
    p = out[0]
    if p.ndim != 2:
        return []
    if p.shape[0] < p.shape[1]:        # (C,N) -> (N,C)
        p = p.T
    if p.shape[1] < 5:
        return []

    b, sc = p[:, :4], p[:, 4:]
    conf = sc.max(axis=1)
    cls = sc.argmax(axis=1)

    thr = np.array(
        [CLASS_CONF.get(CLASSES[i] if 0 <= i < len(CLASSES) else "", CONF_THRES)
         for i in cls.tolist()], dtype=np.float32)
    keep = conf >= thr    
    
    if not keep.any():
        return []

    b, conf, cls = b[keep], conf[keep], cls[keep]
    xyxy = np.concatenate([
        b[:, 0:1] - b[:, 2:3] / 2, b[:, 1:2] - b[:, 3:4] / 2,
        b[:, 0:1] + b[:, 2:3] / 2, b[:, 1:2] + b[:, 3:4] / 2], axis=1)

    idxs = cv2.dnn.NMSBoxes(xyxy.tolist(), conf.tolist(), CONF_THRES, NMS_THRES)
    res = []
    for i in np.array(idxs).flatten():
        x1 = max(0.0, min(fw - 1, (xyxy[i, 0] - dx) / r))
        y1 = max(0.0, min(fh - 1, (xyxy[i, 1] - dy) / r))
        x2 = max(0.0, min(fw - 1, (xyxy[i, 2] - dx) / r))
        y2 = max(0.0, min(fh - 1, (xyxy[i, 3] - dy) / r))
        cid = int(cls[i])
        name = CLASSES[cid] if 0 <= cid < len(CLASSES) else str(cid)
        res.append((name, float(conf[i]), float(x1), float(y1), float(x2), float(y2)))
    return res


def start_vision_process(result_queue, stop_event):
    print(f"[Vision] 加载 {MODEL_PATH}", flush=True)
    net = cv2.dnn.readNetFromONNX(MODEL_PATH)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("[Vision] ❌ 摄像头打不开", flush=True)
        return

    print("[Vision] ✅ 就绪", flush=True)
    n, ft = 0, time.time()
    fps = 0.0

    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.05)
            continue

        lb, r, dx, dy = letterbox(frame, IN_W, IN_H)
        blob = cv2.dnn.blobFromImage(lb, 1 / 255.0, (IN_W, IN_H),
                                     swapRB=True, crop=False)
        net.setInput(blob)
        dets = decode(net.forward(), r, dx, dy,
                      frame.shape[1], frame.shape[0])

        counts = {}
        for name, *_ in dets:
            counts[name] = counts.get(name, 0) + 1

        n += 1
        if n % 15 == 0:
            now = time.time()
            fps = 15.0 / max(1e-3, now - ft)
            ft = now

        while True:                        # 只保留最新
            try:
                result_queue.get_nowait()
            except queue.Empty:
                break
        try:
            result_queue.put({"ts": time.time(), "counts": counts,
                              "dets": dets[:8], "n": n,
                              "fps": round(fps, 1)}, block=False)
        except Exception:
            pass

    cap.release()
    print("[Vision] 退出", flush=True)