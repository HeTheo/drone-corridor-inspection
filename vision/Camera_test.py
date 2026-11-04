import cv2
import time

W, H = 320, 240          # 先固定低分辨率，排除带宽问题
COMBOS = [
    ("YUYV", None),      # 不设 BUFFERSIZE，YUYV 最"原始"
    ("YUYV", 1),
    ("MJPG", 1),
    ("MJPG", 4),
    ("MJPG", None),
]


def fourcc_str(v):
    return "".join(chr((v >> 8 * i) & 0xFF) for i in range(4))


def test(fourcc, bufsize):
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    # if bufsize is not None:
        # cap.set(cv2.CAP_PROP_BUFFERSIZE, bufsize)

    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_cc = fourcc_str(int(cap.get(cv2.CAP_PROP_FOURCC)))

    # 预热
    for _ in range(5):
        cap.read()

    # 跑 2 秒统计
    n, bad = 0, 0
    t0 = time.time()
    while time.time() - t0 < 2.0:
        ret, frame = cap.read()
        n += 1
        if not ret or frame is None:
            bad += 1

    dt = time.time() - t0
    fps = n / dt
    cap.release()
    return real_w, real_h, real_cc, fps, bad


def main():
    for fourcc, bufsize in COMBOS:
        r = test(fourcc, bufsize)
        if r is None:
            print(f"{fourcc:5s} bufsize={bufsize}  → 打不开")
            continue
        w, h, cc, fps, bad = r
        flag = "⚠️" if fps > 70 else "  "   # 超过 70fps 基本是假帧率
        print(f"{fourcc:5s} bufsize={bufsize}  → 生效 {w}x{h} {cc}  "
              f"{fps:6.1f}fps  读失败 {bad} 帧 {flag}")


if __name__ == "__main__":
    main()