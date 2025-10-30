#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2, glob, time, re

W, H, FPS = 1280, 720, 60

def open_cam(dev):
    """尽量用 V4L2 打开；失败就 CAP_ANY；设置 MJPG + 分辨率 + FPS；验证能不能读帧。"""
    key = int(re.search(r'/dev/video(\d+)$', dev).group(1)) if dev.startswith("/dev/video") else dev
    cap = cv2.VideoCapture(key, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release(); cap = cv2.VideoCapture(key, cv2.CAP_ANY)
    if not cap.isOpened(): return None
    try: cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    except Exception: pass
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS,          FPS)
    ok, _ = cap.read()
    if not ok: cap.release(); return None
    return cap

def main():
    # 依次试 /dev/video*，谁能读就用谁（通常 DJI 会有一条能读，另一条是控制/副通道）
    for dev in sorted(glob.glob("/dev/video*")):
        cap = open_cam(dev)
        if cap:
            print(f"✅ 使用摄像头: {dev}  (按 q 退出)")
            t0, cnt, fps = time.time(), 0, "--.-"
            while True:
                ok, frame = cap.read()
                if not ok: break
                cnt += 1
                now = time.time()
                if now - t0 >= 1.0:
                    fps = f"{cnt/(now-t0):.1f}"; t0, cnt = now, 0
                cv2.putText(frame, f"{dev}  {fps} fps", (12, 32),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2, cv2.LINE_AA)
                cv2.imshow("DJI Camera", frame)
                if (cv2.waitKey(1) & 0xFF) == ord('q'): break
            cap.release()
            cv2.destroyAllWindows()
            return
    print("❌ 未找到可用的视频节点（/dev/video*）或无法读帧。")

if __name__ == "__main__":
    # 静音 OpenCV 警告（可选）
    try: cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
    except Exception: pass
    main()
