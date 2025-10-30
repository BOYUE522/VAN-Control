#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import os
import re
import time
import subprocess
from datetime import datetime
from pathlib import Path

TARGET_W, TARGET_H = 1600, 900
SAVE_INTERVAL_SEC = 0.5

# -------- 工具函数：自动查找 Osmo Action 5 Pro 的 /dev/video* ----------
def find_action5_device():
    candidates = []

    # 1) 先看 /dev/v4l/by-id 下的友好名（最稳妥）
    byid = Path("/dev/v4l/by-id")
    if byid.exists():
        for p in byid.iterdir():
            name = p.name.lower()
            if any(k in name for k in ["osmo", "action", "dji"]):
                try:
                    real = p.resolve()
                    if real.exists() and "video" in real.name:
                        candidates.append(str(real))
                except Exception:
                    pass

    # 2) 解析 v4l2-ctl --list-devices（在你的机器上可用）
    if not candidates:
        try:
            out = subprocess.check_output(
                ["v4l2-ctl", "--list-devices"],
                text=True, stderr=subprocess.STDOUT
            )
            blocks = re.split(r"\n\s*\n", out.strip())
            for b in blocks:
                header, *rest = b.splitlines()
                header_l = header.lower()
                if any(k in header_l for k in ["osmo", "action", "dji"]):
                    for line in rest:
                        line = line.strip()
                        if line.startswith("/dev/video"):
                            candidates.append(line)
        except Exception:
            pass

    # 3) 兜底：扫一遍 /dev/video*，挑能打开且分辨率能设置成功的
    if not candidates:
        for vid in sorted(Path("/dev").glob("video*")):
            candidates.append(str(vid))

    # 尝试逐个打开，返回第一个可用的
    for dev in candidates:
        cap = cv2.VideoCapture(dev)
        if not cap.isOpened():
            cap.release()
            continue

        # 先请求目标分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)

        ok, frame = cap.read()
        cap.release()
        if ok and frame is not None:
            return dev

    return None

def main():
    # 在脚本所在目录下创建新文件夹
    script_dir = Path(__file__).resolve().parent
    out_dir = script_dir / f"captures_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"📁 保存目录：{out_dir}")

    device = find_action5_device()
    if not device:
        print("❌ 未找到可用的 Osmo Action 5 Pro 摄像头设备节点（/dev/videoX）。")
        print("   请检查连接，并可用 `v4l2-ctl --list-devices` 或 `v4l2-ctl -d /dev/videoX --all` 辅助定位。")
        return
    print(f"🎥 使用设备：{device}")

    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"❌ 无法打开摄像头 {device}")
        return

    # 请求分辨率（驱动可能会协商到接近值）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)

    print("✅ 开始抓取并预览（每0.5s保存一张，按 q 退出，或 Ctrl+C 中断）")

    last_save = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("⚠️ 无法读取帧，继续重试…")
                time.sleep(0.1)
                continue

            # 统一成 1600x900
            frame = cv2.resize(frame, (TARGET_W, TARGET_H))

            # 预览
            cv2.imshow("Osmo Action 5 Preview (Press q to quit)", frame)

            now = time.time()
            if now - last_save >= SAVE_INTERVAL_SEC:
                last_save = now
                fname = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                fpath = str(out_dir / fname)
                cv2.imwrite(fpath, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
                print(f"💾 已保存：{fpath}")

            # 10ms 响应键盘；q 退出
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n🛑 已停止抓取")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
