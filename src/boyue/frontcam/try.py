# show_dji.py
import cv2, time

DEV = "/dev/video5"   # ← 在这里手动改成你的 DJI 节点

cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)
time.sleep(0.2)

if not cap.isOpened():
    raise SystemExit(f"打开失败：{DEV} 可能不是UVC或未处于Webcam模式")

print("按 q 退出")
while True:
    ok, frame = cap.read()
    if not ok: break
    cv2.imshow("DJI Action 5", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
