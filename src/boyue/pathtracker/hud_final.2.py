#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import math
import threading
from datetime import datetime
from collections import deque
import json

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ================== 车辆/模型参数 ==================
SR = 15.7                           # steering_wheel_deg / SR = road_wheel_deg
WHEELBASE_FT = 12.0
WHEELBASE_M  = WHEELBASE_FT * 0.3048  # 3.6576 m

# ================== 话题（按需修改） ==================
TOPIC_STEER = "/vehicle/steering/report"      # ds_dbw_msgs/SteeringReport
TOPIC_VEL   = "/vehicle/vehicle_velocity"     # ds_dbw_msgs/VehicleVelocity

# 同时兼容两种命名风格：/vehicle/throttle/report 与 /vehicle/throttle_report
TOPIC_BRAKE_CANDIDATES    = ["/vehicle/brake/report", "/vehicle/brake_report"]
TOPIC_THROTTLE_CANDIDATES = ["/vehicle/throttle/report", "/vehicle/throttle_report"]

from ds_dbw_msgs.msg import SteeringReport, VehicleVelocity
try:
    from ds_dbw_msgs.msg import BrakeReport
except Exception:
    BrakeReport = None
try:
    from ds_dbw_msgs.msg import ThrottleReport
except Exception:
    ThrottleReport = None

# ================== 频率与文件输出 ==================
TARGET_HZ_TRAJ   = 30.0           # 轨迹积分/打印频率
FRAME_HZ         = 10.0           # 相机抓帧频率
LOG_TRAJ_NAME    = "trajectory_log.txt"       # 30Hz 累积轨迹（t,x,y）
FRAME_LABEL_NAME = "frame_future_traj.txt"    # JSONL（每张图一个对象）
DRIVE_LOG_NAME   = "frame_drive_log.txt"      # 10Hz 车辆+图片日志（旁路日志）
DATASET_SUBDIR   = os.path.join("samples", "CAM_FRONT")

# 运行目录结构（本次运行专属）
RUN_DIR    = os.path.join(os.getcwd(), f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
IMAGES_DIR = os.path.join(RUN_DIR, "images")   # 根图像目录
LABELS_DIR = os.path.join(RUN_DIR, "labels")
IMAGES_DS_DIR = os.path.join(IMAGES_DIR, DATASET_SUBDIR)  # images/samples/CAM_FRONT

# ================== 相机参数（固定为你的示例） ==================
CAMERA_INTRINSIC = [
    [613.422945, 0.0,        816.249047],
    [0.0,        614.078062, 491.50706579294757],
    [0.0,        0.0,        1.0]
]
CAMERA_EXTRINSIC = [
    [ 0.005684778682715578, -0.9999835174398974,  0.0008050713376760109, -1.70079118954],
    [-0.0056366677335981845,-0.0008371152724919503,-0.9999837634756282,  -0.0159456324149],
    [ 0.9999679551206575,    0.005680148462035599,-0.005641333641939506, -1.51095763913],
    [ 0.0,                   0.0,                  0.0,                   1.0]
]
CAMERA_TRANSLATION_INV = [-1.70079118954, -0.0159456324149, -1.51095763913]
CAMERA_ROTATION_MATRIX_INV = [
    [ 0.005684778682715578,  -0.9999835174398974,   0.0008050713376760109],
    [-0.0056366677335981845, -0.0008371152724919503,-0.9999837634756282],
    [ 0.9999679551206575,     0.005680148462035599, -0.005641333641939506]
]

# ================== 可视化（轨迹） ==================
WIN_TRAJ = "Relative Trajectory (30 Hz)"
CANVAS_W, CANVAS_H = 800, 800
BG   = (0, 0, 0)
FG   = (255, 255, 255)
ACC  = (120, 200, 255)
TRJ  = (80, 240, 80)
CUR  = (0, 0, 255)
GRID = (40, 40, 40)
VIS_Y_SIGN = +1   # 仅显示用：屏幕上方为 +y；数据坐标系仍是左正右负

# ================== 可视化（相机） ==================
WIN_CAM = "DJI Action 5 (1600x900, 10 Hz)"
OUT_W, OUT_H = 1600, 900
ACTION5_DEVICE = "/dev/video0"    # 你的设备

# ================== 共享状态（含 yaw 历史） ==================
class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        # 传感
        self.steer_wheel_deg = 0.0
        self.speed_mps       = 0.0
        self.last_steer_ts   = 0.0
        self.last_speed_ts   = 0.0
        # 刹车/油门（尽力从不同字段名里自适应）
        self.brake_pressure = None     # 原始数值（常见 bar/kPa），此处不换单位
        self.throttle_pct   = None     # 0~100（若来的是 0~1 则会×100）
        self.throttle_deg   = None     # 若有角度字段则显示角度
        self.last_brake_ts  = 0.0
        self.last_thr_ts    = 0.0
        # 轨迹
        self.t0   = time.time()
        self.last = self.t0
        self.x = 0.0
        self.y = 0.0              # 左正右负（右移为负）
        self.yaw = 0.0            # 以启动朝向为0（弧度，左转为正）
        # 历史 (t, x, y, yaw)
        self.history = deque(maxlen=200000)
        self.history.append((0.0, 0.0, 0.0, 0.0))
        # 最新图片的相对路径（nuScenes 风格）
        self.latest_img_relpath = ""   # e.g., "samples/CAM_FRONT/....jpg"

S = Shared()

# ================== ROS2 订阅节点 ==================
class SteerSub(Node):
    def __init__(self):
        super().__init__("traj_steer_reader")
        # 改为 RELIABLE，和 Dataspeed 常见设置一致
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(SteeringReport, TOPIC_STEER, self.cb, qos)

    def cb(self, msg: SteeringReport):
        angle = None
        for k in ("steering_wheel_angle", "wheel_angle_deg", "steering_angle_deg"):
            if hasattr(msg, k):
                angle = float(getattr(msg, k))
                break
        if angle is None:
            angle = 0.0
        with S.lock:
            S.steer_wheel_deg = angle
            S.last_steer_ts = time.time()

class VelSub(Node):
    def __init__(self):
        super().__init__("traj_vel_reader")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(VehicleVelocity, TOPIC_VEL, self.cb, qos)

    def cb(self, msg: VehicleVelocity):
        v = None
        for k in ("speed_mps", "vehicle_velocity", "vehicle_velocity_brake", "vehicle_speed", "speed"):
            if hasattr(msg, k):
                v = float(getattr(msg, k))
                break
        if v is None:
            v = 0.0
        with S.lock:
            S.speed_mps = v
            S.last_speed_ts = time.time()

class BrakeSub(Node):
    def __init__(self):
        super().__init__("traj_brake_reader")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._got_once = False
        if BrakeReport is None:
            self.get_logger().warn("BrakeReport 未导入成功，刹车订阅已跳过。")
            return
        # 同时订阅两种候选话题
        for t in TOPIC_BRAKE_CANDIDATES:
            self.create_subscription(BrakeReport, t, self.cb, qos)

    def cb(self, msg):
        pres = None
        for k in ("pressure", "brake_pressure", "brake_pressure_kpa",
                  "brakeLinePressure", "brake_pressure_bar",
                  "pressure_output","pressure_input"):
            if hasattr(msg, k):
                pres = float(getattr(msg, k))
                break
        with S.lock:
            S.brake_pressure = pres
            S.last_brake_ts = time.time()
        if not self._got_once:
            self._got_once = True
            self.get_logger().info("已收到刹车消息。")

class ThrottleSub(Node):
    def __init__(self):
        super().__init__("traj_throttle_reader")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._got_once = False
        if ThrottleReport is None:
            self.get_logger().warn("ThrottleReport 未导入成功，油门订阅已跳过。")
            return
        for t in TOPIC_THROTTLE_CANDIDATES:
            self.create_subscription(ThrottleReport, t, self.cb, qos)

    def cb(self, msg):
        pct = None
        deg = None
        # 百分比类字段
        for k in ("percent_output","percent_input","pedal_input","pedal_cmd",
                  "throttle","throttle_pedal","throttle_pct"):
            if hasattr(msg, k):
                val = float(getattr(msg, k))
                pct = val * 100.0 if 0.0 <= val <= 1.0 else val
                break
        # 角度类字段
        for k in ("pedal_angle_deg", "throttle_angle_deg", "angle_deg"):
            if hasattr(msg, k):
                deg = float(getattr(msg, k))
                break
        with S.lock:
            S.throttle_pct = pct
            S.throttle_deg = deg
            S.last_thr_ts = time.time()
        if not self._got_once:
            self._got_once = True
            self.get_logger().info("已收到油门消息。")

def ros_spin(stop_evt: threading.Event):
    rclpy.init()
    n1 = SteerSub()
    n2 = VelSub()
    n3 = BrakeSub()
    n4 = ThrottleSub()
    exe = rclpy.executors.SingleThreadedExecutor()
    exe.add_node(n1); exe.add_node(n2); exe.add_node(n3); exe.add_node(n4)
    try:
        while not stop_evt.is_set():
            exe.spin_once(timeout_sec=0.05)
    finally:
        for n in (n1, n2, n3, n4):
            try:
                exe.remove_node(n)
                n.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()

# ================== 工具 & UI 辅助 ==================
def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def world_to_canvas(xy_m: np.ndarray, scale_m_per_pix: float, cx:int, cy:int):
    if xy_m.size == 0:
        return np.zeros((0,2), dtype=np.int32)
    px = xy_m[:,0] / scale_m_per_pix
    py = xy_m[:,1] / scale_m_per_pix * VIS_Y_SIGN
    u = np.round(cx + px).astype(np.int32)
    v = np.round(cy - py).astype(np.int32)
    return np.stack([u, v], axis=1)

def draw_grid(img, cx, cy, scale_m_per_pix):
    step_m = 5.0
    step_px = max(1, int(round(step_m / scale_m_per_pix)))
    H, W = img.shape[:2]
    for x in range(cx, W, step_px):
        cv2.line(img, (x, 0), (x, H), GRID, 1, cv2.LINE_AA)
    for x in range(cx, -1, -step_px):
        cv2.line(img, (x, 0), (x, H), GRID, 1, cv2.LINE_AA)
    for y in range(cy, H, step_px):
        cv2.line(img, (0, y), (W, y), GRID, 1, cv2.LINE_AA)
    for y in range(cy, -1, -step_px):
        cv2.line(img, (0, y), (W, y), GRID, 1, cv2.LINE_AA)
    cv2.line(img, (cx-10, cy), (cx+10, cy), (80,80,200), 2, cv2.LINE_AA)
    cv2.line(img, (cx, cy-10), (cx, cy+10), (80,80,200), 2, cv2.LINE_AA)

def interpolate_txyyaw(history_list, t_query):
    if not history_list:
        return 0.0, 0.0, 0.0
    if t_query <= history_list[0][0]:
        _, x, y, yaw = history_list[0]; return x, y, yaw
    if t_query >= history_list[-1][0]:
        _, x, y, yaw = history_list[-1]; return x, y, yaw
    lo, hi = 0, len(history_list)-1
    while lo <= hi:
        mid = (lo + hi) // 2
        tm = history_list[mid][0]
        if tm < t_query:
            lo = mid + 1
        else:
            hi = mid - 1
    i1 = max(1, lo); i0 = i1 - 1
    t0, x0, y0, yaw0 = history_list[i0]
    t1, x1, y1, yaw1 = history_list[i1]
    if t1 == t0:
        return x0, y0, yaw0
    a = (t_query - t0) / (t1 - t0)
    yaw = yaw0 + a*(yaw1 - yaw0)
    return x0 + a*(x1-x0), y0 + a*(y1-y0), yaw

class LatestFrame:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None
    def set(self, img):
        with self._lock:
            self._frame = img
    def get(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

latest_cam_frame = LatestFrame()

# ================== 轨迹线程（30 Hz） ==================
def traj_thread(stop_evt, traj_log_f):
    period = 1.0 / TARGET_HZ_TRAJ
    while not stop_evt.is_set():
        tic = time.time()
        with S.lock:
            v   = float(S.speed_mps)
            swd = float(S.steer_wheel_deg)
            now = time.time()
            dt  = clamp(now - S.last, 0.0, 0.2)
            S.last = now

            delta_rad = math.radians(clamp(swd / SR, -85.0, 85.0))
            yaw_dot = (v / max(1e-6, WHEELBASE_M)) * math.tan(delta_rad)
            S.yaw += yaw_dot * dt

            vx = v * math.cos(S.yaw)
            vy = v * math.sin(S.yaw)

            S.x += vx * dt
            S.y += vy * dt

            t_rel = now - S.t0
            traj_log_f.write(f"{t_rel:.6f},{S.x:.6f},{S.y:.6f}\n")
            S.history.append((t_rel, S.x, S.y, S.yaw))

        dt_remain = period - (time.time() - tic)
        if dt_remain > 0:
            time.sleep(dt_remain)

# ================== 相机（10 Hz）& JSON 标注 ==================
def center_crop_16x9(img):
    h, w = img.shape[:2]
    target_ratio = 16/9
    cur_ratio = w / float(h)
    if cur_ratio > target_ratio:
        new_w = int(h * target_ratio)
        x0 = (w - new_w) // 2
        roi = img[:, x0:x0+new_w]
    else:
        new_h = int(w / target_ratio)
        y0 = (h - new_h) // 2
        roi = img[y0:y0+new_h, :]
    out = cv2.resize(roi, (OUT_W, OUT_H), interpolation=cv2.INTER_AREA)
    return out

def open_fixed_device(dev_path):
    cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    try:
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    except Exception:
        pass
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS,          60)
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None
    return cap

class PendingRecord:
    def __init__(self, t0, filename):
        self.t0 = t0
        self.filename = filename  # 相对路径（samples/CAM_FRONT/xxx.jpg）
        self.done = False

def rotate_to_body(dx, dy, yaw0):
    cos0 = math.cos(yaw0)
    sin0 = math.sin(yaw0)
    xb =  cos0*dx + sin0*dy
    yb = -sin0*dx + cos0*dy
    return xb, yb

def make_nuscenes_style_name(now_dt: datetime, sensor="CAM_FRONT"):
    date_str = now_dt.strftime("%Y-%m-%d-%H-%M-%S")
    tz = "+0800"
    unix_ns = int(now_dt.timestamp() * 1e6) * 1000
    return f"n000-{date_str}{tz}__{sensor}__{unix_ns}.jpg"

def camera_thread(stop_evt, label_txt_f):
    os.makedirs(IMAGES_DIR, exist_ok=True)
    os.makedirs(LABELS_DIR, exist_ok=True)
    os.makedirs(IMAGES_DS_DIR, exist_ok=True)
    print(f"[Camera] 图片目录：{IMAGES_DIR}")
    print(f"[Camera] 标注目录：{LABELS_DIR}")

    cap = open_fixed_device(ACTION5_DEVICE)
    if cap is None:
        print(f"[Camera] 无法打开 {ACTION5_DEVICE}，请检查权限/占用状态。")
        return

    period = 1.0 / FRAME_HZ
    pending = []  # list[PendingRecord]

    while not stop_evt.is_set():
        tic = time.time()
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.01)
            continue

        vis = center_crop_16x9(frame)
        vis = cv2.rotate(vis, cv2.ROTATE_180)
        latest_cam_frame.set(vis)

        # 保存 JPEG（nuScenes 风格路径与命名）
        now_dt = datetime.now()
        fname_ds = make_nuscenes_style_name(now_dt, sensor="CAM_FRONT")
        fpath = os.path.join(IMAGES_DS_DIR, fname_ds)
        cv2.imwrite(fpath, vis, [int(cv2.IMWRITE_JPEG_QUALITY), 92])

        # 更新共享中的最新图片相对路径（供 10Hz drive log 使用）
        rel_path_for_json = os.path.join(DATASET_SUBDIR, fname_ds).replace("\\", "/")
        with S.lock:
            t_cap = time.time() - S.t0
            S.latest_img_relpath = rel_path_for_json
        pending.append(PendingRecord(t_cap, rel_path_for_json))

        # 到点写“未来3秒”的 JSON 标注
        with S.lock:
            hist_copy = list(S.history)
        now_abs = time.time()
        for rec in pending:
            if rec.done:
                continue
            if (now_abs - (S.t0 + rec.t0)) >= 3.0:
                x0, y0, yaw0 = interpolate_txyyaw(hist_copy, rec.t0)
                future_ts = [rec.t0 + k*0.5 for k in range(1, 7)]  # 0.5..3.0
                rel_pts = []
                for tf in future_ts:
                    xf, yf, _ = interpolate_txyyaw(hist_copy, tf)
                    dx, dy = (xf - x0), (yf - y0)
                    xb, yb = rotate_to_body(dx, dy, yaw0)
                    rel_pts.append([float(xb), float(yb), 0.0])   # z 恒 0.0

                obj = {
                    "imgs": [rec.filename],
                    "future_poses": rel_pts,
                    "camera_intrinsic": CAMERA_INTRINSIC,
                    "camera_extrinsic": CAMERA_EXTRINSIC,
                    "camera_translation_inv": CAMERA_TRANSLATION_INV,
                    "camera_rotation_matrix_inv": CAMERA_ROTATION_MATRIX_INV
                }
                label_txt_f.write(json.dumps(obj, ensure_ascii=False) + "\n")
                label_txt_f.flush()
                rec.done = True

        if len(pending) > 100:
            pending = [r for r in pending if not r.done]

        dt = time.time() - tic
        if dt < period:
            time.sleep(period - dt)

    cap.release()
    print("[Camera] 线程退出。")

# ================== 10 Hz 车辆+图片旁路日志 ==================
def drive_log_thread(stop_evt, drive_log_f):
    """
    以 10 Hz 写入：
    t_sec, img_relpath, speed_mps, throttle_percent, brake_pressure_raw, steering_wheel_deg
    并在控制台打印：v=?.??? m/s, thr=?.? %, brk=?.?? (raw), steer=?.?? deg
    """
    period = 0.1  # 10 Hz
    while not stop_evt.is_set():
        tic = time.time()
        with S.lock:
            t_rel = time.time() - S.t0
            img  = S.latest_img_relpath or ""
            v    = S.speed_mps if S.speed_mps is not None else None
            tp   = S.throttle_pct
            bp   = S.brake_pressure
            sd   = S.steer_wheel_deg if S.steer_wheel_deg is not None else None

        # 控制台打印（与已测脚本一致）
        print(f"v={'' if v  is None else f'{v:.3f}'} m/s, "
              f"thr={'' if tp is None else f'{tp:.1f}'} %, "
              f"brk={'' if bp is None else f'{bp:.2f}'} (raw), "
              f"steer={'' if sd is None else f'{sd:.2f}'} deg")

        # 落盘 CSV 风格
        line = ",".join([
            f"{t_rel:.3f}",
            img,
            "" if v  is None else f"{v:.3f}",
            "" if tp is None else f"{tp:.1f}",
            "" if bp is None else f"{bp:.2f}",
            "" if sd is None else f"{sd:.2f}",
        ])
        drive_log_f.write(line + "\n")
        drive_log_f.flush()

        dt = time.time() - tic
        if dt < period:
            time.sleep(period - dt)

# ================== 主线程 UI（唯一 imshow/waitKey） ==================
def ui_loop(stop_evt, traj_log_path, labels_path):
    cv2.namedWindow(WIN_TRAJ, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_TRAJ, CANVAS_W, CANVAS_H)
    cv2.namedWindow(WIN_CAM, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_CAM, OUT_W//2, OUT_H//2)

    scale_m_per_pix = 1.0 / 20.0  # 初始 1m->20px

    while not stop_evt.is_set():
        with S.lock:
            traj_copy = list(S.history)
            x_disp, y_disp, yaw_disp = S.x, S.y, S.yaw
            v_disp, swd_disp = S.speed_mps, S.steer_wheel_deg
            brake_p = S.brake_pressure
            thr_pct = S.throttle_pct
            thr_deg = S.throttle_deg

        canvas = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8); canvas[:] = BG
        cx, cy = CANVAS_W//2, CANVAS_H//2

        traj_np = np.array([(tx, xx, yy) for (tx, xx, yy, _) in traj_copy], dtype=np.float32)
        if traj_np.shape[0] >= 2:
            max_range = max(1.0, float(np.max(np.abs(traj_np[:,1]))), float(np.max(np.abs(traj_np[:,2]))))
            px_half = int(0.7 * min(CANVAS_W, CANVAS_H) * 0.5)
            scale_m_per_pix = max(0.01, max_range / max(1, px_half))

        draw_grid(canvas, cx, cy, scale_m_per_pix)
        if traj_np.shape[0] >= 2:
            pts = world_to_canvas(traj_np[:,1:3], scale_m_per_pix, cx, cy)
            if len(pts) >= 2:
                cv2.polylines(canvas, [pts], False, TRJ, 2, cv2.LINE_AA)

        head_len = 1.5
        hx = x_disp + head_len * math.cos(yaw_disp)
        hy = y_disp + head_len * math.sin(yaw_disp)
        head_pts = world_to_canvas(np.array([[x_disp,y_disp],[hx,hy]], dtype=np.float32), scale_m_per_pix, cx, cy)
        cv2.arrowedLine(canvas, tuple(head_pts[0]), tuple(head_pts[1]), CUR, 2, tipLength=0.25)

        cv2.putText(canvas, f"x={x_disp:6.2f} m, y={y_disp:6.2f} m, yaw={math.degrees(yaw_disp):5.1f} deg",
                    (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, ACC, 2, cv2.LINE_AA)
        cv2.putText(canvas, f"v={v_disp:4.2f} m/s, steer_wheel={swd_disp:5.1f} deg",
                    (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, FG, 2, cv2.LINE_AA)

        brake_str = "N/A"
        if brake_p is not None:
            brake_str = f"{brake_p:.1f} (raw)"
        thr_parts = []
        if thr_pct is not None:
            thr_parts.append(f"{thr_pct:.0f}%")
        if thr_deg is not None:
            thr_parts.append(f"{thr_deg:.1f} deg")
        thr_str = " / ".join(thr_parts) if thr_parts else "N/A"
        cv2.putText(canvas, f"Brake Pressure: {brake_str}   Throttle: {thr_str}",
                    (12, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180,230,180), 2, cv2.LINE_AA)

        cv2.putText(canvas, f"L={WHEELBASE_M:.3f} m, SR={SR}",
                    (12, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.6, FG, 1, cv2.LINE_AA)
        cv2.putText(canvas, f"traj log: {os.path.basename(traj_log_path)}  |  labels: {os.path.basename(labels_path)}",
                    (12, 122), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180,180,180), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Press 'q' to quit", (12, CANVAS_H-12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1, cv2.LINE_AA)

        cv2.imshow(WIN_TRAJ, canvas)

        camimg = latest_cam_frame.get()
        if camimg is not None:
            cv2.imshow(WIN_CAM, camimg)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            stop_evt.set()
            break

# ================== 主程序 ==================
def main():
    os.makedirs(IMAGES_DIR, exist_ok=True)
    os.makedirs(LABELS_DIR, exist_ok=True)
    os.makedirs(IMAGES_DS_DIR, exist_ok=True)

    traj_log_path = os.path.join(RUN_DIR, LOG_TRAJ_NAME)
    labels_path   = os.path.join(LABELS_DIR, FRAME_LABEL_NAME)
    drive_log_path= os.path.join(RUN_DIR, DRIVE_LOG_NAME)

    # 打开日志文件
    traj_f = open(traj_log_path, "w", buffering=1)
    traj_f.write("# t_sec,x_m,y_m\n")

    label_f = open(labels_path, "w", buffering=1)
    label_f.write("# JSON Lines: one object per captured frame with 3s future_poses\n")

    # 10 Hz 车辆+图片日志
    drive_f = open(drive_log_path, "w", buffering=1)
    drive_f.write("# t_sec,img_relpath,speed_mps,throttle_percent,brake_pressure_raw,steering_wheel_deg\n")

    stop_evt = threading.Event()

    # 线程：ROS、轨迹、相机、10Hz旁路日志
    t_ros   = threading.Thread(target=ros_spin,        args=(stop_evt,),        daemon=True); t_ros.start()
    t_traj  = threading.Thread(target=traj_thread,     args=(stop_evt, traj_f), daemon=True); t_traj.start()
    t_cam   = threading.Thread(target=camera_thread,   args=(stop_evt, label_f),daemon=True); t_cam.start()
    t_drive = threading.Thread(target=drive_log_thread,args=(stop_evt, drive_f),daemon=True); t_drive.start()

    try:
        ui_loop(stop_evt, traj_log_path, labels_path)
    except KeyboardInterrupt:
        stop_evt.set()
    finally:
        stop_evt.set()
        t_cam.join(timeout=2.0)
        t_traj.join(timeout=2.0)
        t_drive.join(timeout=2.0)
        t_ros.join(timeout=2.0)
        for f in (traj_f, label_f, drive_f):
            try: f.close()
            except: pass
        cv2.destroyAllWindows()
        print(f"[Done] 运行目录：{RUN_DIR}")
        print(f"[Done] 轨迹日志：{traj_log_path}")
        print(f"[Done] 图像目录：{IMAGES_DIR}")
        print(f"[Done] 标注文件：{labels_path}")
        print(f"[Done] 10Hz 车辆+图片日志：{drive_log_path}")

if __name__ == "__main__":
    main()
