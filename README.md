# ROS2 车辆控制与数据采集系统

本项目整合了三个核心模块：
1. **摄像头自动录制**
2. **G920 方向盘控制与双路画面显示**
3. **车辆轨迹与标签生成（nuScenes 风格）**
并包含 **三雷达周视融合节点**（旁路运行，不影响原系统）。

---

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble  
- Python ≥ 3.8  
- 依赖库：
  ```bash
  pip install opencv-python pygame numpy
模块说明
📷 1. 摄像头自动录制
路径：E:\ros2_ws_old\src\boyue\frontcam\capture_action5_auto.py

功能

自动识别系统摄像头并录制画面

自动保存视频文件用于数据采集

运行

bash
Copy code
python3 capture_action5_auto.py
🕹️ 2. G920 方向盘控制与画面预览
路径：E:\ros2_ws_old\src\boyue\frontcam\G920_control_view.py

功能

使用罗技 G920 控制车辆转向、油门、刹车

显示车内/车外双路摄像头视角

支持方向盘 + 键盘混合输入

运行

bash
Copy code
python3 G920_control_view.py
🧭 3. 轨迹与标签生成
路径：E:\ros2_ws_old\src\boyue\pathtracker

功能

订阅车辆话题 /vehicle/steering/report、/vehicle/vehicle_velocity

以 30 Hz 积分里程计、10 Hz 抓取相机画面

生成 nuScenes 风格标签（未来 3 s，每 0.5 s 一帧）

运行

bash
Copy code
python3 hud_final.2.py
输出目录

bash
Copy code
run_时间戳/
├─ images/samples/CAM_FRONT/*.jpg
├─ labels/frame_future_traj.txt
├─ trajectory_log.txt
└─ frame_drive_log.txt
🌐 4. 三雷达周视融合（旁路节点）
雷达话题

/front_lidar/scan、/rl_lidar/scan、/rr_lidar/scan @ 10 Hz

base_footprint→*_lidar TF 可用

包结构

bash
Copy code
car_interfaces/        # 定义 SurroundingInfoInterface.msg
uw_surrounding_info/   # 融合节点，发布 /surrounding_info_data
运行

bash
Copy code
colcon build --packages-select car_interfaces uw_surrounding_info
source install/setup.bash
ros2 launch uw_surrounding_info surrounding_info.launch.py
输出

/surrounding_info_data @ 10 Hz

240 维距离向量（0–30 m 归一化）

200 维路点占位 path_rfu

回退（关闭 SCAN）

bash
Copy code
ros2 lifecycle set /<lidar>/os_driver deactivate
ros2 lifecycle set /<lidar>/os_driver cleanup
ros2 param set /<lidar>/os_driver proc_mask 'PCL|IMU'
ros2 lifecycle set /<lidar>/os_driver configure
ros2 lifecycle set /<lidar>/os_driver activate
🚗 5. 远程控制（可选）
开启 rosbridge

bash
Copy code
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=127.0.0.1
SSH 反向隧道

bash
Copy code
ssh -p 15000 -R 9090:localhost:9090 boyuewang@18.191.181.179
车辆控制

bash
Copy code
ros2 launch ds_dbw_joystick_demo joystick_demo.launch.xml sys:=true
ros2 topic pub /vehicle/enable std_msgs/msg/Empty "{}"
状态确认
三雷达 /scan 话题 ≈ 10 Hz

/surrounding_info_data 正常发布 240 维向量

摄像头与轨迹录制脚本运行正常

目录概览
css
Copy code
frontcam/
├─ capture_action5_auto.py
└─ G920_control_view.py
pathtracker/
└─ main.py
car_interfaces/
uw_surrounding_info/
