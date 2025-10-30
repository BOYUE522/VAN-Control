#!/bin/bash

SESSION_NAME=ros_control_session

# Kill any existing session
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start new tmux session
tmux new-session -d -s $SESSION_NAME

# 1) rosbridge_server
tmux rename-window -t $SESSION_NAME:0 'rosbridge'
tmux send-keys -t $SESSION_NAME:0 \
'ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=127.0.0.1' C-m

# 2) ssh tunnel
tmux new-window -t $SESSION_NAME:1 -n 'ssh_tunnel'
tmux send-keys -t $SESSION_NAME:1 \
'ssh -p 15000 -R 9090:localhost:9090 boyuewang@18.191.181.179' C-m

# 3) joystick demo
tmux new-window -t $SESSION_NAME:2 -n 'joystick_demo'
tmux send-keys -t $SESSION_NAME:2 \
'ros2 launch ds_dbw_joystick_demo joystick_demo.launch.xml sys:=true' C-m

# 4) kill joy_demo process
tmux new-window -t $SESSION_NAME:3 -n 'kill_joy_demo'
tmux send-keys -t $SESSION_NAME:3 \
'sleep 5; PID=$(ps aux | grep joy_demo | grep -v grep | awk '"'"'{print $2}'"'"'); \
if [ -n "$PID" ]; then echo "Killing PID $PID"; kill $PID; else echo "No joy_demo found"; fi' C-m

#5) 启动 G920_control_view.py（在 frontcam 目录）
tmux new-window -t $SESSION_NAME:4 -n 'G920_control'
tmux send-keys -t $SESSION_NAME:4 \
'cd ~/ros2_ws/src/boyue/frontcam && source ~/ros2_ws/install/setup.bash && python3 G920_control_view.py' C-m

# 6) 发布 enable 消息；随后自动关闭 cam.py（可选再收尾整场）
tmux new-window -t $SESSION_NAME:5 -n 'enable'
tmux send-keys -t $SESSION_NAME:5 \
'sleep 7; \
ros2 topic pub /vehicle/enable std_msgs/msg/Empty "{}"; \
echo "等待 3 秒后关闭 cam.py..."; \
sleep 3; \
PID=$(ps aux | grep "[c]am.py" | awk '"'"'{print $2}'"'"'); \
if [ -n "$PID" ]; then echo "Killing cam.py PID $PID"; kill $PID; else echo "No cam.py found"; fi' C-m
# 如需跑完自动退出 tmux 会话，请取消下一行注释
# tmux send-keys -t $SESSION_NAME:5 'tmux kill-session -t '"$SESSION_NAME" C-m

# Attach to the session
tmux attach -t $SESSION_NAME
