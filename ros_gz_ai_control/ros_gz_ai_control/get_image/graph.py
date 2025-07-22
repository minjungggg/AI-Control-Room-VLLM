import subprocess
import matplotlib.pyplot as plt
import os
import time
import math
import signal
import sys

# 시각화 스타일 설정
style_map = {
    "duck": {"color": "yellow", "dot_color": "yellow"},
    "surmark_buoy_red": {"color": "red", "dot_color": "red"},
    "surmark_buoy_purple": {"color": "purple", "dot_color": "purple"},
    "surmark_buoy_black": {"color": "black", "dot_color": "black"},
    "wamv_camera": {"color": "gray", "dot_color": "gray"},
}

target_names = list(style_map.keys())
trajectories = {name: {"x": [], "y": []} for name in target_names}

# 거리, 속도 로그
prev_position = None
total_distance = 0.0
last_time = time.time()
speed_log = []

start_time = time.time()

# 안전한 종료 처리
def shutdown_handler(sig, frame):
    print("\nInterrupted. Saving plots...")

    # Trajectory plot
    plt.figure(figsize=(6, 5))
    for name, coord in trajectories.items():
        style = style_map[name]
        x_vals = coord["x"]
        y_vals = coord["y"]
        if x_vals:
            plt.plot(x_vals, y_vals, label=name, color=style["color"], linewidth=2)
            plt.scatter(x_vals[-1], y_vals[-1], color=style["dot_color"], s=100, zorder=5)

    plt.title("2D Trajectory")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid(True)
    os.makedirs(os.path.expanduser("~/saved_images"), exist_ok=True)
    plt.savefig(os.path.expanduser("~/saved_images/trajectory_plot.png"), dpi=300)
    print("[✓] Trajectory saved as 'trajectory_plot.png'")

    # Speed-time plot
    if speed_log:
        times, speeds = zip(*speed_log)
        plt.figure(figsize=(6, 4))
        plt.plot(times, speeds, marker='o', color=style_map["wamv_camera"]["color"])
        plt.xlabel("Time (s)")
        plt.ylabel("Speed (m/s)")
        plt.title("Speed over Time (wamv_camera)")
        plt.grid(True)
        plt.savefig(os.path.expanduser("~/saved_images/speed_plot.png"), dpi=300)
        print("[✓] Speed-time plot saved as 'speed_plot.png'")

    print(f"\n[FINAL] Total Distance: {total_distance:.2f} m")
    print(f"[FINAL] Elapsed Time: {time.time() - start_time:.1f} sec")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown_handler)

# Gz topic listen
process = subprocess.Popen(
    ["gz", "topic", "-e", "-t", "/world/waves/pose/info"],
    stdout=subprocess.PIPE,
    text=True
)

current_name = None

for line in process.stdout:
    line = line.strip()
    if line.startswith("name:"):
        current_name = line.split('"')[1]
    elif line.startswith("position") and current_name in target_names:
        x_line = next(process.stdout).strip()
        y_line = next(process.stdout).strip()
        x = float(x_line.split(":")[1])
        y = float(y_line.split(":")[1])
        trajectories[current_name]["x"].append(x)
        trajectories[current_name]["y"].append(y)

        if current_name == "wamv_camera":
            now = time.time()
            if prev_position and now - last_time >= 1.0:
                dx = x - prev_position[0]
                dy = y - prev_position[1]
                d = math.sqrt(dx**2 + dy**2)
                if d > 0.01:
                    total_distance += d
                    speed_log.append((int(now - start_time), d))
                last_time = now
            prev_position = (x, y)
