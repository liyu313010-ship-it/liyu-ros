#!/bin/bash

set -u

SERVICE_WS="${SERVICE_WS:-$HOME/siyuu}"
BASE_WS="${BASE_WS:-$HOME/bobac3_ws}"
MAP_DIRECTORY="${MAP_DIRECTORY:-/home/bobac3/.reinovo/maps}"
MAP_NAME="${MAP_NAME:-D123map}"

# 发放工程外层目录和实际catkin工作空间同名时，自动进入内层bobac3_ws。
if [ ! -f "$BASE_WS/devel/setup.bash" ] && \
   [ -f "$BASE_WS/bobac3_ws/devel/setup.bash" ]; then
    BASE_WS="$BASE_WS/bobac3_ws"
fi

if [ ! -f "$SERVICE_WS/devel/setup.bash" ]; then
    echo "未找到服务机器人工作空间: $SERVICE_WS/devel/setup.bash"
    echo "请先在 $SERVICE_WS 执行 catkin_make，或通过 SERVICE_WS 指定实际路径。"
    exit 1
fi

if [ ! -f "$BASE_WS/devel/setup.bash" ]; then
    echo "未找到导航工作空间: $BASE_WS/devel/setup.bash"
    echo "当前工程通常位于 $HOME/bobac3_ws/bobac3_ws。"
    echo "请先编译该工作空间，或通过 BASE_WS 指定实际路径。"
    exit 1
fi

source /opt/ros/noetic/setup.bash
if [ -f "$BASE_WS/devel/setup.bash" ]; then
    source "$BASE_WS/devel/setup.bash"
fi
source "$SERVICE_WS/devel/setup.bash" --extend
export MAP_DIRECTORY

# 比赛系统必须单实例运行。重复执行 bfs.sh 会让地图、定位和语音节点
# 互相顶掉，因此检测到已有 ROS Master 时直接退出。
if rosnode list >/dev/null 2>&1; then
    echo "检测到已有ROS系统在运行，本次不重复启动。"
    echo "请先关闭上一次bfs.sh打开的所有终端，再重新运行。"
    echo "当前ROS节点："
    rosnode list 2>/dev/null || true
    exit 1
fi

# ROS Master 已退出时，单独调试过的语音进程仍可能占用麦克风，导致新的
# voice_collect_node 无法注册服务。此时这些进程不可能属于健康比赛实例，
# 所以只清理本项目明确列出的语音进程，不影响系统音频服务和其他程序。
STALE_VOICE_PATTERNS=(
    "roslaunch.*robot_audio.*voice_interaction.launch"
    "voice_collect_node"
    "voice_aiui_node"
    "voice_interaction_node"
)
stale_voice_found=false
for process_pattern in "${STALE_VOICE_PATTERNS[@]}"; do
    if pgrep -f "$process_pattern" >/dev/null 2>&1; then
        stale_voice_found=true
        pkill -TERM -f "$process_pattern" >/dev/null 2>&1 || true
    fi
done
if [ "$stale_voice_found" = "true" ]; then
    echo "已停止上一次遗留的语音进程，等待麦克风释放。"
    sleep 2
fi

for process_pattern in "${STALE_VOICE_PATTERNS[@]}"; do
    if pgrep -f "$process_pattern" >/dev/null 2>&1; then
        echo "语音残留进程无法正常停止：$process_pattern"
        echo "请关闭旧语音终端后重新运行 bfs.sh。"
        exit 1
    fi
done

REQUIRED_PACKAGES=(
    berxel_camera bobac3_navigation face_rec ar_pose robot_audio service_robot
)
for package_name in "${REQUIRED_PACKAGES[@]}"; do
    if ! rospack find "$package_name" >/dev/null 2>&1; then
        echo "未找到ROS包: $package_name"
        echo "请检查工作空间是否编译，以及setup.bash是否正确加载。"
        exit 1
    fi
done

# 在打开任何比赛节点前一次性校验所有关键配置。过去如果导航 YAML 或
# launch 文件损坏，脚本仍会继续打开相机、AR 和语音窗口，最终产生一串
# 与根因无关的 TF/服务错误。预检失败时立即停止，只报告最早的配置错误。
if ! python3 - "$BASE_WS" "$SERVICE_WS" <<'PY'
import pathlib
import py_compile
import sys
import xml.etree.ElementTree as ET

try:
    import yaml
except Exception as exc:
    print("启动预检失败：无法导入 python3-yaml：%s" % exc)
    raise SystemExit(1)

base_ws = pathlib.Path(sys.argv[1])
service_ws = pathlib.Path(sys.argv[2])
errors = []

yaml_roots = [
    base_ws / "src" / "bobac3_navigation" / "param",
    service_ws / "src" / "service_robot" / "config",
]
for root in yaml_roots:
    if not root.exists():
        continue
    for path in sorted(root.rglob("*.yaml")):
        try:
            text = path.read_text(encoding="utf-8")
            for marker in ("ROS_INFO(", "rospy.", "text.c_str()", "std::"):
                if marker in text:
                    raise ValueError("发现不属于 YAML 的代码片段：%s" % marker)
            data = yaml.safe_load(text)
            if data is not None and not isinstance(data, dict):
                raise ValueError(
                    "YAML 顶层必须是键值表，实际为 %s" % type(data).__name__
                )
        except Exception as exc:
            errors.append("YAML %s：%s" % (path, exc))

launch_roots = [
    base_ws / "src" / "bobac3_navigation" / "launch",
    base_ws / "src" / "robot_audio" / "launch",
    base_ws / "src" / "ar_pose" / "launch",
    base_ws / "src" / "face_rec" / "launch",
    service_ws / "src" / "service_robot" / "launch",
]
for root in launch_roots:
    if not root.exists():
        continue
    for path in sorted(root.rglob("*.launch")):
        try:
            ET.parse(str(path))
        except Exception as exc:
            errors.append("LAUNCH %s：%s" % (path, exc))

service_main = service_ws / "src" / "service_robot" / "scripts" / "service_main.py"
try:
    py_compile.compile(str(service_main), doraise=True)
except Exception as exc:
    errors.append("PYTHON %s：%s" % (service_main, exc))

if errors:
    print("比赛系统启动预检未通过：")
    for error in errors:
        print("  - " + error)
    raise SystemExit(1)

print("比赛系统启动预检通过：YAML、launch 和 service_main.py 均正常。")
PY
then
    echo "未打开任何比赛节点。请先修复以上第一个错误。"
    exit 1
fi

COMMON_SOURCE="source /opt/ros/noetic/setup.bash; \
if [ -f '$BASE_WS/devel/setup.bash' ]; then source '$BASE_WS/devel/setup.bash'; fi; \
source '$SERVICE_WS/devel/setup.bash' --extend; \
export MAP_DIRECTORY='$MAP_DIRECTORY';"

echo "服务工作空间: $SERVICE_WS"
echo "导航工作空间: $BASE_WS"
echo "比赛地图: $MAP_DIRECTORY/$MAP_NAME.yaml"

gnome-terminal --title="roscore" -- bash -c "
$COMMON_SOURCE
roscore;
exec bash"

sleep 2

gnome-terminal --title="camera" -- bash -c "
$COMMON_SOURCE
roslaunch berxel_camera berxel_camera.launch;
exec bash"

sleep 2

# bobac3_nav_2d.launch 已经包含底盘、雷达、地图服务器、定位、move_base 和虚拟墙。
# 不再额外启动第二个 map_server，避免 /map 话题冲突。
gnome-terminal --title="navigation" -- bash -c "
$COMMON_SOURCE
roslaunch bobac3_navigation bobac3_nav_2d.launch map_file_name:='$MAP_NAME';
exec bash"

sleep 4

gnome-terminal --title="face_rec" -- bash -c "
$COMMON_SOURCE
roslaunch face_rec face_rec_service.launch;
exec bash"

sleep 2

gnome-terminal --title="ar_pose" -- bash -c "
$COMMON_SOURCE
roslaunch ar_pose ar_base.launch;
exec bash"

sleep 2

gnome-terminal --title="voice" -- bash -c "
$COMMON_SOURCE
roslaunch robot_audio voice_interaction.launch;
exec bash"

# 不用固定sleep猜测语音节点是否启动成功。必须同时确认采集服务和
# 科大讯飞TTS服务可用，才启动比赛主控。
VOICE_READY=false
for attempt in $(seq 1 15); do
    if rosservice info /voice_collect >/dev/null 2>&1 && \
       rosservice info /voice_tts >/dev/null 2>&1; then
        VOICE_READY=true
        echo "语音采集和科大讯飞服务已就绪。"
        break
    fi
    echo "等待语音服务就绪... ($attempt/15)"
    sleep 1
done

if [ "$VOICE_READY" != "true" ]; then
    echo "语音系统启动失败，未启动service_robot。"
    echo "请查看voice终端中voice_collect_node最早出现的错误。"
    exit 1
fi

gnome-terminal --title="service_robot" -- bash -c "
$COMMON_SOURCE
roslaunch service_robot national_service.launch;
exec bash"

