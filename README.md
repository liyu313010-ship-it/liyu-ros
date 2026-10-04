# liyu - ROS 工作空间

基于 ROS catkin 的机器人工作空间，包含目标检测模型与导航相关脚本。

## 内容

- `src/` - ROS 功能包源码
- `best.pt` - YOLO 目标检测训练权重
- `bfs.sh` - 启动脚本
- `build/` `devel/` - catkin_make 构建产物

## 技术栈

- ROS (catkin)
- PyTorch / YOLO

## 构建

```bash
cd ~/liyu
catkin_make
source devel/setup.bash
```
