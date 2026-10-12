# 1. 安装、简介和示例数据

## 三种描述子

- **STD**：从点云提取平面和角点，组合为三角形；边长等几何量用于候选匹配，顶点对应关系用于几何验证。适合已有关键帧或子地图的系统。
- **BTC**：提取二值局部特征，再把三个特征点组合为三角描述子。导出的 CSV 同时保存三角几何量和三个顶点的占据位串。
- **Scan Context**：把单帧点云划分成 20 个距离环 × 60 个角度扇区，记录每个单元的最大高度；环键可用于候选检索，矩阵列移位用于估计偏航对齐。

三个算法都依赖输入点云的坐标系、预处理和传感器高度。STD/BTC 通常使用关键帧或聚合子地图；对原始单帧提取也可以运行，但描述子数量与稳定性可能不同。Scan Context 默认按 LiDAR 局部坐标计算，源代码中的 `LIDAR_HEIGHT=2.0`、最大半径 80 m；换传感器时应校正这些参数。

## 目录

```text
src/STD/                     STD 核心库和 ROS 2 节点
src/btc_descriptor/          BTC 核心库和 ROS 2 节点
src/scancontext_tro/         Scan Context 核心库、统一 launch、RViz2 配置
examples/data/000095.bin     一帧 KITTI 格式示例点云（float32 x,y,z,intensity）
examples/output/             与该点云对应的三种 CSV 输出
tools/publish_example.py     发布示例点云
tools/plot_descriptors.py    从 CSV 生成对比图
visualization/              已生成的示例图
```

`build/`、`install/`、`log/` 是 colcon 生成目录，不属于算法源码。

## 环境与安装

已在 **Ubuntu 22.04 + ROS 2 Humble** 构建和运行。需要 ROS 2 的 `rclcpp`、`sensor_msgs`、`geometry_msgs`、`visualization_msgs`、`pcl_conversions`、`cv_bridge`、`launch_ros`、`rviz2`、`sensor_msgs_py`，以及 PCL、Eigen、Ceres、OpenCV、TBB、Python NumPy/Matplotlib。Ubuntu 可安装对应的 `ros-humble-*` 包和 `libpcl-dev libeigen3-dev libceres-dev libopencv-dev libtbb-dev python3-numpy python3-matplotlib`。

```bash
cd /home/ros/ros_ws/descriptor_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select std_detector btc_desc scancontext_ros2
source install/setup.bash
```

本机的 `/usr/bin/cmake` 若指向无法运行的 Snap 版本，构建前使用已安装的普通 CMake：

```bash
export PATH=/opt/cmake/3.28.3/cmake-3.28.3-linux-x86_64/bin:$PATH
```

## 运行示例

终端 A，启动三个节点和 RViz2：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scancontext_ros2 visualize_descriptors.launch.py \
  input_topic:=/points_raw output_root:=/tmp/descriptors
```

终端 B，发布随附的一帧点云。默认每 10 个点取 1 个点；这与随附 CSV 的生成方式相同：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/publish_example.py
```

RViz2 中灰色是点云，蓝色是 STD 三角形，橙色是 BTC 三角形，黄色是顶点；Scan Context 另以灰度图像显示。默认固定坐标系为 `lidar`。自己的消息若使用其他 `frame_id`，请在 RViz2 的 Global Options 中同步修改。可用 `max_visualized_descriptors:=100` 限制每种三角形的显示量；**CSV 仍导出全部描述子**。

仅需导出、不打开 RViz2：

```bash
ros2 launch scancontext_ros2 all_descriptors.launch.py \
  input_topic:=/points_raw output_root:=/tmp/descriptors
```

也可分别启动：`std_detector std_extract.launch.py`、`btc_desc btc_extract.launch.py`、`scancontext_ros2 scancontext_extract.launch.py`。各 launch 都支持 `input_topic` 和 `output_dir`；BTC 额外支持 `config_file`。统一 launch 支持 `btc_config`。STD 的 ROS 2 参数在 `src/STD/ros2/config/std_extract.yaml`。

## 随附的真实输出

[`examples/output`](../examples/output) 是对 [`000095.bin`](../examples/data/000095.bin) 发布一次后的结果：

| 文件 | 内容 | 实际大小 |
|---|---|---:|
| `std/std_0.csv` | 表头 + 551 行 STD；每行 22 列 | 552 行 |
| `btc/btc_0.csv` | 表头 + 339 行 BTC；每行 25 列 | 340 行 |
| `scancontext/scancontext_0.csv` | 20 行 × 60 列最大高度矩阵 | 20×60 |

提取结果可能随编译选项、PCL 和并行执行顺序变化；随附文件用于展示数据格式和可视化，不应视为逐位一致的回归基准。

```bash
python3 tools/plot_descriptors.py \
  --std-csv examples/output/std/std_0.csv \
  --btc-csv examples/output/btc/btc_0.csv \
  --scancontext-csv examples/output/scancontext/scancontext_0.csv \
  --points-bin examples/data/000095.bin \
  --output visualization/descriptors.png
```

![三种描述子的示例可视化](../visualization/example_descriptors.png)

示例 `.bin` 来自原 Scan Context 项目随附的 Python 示例数据。算法出处：STD 原项目《STD: Stable Triangle Descriptor for 3D place recognition》；BTC 原项目《BTC: A Binary and Triangle Combined Descriptor for 3D Place Recognition》；Scan Context 原项目及其论文。STD 的许可证文件保留在 `src/STD/LICENCE`，其余来源与再分发条件请以各上游项目为准。
