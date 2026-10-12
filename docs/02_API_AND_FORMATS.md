# 2. C++ 接口、ROS 2 话题与输出格式

## 核心库接口

构建并 `source install/setup.bash` 后，三个包会安装头文件和核心库：

| 包 | 头文件 | CMake 库目标 | 单帧提取 |
|---|---|---|---|
| `std_detector` | `<STDesc.h>` | `std_descriptor_core` | `STDescManager::GenerateSTDescs` |
| `btc_desc` | `<btc.h>` | `btc_descriptor_core` | `BtcDescManager::GenerateBtcDescs` |
| `scancontext_ros2` | `<Scancontext.h>` | `scancontext_core` | `SCManager::makeScancontext` |

下游 ROS 2 包可在 `CMakeLists.txt` 使用 `find_package(std_detector REQUIRED)`、`find_package(btc_desc REQUIRED)` 或 `find_package(scancontext_ros2 REQUIRED)`，并通过 `ament_target_dependencies(你的目标名 std_detector)` 等方式链接对应库。`package.xml` 也须加入对应 `<depend>`。

STD 和 BTC 的上游代码都把 `ConfigSetting`、`VOXEL_LOC` 等类型定义在全局命名空间，并有同名自由函数。**不要在同一个翻译单元同时包含 `<STDesc.h>` 与 `<btc.h>`，也不要直接把两套核心静态库强行链接到同一个二进制。**同时使用两者时，建议分别运行两个进程，由上层通过 ROS 2 消息汇总候选结果；若必须在一个二进制中使用，先完成命名空间与符号重构。

### STD

```cpp
ConfigSetting config;
read_parameters(node, config);  // node: rclcpp::Node&；声明并读取 ROS 2 参数
STDescManager manager(config);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = /* 当前关键帧/子地图 */;
std::vector<STDesc> descriptors;
manager.GenerateSTDescs(cloud, descriptors);
```

`GenerateSTDescs` 同时向 `plane_cloud_vec_` 和 `corner_cloud_vec_` 追加一帧。只提取时，使用完后可清空这两个向量；准备调用 `SearchLoop` 时必须保留数据库帧的平面云。构造 manager 时请使用带 `ConfigSetting` 的构造函数，勿使用未初始化配置的默认构造函数。

### BTC

```cpp
ConfigSetting config;
std::string config_file = "/path/to/config_outdoor.yaml";
load_config_setting(config_file, config);  // 使用 OpenCV FileStorage 格式
BtcDescManager manager(config);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = /* 非空点云 */;
std::vector<BTC> descriptors;
manager.GenerateBtcDescs(cloud, frame_id, descriptors);
const auto &binary_points = manager.history_binary_list_.back();
```

`GenerateBtcDescs` 同时向 `plane_cloud_vec_` 和 `history_binary_list_` 追加一帧；输入必须非空。BTC 的 `frame_id` 由调用者管理，建议从 0 连续编号，并与数据库的平面云索引一致。

### Scan Context

```cpp
SCManager manager;
pcl::PointCloud<SCPointType> cloud = /* 当前点云 */;
Eigen::MatrixXd sc = manager.makeScancontext(cloud);  // 默认 20×60
Eigen::MatrixXd ring_key = manager.makeRingkeyFromScancontext(sc);
Eigen::MatrixXd sector_key = manager.makeSectorkeyFromScancontext(sc);
```

`makeScancontext` 只计算当前矩阵；`makeAndSaveScancontextAndKeys(cloud)` 会把矩阵与键追加到 manager 的内存数据库。默认几何参数定义在 `Scancontext.h`，包括 20 环、60 扇区、80 m 半径和 2.0 m 高度偏移。

## ROS 2 提取节点

三个节点订阅同一类型的输入 `sensor_msgs/msg/PointCloud2`，建议字段为 `x,y,z,intensity`，订阅使用 SensorDataQoS。它们负责**提取、导出、展示**；不维护可供回环优化的在线匹配数据库，也不发布位姿约束。匹配和优化的接入流程见[第三份文档](03_LOOP_AND_RELOCALIZATION.md)。

| 节点 | 文件输出 | 可视化话题 |
|---|---|---|
| `std_extract_node` | `std_<序号>.csv` | `/std/descriptors` (`MarkerArray`) |
| `btc_extract_node` | `btc_<序号>.csv` | `/btc/descriptors` (`MarkerArray`) |
| `scancontext_extract_node` | `scancontext_<序号>.csv` | `/scancontext/image` (`Image`, 600×200 mono8) |

三角 Marker 最多展示 `max_visualized_descriptors` 个，默认 100；CSV 保存全部。Marker 使用输入消息的 `header.frame_id`。Scan Context 的显示图像按该帧最大高度线性映射为灰度，**数值分析应读取原始 CSV**。

## CSV 格式

- **STD**：22 列，`frame_id`、三条边长、三个角度、中心 XYZ、三个顶点 XYZ、三个顶点附加值。表头见 [`std_0.csv`](../examples/output/std/std_0.csv)。
- **BTC**：25 列，`frame_id`、三角形几何量、中心 XYZ、三个顶点 XYZ，以及每个顶点的 `summary` 与 `bits`。`bits` 是由 `occupy_array_` 顺序转换的 `0/1` 字符串；表头见 [`btc_0.csv`](../examples/output/btc/btc_0.csv)。
- **Scan Context**：无表头，20 行 × 60 列浮点数；行是由近及远的距离环，列是角度扇区，值为最大高度。见 [`scancontext_0.csv`](../examples/output/scancontext/scancontext_0.csv)。

一个输入点云产生一个文件，序号从 0 开始，节点重启后重新计数。重复使用同一输出目录会覆盖同名文件；生产系统应按会话设置独立的 `output_root` 或 `output_dir`。CSV 中的三角顶点坐标始终属于**传入点云的坐标系**，不会自动变换到地图系。
