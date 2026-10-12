# 3. 接入回环优化与重定位

这三个描述子提供地点候选或初始配准。现有 ROS 2 提取节点只输出 CSV 和可视化话题；下面的匹配数据库、位姿图和定位状态机应在自己的 SLAM 节点中接入核心 C++ 库。接口与链接方法见[接口文档](02_API_AND_FORMATS.md)。

## 输入与坐标约定

1. 从里程计取关键帧或局部子地图，完成去畸变、地面/动态物体过滤和必要的降采样。每帧保存原始时间戳、连续整数 ID、点云和里程计位姿。
2. 描述子输入点云应位于各自的 **LiDAR/关键帧局部坐标系**。地图建库与查询使用相同的预处理、传感器外参和算法参数。STD/BTC 尤其适合局部子地图；Scan Context 也可直接使用单帧。
3. 记 `T_A_B` 为把 B 坐标系中的点变到 A 坐标系的刚体变换，即 `p_A = T_A_B p_B`。位姿图节点保存 `T_world_frame`。
4. 为每种算法维护独立的 `descriptor ID → 关键帧 ID` 映射。不要假定三个节点发出的 CSV 序号或内部数据库序号等于 SLAM 的关键帧号。

建议按以下管线处理每个新关键帧：

```text
去畸变点云 + 里程计位姿
        ↓
关键帧/子地图及三种描述子
        ↓
候选检索、排除时间和空间邻近帧
        ↓
几何验证：STD/BTC 顶点与平面；SC 需另做点云配准
        ↓
质量门限、重叠度/残差检查
        ↓
相对位姿约束 → 位姿图优化 → 更新 map→odom
```

三个算法可以各自产生候选，再合并候选 ID 并统一做配准；只有通过几何验证的候选才能成为回环约束。描述子相似度、Scan Context 偏航角或可视化 Marker 本身都不是完整的 6 自由度约束。

## STD 在线回环

每个关键帧严格按 **生成 → 搜索 → 验证 → 入库** 处理。以下代码只展示与描述子相关的调用，云对象的创建和异常处理需由应用补齐：

```cpp
ConfigSetting cfg = /* 与建库一致的配置 */;
STDescManager manager(cfg);

// 每个新关键帧 i：i 与 manager.current_frame_id_ 保持一致。
std::vector<STDesc> desc;
manager.GenerateSTDescs(cloud_i, desc); // 同时追加当前帧平面/角点云

std::pair<int, double> result{-1, 0.0};
std::pair<Eigen::Vector3d, Eigen::Matrix3d> tf;
std::vector<std::pair<STDesc, STDesc>> matched;
manager.SearchLoop(desc, result, tf, matched);

if (result.first >= 0) {
  const int candidate = result.first;
  // tf: p_candidate = tf.second * p_i + tf.first
  manager.PlaneGeomrtricIcp(manager.plane_cloud_vec_.back(),
                            manager.plane_cloud_vec_.at(candidate), tf);
  // 用自己的门限检查 result.second、匹配数和 ICP 残差，再添加回环因子。
}
manager.AddSTDescs(desc); // 调用后 current_frame_id_ 增 1
```

`SearchLoop` 内部使用 `skip_near_num_` 排除最近的帧，默认 50。前期数据库太小时没有候选是正常的。**不要在搜索前调用 `AddSTDescs`**；也不要在搜索之前清除 `plane_cloud_vec_`，因为候选平面云用于内部几何验证。`corner_cloud_vec_` 随生成调用增长，长期运行可在确认后续不再使用角点云时管理其内存，但不要打乱帧索引。

## BTC 在线回环

BTC 的调用顺序相同，但 `frame_id` 由调用者传入：

```cpp
ConfigSetting cfg;
load_config_setting(config_path, cfg);
BtcDescManager manager(cfg);

// 每个新关键帧 i；i 从 0 连续增长，并与平面云下标相同。
std::vector<BTC> desc;
manager.GenerateBtcDescs(cloud_i, i, desc);

std::pair<int, double> result{-1, 0.0};
std::pair<Eigen::Vector3d, Eigen::Matrix3d> tf;
std::vector<std::pair<BTC, BTC>> matched;
if (!desc.empty()) manager.SearchLoop(desc, result, tf, matched);

if (result.first >= 0) {
  const int candidate = result.first;
  // tf: p_candidate = tf.second * p_i + tf.first
  manager.PlaneGeomrtricIcp(manager.plane_cloud_vec_.back(),
                            manager.plane_cloud_vec_.at(candidate), tf);
  // 检查得分、匹配数和配准质量后添加回环因子。
}
manager.AddBtcDescs(desc);
```

若 `desc.empty()`，跳过 `SearchLoop`，因为 BTC 候选选择会读取第一个描述子；仍要为该关键帧维持连续 ID，并按系统策略保存平面云和空描述子。`skip_near_num_` 默认 20。`history_binary_list_` 和 `plane_cloud_vec_` 会按生成顺序增长；入库 ID、`frame_id` 和这两个向量的下标必须一致。输入点云必须非空。上游 BTC 的 `frame_number_` 字段是 16 位无符号整数，长时建图到 65536 帧前需扩展字段或分段建库。

## Scan Context 在线回环

`SCManager` 管理矩阵和环键数据库。每次追加当前帧后调用检测：

```cpp
SCManager manager;
// 每个新关键帧 i，cloud_i 的类型为 pcl::PointCloud<SCPointType>。
manager.makeAndSaveScancontextAndKeys(cloud_i);
auto [candidate, yaw_rad] = manager.detectLoopClosureID();
if (candidate >= 0) {
  // yaw_rad 是矩阵扇区循环对齐的偏航估计，单位为弧度。
  // 用它作为点云配准的偏航初值，并由 ICP/GICP/NDT 等求完整 SE(3)。
}
```

内部默认排除最近 50 帧，只搜索较旧的帧。返回的偏航值来源于**把候选 Scan Context 矩阵移位以对齐当前矩阵**；在特定坐标约定下它可能需要取反或换轴，因此接入配准器前应以已知相对姿态的点云检验符号。它没有平移或完整旋转，也没有几何验证得分；配准与质量门限由上层实现。

## 把匹配结果送入位姿图

STD/BTC 的 `SearchLoop` 返回平移 `t` 与旋转 `R`，把**当前查询帧**中的点变到**历史候选帧**：

```text
p_candidate = R * p_query + t
T_candidate_query = [R, t]
```

`PlaneGeomrtricIcp(source=当前平面云, target=候选平面云, tf)` 也沿用这个方向。若位姿图以 `X_k = T_world_k` 保存节点，回环约束应连接 `(candidate, query)`，其测量值为 `Z_candidate_query = T_candidate_query`；它对应预测值 `X_candidate⁻¹ X_query`。GTSAM 风格伪代码：

```cpp
Pose3 Z_candidate_query = Pose3(R, t);
graph.add(BetweenFactor<Pose3>(candidate_id, query_id,
                                Z_candidate_query, loop_noise));
optimizer.update(graph, values);
```

`loop_noise` 应来自配准误差/几何一致性评估，使用鲁棒核并拒绝低重叠、退化或与里程计冲突过大的结果。优化后由 SLAM 前端或地图模块更新关键帧位姿及 `map→odom`；这些状态不由描述子库维护。

如果输入算法的点云已经被变换到同一个全局坐标系，算法返回的是全局点云间变换 `T_globalCandidate_globalQuery`，不能原样当作局部帧间因子。须按构造点云所用的位姿转换：

```text
T_candidate_query = T_world_candidate⁻¹
                    * T_globalCandidate_globalQuery
                    * T_world_query
```

更容易核对的做法是全程在每个关键帧的局部坐标系提取和配准。

## 固定地图重定位

重定位的地图描述子数据库与在线回环数据库应分开。地图关键帧的 ID、局部点云、`T_world_candidate`、算法参数和预处理方式必须一同保存；重启后按原有顺序重建描述子数据库。仅有提取节点的 CSV 不足以恢复 STD/BTC 内部匹配数据库和平面云。

### STD/BTC 查询

1. 对地图关键帧依次 `Generate*Descs` → `Add*Descs`，保存 ID 到地图位姿的映射。每次 `Generate` 追加的平面云要保留。
2. 固定地图查询时把 `config.skip_near_num_ = 0` 再构造 manager，避免时间排除规则把地图末尾的帧屏蔽；或在原有 manager 上设置 `manager.config_setting_.skip_near_num_ = 0`。
3. 对实时查询云 `Generate*Descs` → `SearchLoop` → 可选 `PlaneGeomrtricIcp`，**不要对查询调用 `Add*Descs`**。查询所生成的临时平面云位于 `.back()`。
4. 查询完成后弹出临时状态：STD 的 `plane_cloud_vec_` 与 `corner_cloud_vec_`；BTC 的 `plane_cloud_vec_` 与 `history_binary_list_`。这样下一次查询仍能与原地图索引对应。STD 的 `current_frame_id_` 保持地图帧数；BTC 每次传入地图帧数作为查询 `frame_id`。
5. 匹配通过验证后，查询位姿是 `T_world_query = T_world_candidate * T_candidate_query`。结合连续里程计和时序一致性决定何时接受重定位，随后更新 `map→odom`。

不要在仍需匹配地图帧时清除数据库平面云。若地图仅有若干帧，BTC/STD 的候选数量、最小匹配数和分数门限需要按实际场景标定。

### Scan Context 查询

`detectLoopClosureID()` 固定排除最近 50 帧，适用于在线回环，**不适合直接查询小型固定地图**。固定地图中保存每帧 `makeScancontext(map_cloud)` 的矩阵和 `T_world_candidate`；实时查询调用 `makeScancontext(query_cloud)`，对所有地图矩阵计算 `distanceBtnScanContext(query_sc, map_sc)`，按距离升序选候选。大地图可先比较 `makeRingkeyFromScancontext` 的环键并做最近邻索引，再对少数候选计算完整距离。函数返回 `(距离, 扇区移位数)`；扇区移位数只是配准偏航初值。对候选地图云执行完整点云配准，得到 `T_candidate_query` 后才计算 `T_world_query`。

## 工程注意事项

- STD 与 BTC 上游头文件含有同名的全局类型和函数。建议各运行在独立 ROS 2 组件/进程，并把候选 ID、得分和变换发布给统一的 SLAM 后端；若要在同一二进制链接，先重构命名空间及符号。
- STD/BTC 的部分候选计数数组按 20000 帧固定分配；更大地图需修改实现或分库检索。
- 算法分数取值随点云密度、运动和场景变化。示例 CSV 只验证提取和格式，不能代替针对目标数据集的召回率、误匹配率及回环因子残差评估。
- 可视化 launch 展示 `/std/descriptors`、`/btc/descriptors`、`/scancontext/image`；接入优化时可并行运行，但这些话题不含已验证的回环约束。
