# TextMap Reproduction

textmap_reproduction这个项目和原有的 `parking_slot_detection` 代码是并行存在的，目的是把论文中的三部分方法拆成结构清晰、便于继续开发和替换的模块，而不是直接修改原来的 `associate.cpp`。原代码仍然保留在：

`D:\sjtu\sjtu-yhy\textmap-loc\textmap\src\parking_slot_detection\src\associate.cpp`

## 1. 项目目标

本项目复现论文中的三部分内容：

1. 单车文本地图构建
2. 基于 LLM 的地图优化与众包融合
3. 基于文本锚点和 VLM 增强的定位

整体思路是：

- 单车端先把多帧观测融合为稀疏文本地图
- 再对单车地图做文本纠错、补全和多车融合
- 最后用融合后的全局文本地图做快速定位

和传统高精点云地图相比，这里保留的是“车位 ID + 车位几何”的轻量语义地图。

## 2. 当前代码结构

目录结构如下：

```text
textmap_reproduction/
├─ CMakeLists.txt
├─ README.md
├─ launch/
│  └─ single_vehicle_mapping.launch
├─ include/
│  ├─ textmap_types.hpp
│  ├─ geometry.hpp
│  ├─ json_io.hpp
│  ├─ single_vehicle_mapper.hpp
│  ├─ llm_map_optimizer.hpp
│  ├─ crowd_fusion.hpp
│  └─ localization.hpp
├─ src/
│  ├─ geometry.cpp
│  ├─ json_io.cpp
│  ├─ single_vehicle_mapper.cpp
│  ├─ llm_map_optimizer.cpp
│  ├─ crowd_fusion.cpp
│  └─ localization.cpp
└─ tools/
   ├─ build_single_vehicle_map.cpp
   ├─ optimize_and_fuse_maps.cpp
   └─ localize_frame.cpp
```

各文件职责如下：

- `textmap_types.hpp`
  定义全项目共用的数据结构，包括 `FrameObservation`、`MapSlot`、`TextMap`、`LocalizationResult` 等。

- `geometry.cpp/.hpp`
  负责二维几何计算，包括：
  车位中心计算、角点变换、刚体变换估计、RANSAC/SVD 中使用的基础几何函数。

- `json_io.cpp/.hpp`
  负责 JSON 读写，兼容原始 `associate.cpp` 导出的格式，并支持新的标准化文本地图格式。

- `single_vehicle_mapper.cpp/.hpp`
  复现论文第 3 章的单车地图构建流程。

- `llm_map_optimizer.cpp/.hpp`
  复现论文第 4 章中的 LLM 前预处理、滑窗校正、后验校验接口和离线规则后备逻辑。

- `crowd_fusion.cpp/.hpp`
  复现第 4 章的多车地图粗对齐、文本锚点融合与车位级因子图优化。

- `localization.cpp/.hpp`
  复现第 5 章的文本锚点定位、文本几何联合优化和异常修复流程。

- `tools/*.cpp`
  是三个命令行入口，方便直接跑单独模块。


## 4. 数据结构设计

### 4.1 核心输入结构

整个项目围绕两个层级的数据：

1. 帧级观测 `FrameObservation`
2. 地图级输出 `TextMap`

帧级观测表示单帧检测结果及车辆位姿，地图级输出表示多帧融合后的车位集合。

### 4.2 兼容的输入 JSON

当前读取模块兼容两种主要输入格式。

#### 格式 A：原 `associate.cpp` 风格

这是你当前已有单车地图更接近的格式：

```json
[
  {
    "ParkingSpot": {
      "x1": 0.0, "y1": 0.0,
      "x2": 1.0, "y2": 0.0,
      "x3": 1.0, "y3": 2.0,
      "x4": 0.0, "y4": 2.0,
      "vacant": 0
    },
    "OCRPoint": {
      "text": "101",
      "confidence": 0.90,
      "x1": 0.2, "y1": 0.1,
      "x2": 0.8, "y2": 0.3
    }
  }
]
```

适用场景：

- 直接读取你原有代码保存下来的单车地图
- 作为后续众包融合和定位的输入

#### 格式 B：多帧观测格式

用于重新跑单车建图时的多帧输入：

```json
{
  "frames": [
    {
      "timestamp": 0.0,
      "pose": {
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0
      },
      "slots": [
        {
          "slot_id": "101",
          "confidence": 0.9,
          "bbox": [[0.2, 0.1], [0.8, 0.3]],
          "corners": [[0, 0], [1, 0], [1, 2], [0, 2]],
          "vacant": 0
        }
      ]
    }
  ]
}
```

适用场景：

- 重新按照论文第 3 章跑单车建图
- 将检测结果按帧组织起来做跨帧融合

### 4.3 单车建图输出格式

当前单车建图默认输出的是标准化文本地图结构，也就是：

```json
{
  "schema": "textmap_reproduction/v1",
  "name": "single_vehicle_text_map",
  "spots": [
    {
      "track_id": 0,
      "slot_id": "101",
      "confidence": 0.92,
      "bbox": [[12.3, 4.5], [13.1, 5.0]],
      "corners": [[10.0, 3.0], [12.0, 3.0], [12.0, 8.0], [10.0, 8.0]],
      "vacant": 0,
      "match_count": 6,
      "frame_count": 12,
      "stable": true,
      "text_votes": {
        "101": 3.54
      }
    }
  ]
}
```

字段含义如下：

- `schema`
  当前输出格式版本号。

- `name`
  地图名称。

- `spots`
  所有融合后的车位。

- `track_id`
  单车建图内部跟踪 ID，用于区分地图管理过程中的车位实体。

- `slot_id`
  车位编号，对应论文中的文本锚点。

- `confidence`
  当前保留的最佳文本编号置信度。

- `bbox`
  文本框两对角点坐标，对应原来的 `OCRPoint.x1,y1,x2,y2`。

- `corners`
  车位四角点坐标，对应原来的 `ParkingSpot.x1...x4,y1...y4`。

- `vacant`
  占用状态。

- `match_count`
  成功关联和融合的次数。

- `frame_count`
  生命周期内累计经历的帧数。

- `stable`
  是否达到稳定状态。

- `text_votes`
  多帧文本投票结果，给 LLM 优化和后续分析使用。

### 4.4 LLM 模块实际使用什么输入

这里需要特别说明一下。

第二部分做 LLM 优化时，文件级输入是上面的完整 `TextMap` JSON，也就是：

- 单车建图先输出标准化 JSON
- `optimize_and_fuse_maps` 再读这些单车地图 JSON
- 对每一张单车地图先调用 `LlmMapOptimizer::refine()`
- 将所有 LLM 优化后的局部地图送入 `CrowdFusion::fuse()`
- 众包融合内部先做 RANSAC 粗对齐，再做车位级 g2o 因子图优化，最后输出全局地图

所以可以理解成：

- 单车模块的输出文件：完整 `TextMap`
- LLM 模块的语义输入源：`TextMap.spots`
- 真正喂给 Prompt 的内容：从 `spots` 中裁剪出的窗口级描述
- 众包融合模块的输入：多张已经经过 LLM 优化的 `TextMap`

当前实现中，`LlmMapOptimizer::buildPrompt()` 已经预留了这种滑窗 Prompt 接口。

## 5. 单车建图部分

### 5.1 论文对应

这部分对应论文第 3 章，尤其是：

- 3.4 多帧车位信息的融合和管理
- 3.4.1 停车位车位号的优化管理
- 3.4.2 跨帧停车位关联与融合策略

### 5.2 当前实现思路

`SingleVehicleMapper` 的处理流程如下：

1. 读取每一帧的车位观测和车辆位姿
2. 将当前帧车位从局部坐标变到世界坐标
3. 将当前帧车位与活动地图中的车位做关联
4. 若关联成功，则融合角点和文本信息
5. 若关联失败，则创建新轨迹
6. 更新生命周期状态，过滤噪声并输出稳定车位

### 5.3 文本优化管理

文本 ID 的选择不是简单取最新值，而是综合以下成本：

- 文本置信度成本：`1 - confidence`
- 几何成本：车辆到文本框中心的距离

当前代码中保留了这个思路，对同一车位的多帧文本进行择优更新。

### 5.4 跨帧关联

跨帧关联目前采用：

- 同名 ID 优先
- 几何中心距离作为验证条件

如果当前观测和已有车位的中心距离小于阈值，或者文本 ID 一致且几何可接受，就认为是同一物理车位。

### 5.5 多帧融合

角点和文本框位置采用动态加权平均融合。设计目标和论文一致：

- 观测早期权重大，便于快速收敛
- 观测后期权重衰减，避免新噪声破坏已稳定地图

### 5.6 生命周期管理

当前维护三类状态量：

- `match_count`
- `frame_count`
- `unseen_count`

根据这些指标判断：

- 一个车位是否只是单帧噪声
- 一个车位是否已经足够稳定
- 一个稳定车位是否可以从动态集合转入最终地图输出

## 6. LLM 优化与众包融合部分

### 6.1 论文对应

对应论文第 4 章，尤其是：

- 4.3 基于大语言模型的地图数据优化
- 4.4 基于文本锚点的众包地图融合

### 6.2 当前实现拆成两步

#### 第一步：单车地图优化

由 `LlmMapOptimizer` 负责。

当前实现包含两层含义：

1. 预留了滑动窗口 Prompt 构造接口
2. 默认提供离线规则后备

LLM 处理前现在已经加入论文中的预处理步骤。流程是：

1. 空间聚类
   按车位中心位置将车位划分为局部车位行或局部车位簇，避免把不同区域、不同排的编号混在一个序列里。

2. 聚类内排序
   在每个局部簇内按空间顺序排序，得到结构化且有序的车位序列 `S_raw`。

3. 滑动窗口切分
   按论文思路使用固定窗口和重叠区。当前默认 `sliding_window = 20`，`window_overlap = 8`。

4. 历史锚点传递
   重叠区在实现中作为相邻窗口共享的上下文，保证编号逻辑在窗口之间连续。

5. 窗口级推理和后验校验
   每个窗口会经过文本归一化、序列补全、几何插值和后验验证，然后再合并回全局序列。

离线规则后备目前做的事情包括：

- OCR 易混字符归一化
  例如 `O->0`、`I/L->1`、`S->5`、`B->8`

- 邻近连续编号补全
  当左右车位编号差为 2 时，尝试补中间编号

- 缺失几何插值
  当局部拓扑合理时，用邻居插值补部分缺失角点

这部分不是云端大模型真实调用，而是为了先把论文方法链路完整搭起来，并为后续接入真实 LLM 留好接口。

#### 第二步：多车地图融合与因子图优化

由 `CrowdFusion` 负责。

流程如下：

1. 用相同 `slot_id` 构建跨地图锚点匹配
2. 用 RANSAC + SVD 估计两张地图之间的刚体变换
3. 用严格几何误差阈值过滤粗匹配异常对
4. 用局部邻接拓扑相似度进一步筛掉错误匹配
5. 使用粗对齐结果初始化所有车位节点
6. 构建车位级全局因子图
7. 使用拓扑约束和文本约束联合优化所有车位节点
8. 将优化后的同名车位融合为全局文本地图

### 6.3 当前因子图优化实现到什么程度


当前实现中：

- 图节点：每张单车地图中的每一个车位观测节点
- 拓扑约束边：同一张单车地图内部相邻车位之间的相对位姿约束
- 文本约束边：不同单车地图中相同 `slot_id` 车位之间的闭环约束
- 边权重：文本闭环边权重大于普通拓扑边
- 鲁棒核：使用 Huber 风格权重抑制异常边
- 求解方式：Gauss-Newton/LM 风格的二维 SE(2) 因子图优化
- 先验约束：第一张单车地图的车位节点使用较强先验固定全局参考系

对应代码位置：

- [crowd_fusion.hpp](D:/sjtu/sjtu-yhy/textmap-loc/textmap/src/textmap_reproduction/include/crowd_fusion.hpp)
- [crowd_fusion.cpp](D:/sjtu/sjtu-yhy/textmap-loc/textmap/src/textmap_reproduction/src/crowd_fusion.cpp)

其中新增的核心逻辑包括：

- `buildPoseGraphEdges`
  根据同名 ID 为多张单车地图建立粗对齐约束，提供全局初值

- `optimizePoseGraph`
  对局部地图位姿进行粗优化，给车位级因子图提供初始化

- `optimizeSlotFactorGraph`
  构建并求解车位级全局因子图

- `buildSlotGraphEdges`
  建立同车拓扑约束边和跨车文本约束边

- `solveSlotGraph`
  使用 g2o 的 `VertexSE2`、`EdgeSE2` 和 Huber 鲁棒核优化所有车位节点

这部分现在已经从手写 Eigen 求解器替换为 g2o 图优化实现。图节点使用 `g2o::VertexSE2`，拓扑约束和跨车文本约束使用 `g2o::EdgeSE2`，并为约束边设置 Huber 鲁棒核。它的建模对象已经从“整张地图”细化为“车位节点”，更贴近论文第 4 章中“融合文本与拓扑约束的全局因子图优化”方案。

换句话说：

- 现在已经有众包融合
- 现在已经有车位级因子图优化
- 当前已经接入 g2o，算法结构按论文方案改成“拓扑边 + 文本边 + 鲁棒优化”

## 7. 定位部分

### 7.1 论文对应

对应论文第 5 章，尤其是：

- 5.2 基于文本锚点定位技术
- 5.2.1 基于车位 ID 的位姿初始化
- 5.2.2 文本-几何联合优化的位姿计算方法
- 5.3 基于 VLM 辅助的定位修复方法

### 7.2 当前定位流程

`TextGeometryLocalizer` 的核心流程如下：

1. 根据当前帧检测到的 `slot_id` 在全局地图中做哈希检索
2. 建立初始文本匹配对
3. 用整车位四角点作为 RANSAC 内点单元求初始位姿
4. 在初始位姿上做文本-几何联合 ICP
5. 若定位异常，则触发修复流程，再重新估计位姿

### 7.3 初始位姿估计

这里和论文一致，不是以“单个点”为最小单位，而是以“整车位四角点”为逻辑实体做匹配。这能减小停车场重复结构下的歧义匹配。

### 7.4 文本-几何联合优化

定位代价由两部分构成：

- 文本约束
  车位 ID 一致时，四角点对应关系固定

- 几何约束
  没有可靠文本时，用最近邻几何匹配补充

动态权重由以下因素决定：

- 有效文本匹配数量
- 有效文本平均置信度

这样可以在文本可靠时更多信任文本，在文本缺失或 OCR 差时自动退回几何约束。

### 7.5 VLM 修复逻辑

当前也分成两层：

1. 论文中的目标流程
   当定位异常时，调用 VLM 对 OCR 结果进行上下文校验和补全

2. 当前代码中的离线后备
   当检测到异常时，用全局地图的近邻车位为缺失 ID 做保守补全，再重新定位

所以现在的代码已经有“异常检测 -> 修复 -> 重定位”的完整控制流，但真实多模态大模型推理还没有接入。

## 8. ROS 单车建图入口

### 8.1 为什么要有 ROS 节点

前面的 `SingleVehicleMapper` 是算法层，负责“给我每帧车位观测和位姿，我来做跨帧融合”。但如果直接处理 rosbag，还缺一层很关键的入口：

- 从 bag 播放出来的图像话题里拿到 AVM 图像
- 调用原有检测服务提取车位角点和 OCR 结果
- 读取 IMU/GPS 生成当前车辆位姿
- 把这些内容组装成 `FrameObservation`
- 再交给 `SingleVehicleMapper`

这一层现在已经补进来了，入口节点是：

`single_vehicle_mapping_node`

对应源码：

- [ros_single_vehicle_mapping_node.cpp](D:/sjtu/sjtu-yhy/textmap-loc/textmap/src/textmap_reproduction/src/ros_single_vehicle_mapping_node.cpp)
- [ros_pose_provider.cpp](D:/sjtu/sjtu-yhy/textmap-loc/textmap/src/textmap_reproduction/src/ros_pose_provider.cpp)

### 8.2 当前 ROS 节点做了什么

`single_vehicle_mapping_node` 当前流程如下：

1. 订阅 `/driver/fisheye/avm/compressed`
2. 订阅 `/driver/fisheye/front/compressed`
3. 使用近似时间同步，把 AVM 和前视图像对齐
4. 从 `/Inertial/gps/fix` 和 `/Inertial/imu/data` 估计当前位姿
5. 将 AVM 图像解码成 OpenCV 图像
6. 调用 `parking_slot_detection/gcn_parking` 服务
7. 将服务输出中的车位角点、OCR 文本框和文本内容做关联
8. 将像素坐标变成车辆坐标，再变到世界坐标
9. 组装成 `FrameObservation`
10. 送入 `SingleVehicleMapper`
11. 周期性把当前单车地图保存成 JSON

### 8.3 和原 `associate.cpp` 的对应关系

这部分是直接参考原 `associate.cpp` 的处理逻辑补的，保留了这些关键设计：

- 仍然使用 `gcn_service` 作为车位和 OCR 的前端检测服务
- 仍然基于车位框中心和 OCR 框中心的距离做初始关联
- 仍然使用原始代码里的像素到 BEV/车辆坐标缩放关系
- 仍然基于姿态把局部观测变换到世界坐标

当前也有一些简化：

- 目前 ROS 入口使用的是 `GPS + IMU` 位姿提供器 `RosPoseProvider`
- 还没有把原 `ekfodom.cpp` 那套 EKF 状态完整迁到新项目
- 前视图像目前主要用于同步时序，单车建图实际调用的是 AVM 图像上的 `gcn_service`

也就是说，这版已经具备“rosbag 输入 -> 图像检测 -> 单车建图输出”的完整入口，但位姿部分还是一个比原工程更轻的版本。

### 8.4 当前默认话题和服务

默认读取：

- `avm_topic`: `/driver/fisheye/avm/compressed`
- `front_topic`: `/driver/fisheye/front/compressed`
- `gps_topic`: `/Inertial/gps/fix`
- `imu_topic`: `/Inertial/imu/data`

默认调用服务：

- `gcn_service`

默认输出参数：

- `output_path`
- `save_every_n_frames`

### 8.5 如何用 rosbag 驱动

典型使用方式是：

1. 先启动 `gcn_service`
2. 再用 `roslaunch` 启动 `single_vehicle_mapping_node`
3. 播放你的 rosbag
4. 节点会边处理边保存当前单车地图

一个典型流程可以理解成：

```bash
roscore
rosrun parking_slot_detection client_test_node
roslaunch textmap_reproduction single_vehicle_mapping.launch output_path:=/tmp/single_vehicle_text_map.json
rosbag play your_data.bag --clock
```

这里第 2 行的 `client_test_node` 只是沿用你原工程的启动习惯，核心要求是保证 `gcn_service` 已经存在。真正的新单车建图节点由 `single_vehicle_mapping.launch` 启动。

如果你的 topic 名和默认值不同，可以在 launch 时覆盖：

```bash
roslaunch textmap_reproduction single_vehicle_mapping.launch \
  avm_topic:=/driver/fisheye/avm/compressed \
  front_topic:=/driver/fisheye/front/compressed \
  gps_topic:=/Inertial/gps/fix \
  imu_topic:=/Inertial/imu/data \
  output_path:=/tmp/single_vehicle_text_map.json \
  save_every_n_frames:=20
```

### 8.6 当前 ROS 节点输出什么

输出仍然是单车文本地图的标准 JSON：

- 文件级输出：`TextMap`
- 地图元素：`spots[]`
- 每个车位包含：`slot_id`、`bbox`、`corners`、`confidence`、`vacant`
- 同时保留：`track_id`、`match_count`、`frame_count`、`stable`、`text_votes`

所以现在 rosbag 处理出来的结果，可以直接作为：

- 第二部分 LLM 优化的输入
- 第三部分定位的先验地图输入

## 9. 三个命令行工具

### 9.1 单车建图

入口：

`tools/build_single_vehicle_map.cpp`

用法：

```bash
./build/build_single_vehicle_map frames.json single_map.json
```

作用：

- 读取多帧观测
- 运行单车建图
- 输出标准化 `TextMap`

### 9.2 LLM 优化与众包融合

入口：

`tools/optimize_and_fuse_maps.cpp`

用法：

```bash
./build/optimize_and_fuse_maps global_map.json car_1_map.json car_2_map.json car_3_map.json
```

作用：

- 逐张读取单车地图
- 对每张单车地图先做 LLM 文本优化和预处理
- 再把优化后的多张地图送入众包融合
- 众包融合中先做 RANSAC 粗对齐
- 再做车位级 g2o 因子图优化
- 输出全局地图

注意：

命令行参数顺序是：

- 第一个参数：输出全局地图
- 后面的参数：所有待融合的单车地图

### 9.3 定位

入口：

`tools/localize_frame.cpp`

用法：

```bash
./build/localize_frame global_map.json current_frame.json pose.json
```

作用：

- 读取全局地图
- 读取当前帧观测
- 输出定位结果 JSON

## 10. 构建方式

本目录现在是一个 `catkin` 包，已经增加了 `package.xml`，并依赖：

- `roscpp`
- `sensor_msgs`
- `std_msgs`
- `cv_bridge`
- `image_transport`
- `message_filters`
- `parking_slot_detection`
- `OpenCV`
- `Eigen3`
- `g2o`

构建方式可以直接在你的 catkin workspace 里执行：

```bash
catkin_make
```

生成的可执行文件包括：

- `build_single_vehicle_map`
- `optimize_and_fuse_maps`
- `localize_frame`
- `single_vehicle_mapping_node`


## 11. 当前实现和论文的对应情况

### 已经复现到代码里的部分

- 单车地图的数据结构重构
- 文本 ID 的多帧择优机制
- 跨帧车位关联
- 动态加权融合
- 生命周期管理
- 基于文本锚点的多车匹配
- RANSAC + SVD 坐标系对齐
- 车位级 g2o 因子图优化
- 拓扑相似性过滤
- 基于车位 ID 的定位初始化
- 文本几何联合优化框架
- 异常检测与修复重试流程

### 当前以“接口/后备逻辑”形式存在的部分

- 真正调用 LLM 的滑窗层次化推理
- 真正调用 VLM 的 OCR 校验与补全

### 这意味着什么

当前项目已经具备“论文三部分完整跑通的程序骨架和核心算法逻辑”，适合：

- 和原工程做解耦
- 整理算法模块边界
- 为后续接入真 LLM/VLM 接口做准备
- 作为论文复现实验的基础版本

但如果目标是“严格一比一还原论文所有实验模块”，后面仍建议继续补：

1. 真实 LLM Prompt 推理接口
2. 真实 VLM 校验接口
3. 根据实验需要继续扩展 g2o 边类型和信息矩阵配置
4. 和你原 ROS 感知节点的直接对接

## 12. 如果要和你现有工程接上

当前最自然的接入方式不是推翻原工程，而是这样：

1. 保留原 `parking_slot_detection` 的检测和 ROS 订阅逻辑
2. 每帧把检测结果整理成 `FrameObservation`
3. 调用 `SingleVehicleMapper::addFrame()` 做单车建图
4. 单车地图输出为标准化 JSON
5. 后处理阶段再运行众包融合与定位模块

这样你现有的感知侧工作不会白费，新项目也能逐步替代原来 `associate.cpp` 里耦合太重的地图管理逻辑。

## 13. 后续建议

如果你准备继续往论文完整复现推进，最推荐的顺序是：

1. 先把单车建图输出改成你最希望的最终格式
2. 再把 `LlmMapOptimizer` 的输入窗口格式固定下来
3. 接着补真实的 LLM/VLM 接口
4. 最后根据实验数据调优 g2o 信息矩阵、鲁棒核和收敛阈值

这样会比一口气把所有部分一起重写稳很多，也更方便逐步验证。
