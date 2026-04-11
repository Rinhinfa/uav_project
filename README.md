# 多无人机果园协同巡检仿真工作区说明

本文档描述位于本目录下的 **ROS 2 Jazzy（Ubuntu 24.04）** 工作区：面向开题报告中的「任务分配—全局路径—轨迹优化—局部避障—动态调度—评估对比」闭环，当前已支持 **可编译、可一键启动 Gazebo、可视化多机运动、可批跑对比**。

---

## 1. 工作区结构

标准 colcon 布局：

| 路径 | 说明 |
|------|------|
| `src/` | 全部功能包源码 |
| `build/`、`install/`、`log/` | colcon 构建产物（本地生成，勿提交版本库时按需忽略） |

### 1.1 功能包一览

| 包名 | 语言 | 职责摘要 |
|------|------|----------|
| `orchard_msgs` | CMake | **占位包**。当前为规避 Jazzy 下自定义 `rosidl` 的兼容成本，业务通信使用标准消息 + JSON；日后可在此恢复自定义接口。 |
| `orchard_sim` | Python | 果园 SDF 程序化生成、多机状态与里程计模拟、障碍物与 spawn 位姿发布。 |
| `orchard_task_allocation` | Python | 任务分配、事件订阅与重分配触发、场景事件时间轴回放、命令行事件注入。 |
| `orchard_traj_minco` | C++ | 订阅粗航点路径，**关键帧松弛**后输出采样全局轨迹（MINCO 思路的简化实现）；提供服务探活。 |
| `orchard_ego_bridge` | C++ | **EgoPlanner 占位**：按障碍对全局路径做局部偏移；**规划仲裁器**在全局轨迹与局部轨迹间切换。 |
| `orchard_evaluation` | Python | 指标落盘（CSV）、Baseline/Proposed 批跑、汇总报告与可选 PNG 图表。 |
| `orchard_bringup` | CMake + launch | 统一 launch、场景与 UAV 参数 YAML。 |

---

## 2. 系统架构与数据流

### 2.1 节点与职责

推荐启动文件：[`src/orchard_bringup/launch/sim_with_gz.launch.py`](src/orchard_bringup/launch/sim_with_gz.launch.py)。

1. **world_generator**：按行距、株距生成行垄果树模型，写出 SDF（默认 `/tmp/orchard_world.sdf`），可直接被 `gz sim` 加载。
2. **fleet_state_publisher**：发布多机 JSON 状态、各机 `nav_msgs/Odometry`、固定示例障碍物 `geometry_msgs/PoseArray`。
3. **spawn_plan_publisher**：发布多机初始 spawn 位姿（`/sim/spawn_poses`），便于后续接入 spawn 服务。
4. **task_allocator**：根据机群状态做任务分配，输出粗航点 `nav_msgs/Path` 与分配结果 JSON、`mission_id`；默认采用**事件驱动重分配**减少路径反复重置。
5. **dynamic_scheduler**：监听 `/scheduler/event_in`，异步调用重分配服务，在 `/scheduler/events_log` 输出 **含 `response_latency_ms` 的 JSON**。
6. **event_profile_player**：若 `event_profile_file` 非空，按 JSON 内时间戳向 `/scheduler/event_in` 注入事件（与 `scenario_*.yaml` 联动）。
7. **traj_optimizer_node**：`proposed` 下对粗航点做关键帧子采样 + 线段密化；`baseline` 下直通粗航点。
8. **ego_local_planner_stub_node**：`proposed` 下对近障路径点做横向偏移并发布局部路径；`baseline` 下不发布局部路径。
9. **planning_arbitrator_node**：短时跟踪局部路径，否则跟踪全局路径；按 `uav_count` 发布最终 `nav_msgs/Path` 至 `/uav_i/cmd_path`，并发布模式字符串。
10. **parameter_bridge（ros_gz_bridge）**：桥接 `/world/orchard_world/set_pose` 服务。
11. **gz_path_follower**：订阅分配结果 JSON，按 `mission_id` 变化更新各机路径，并通过 set_pose 服务驱动 Gazebo 中 `uav_1..uav_n` 运动。
12. **metrics_aggregator**：周期性将覆盖率、里程、`mission_id`、规划模式、事件响应时延等写入 CSV（含多机总里程与活跃机数量）。
13. **TimerAction**：`run_duration_sec` 到达后整体 Shutdown，便于批跑自动结束。

### 2.2 管线模式：Baseline vs Proposed

| 参数 | Baseline | Proposed |
|------|----------|----------|
| `pipeline_mode`（launch） | `baseline` | `proposed` |
| `task_allocator` / `algorithm_mode` | 近似**最近邻**式分配（按任务 id 顺序弱化） | **行垄启发 +** 距离/行偏置加权 |
| `traj_optimizer_node` / `planner_mode` | 不优化，粗航点直通 | **关键帧松弛 +** 轨迹密化 |
| `ego_local_planner_stub_node` | 不触发局部避障输出 | 障碍附近横向偏移（模拟 EgoPlanner） |

用于论文中的「分离式 / 协同式」对照实验入口一致，仅切换 launch 参数即可。

---

## 3. Topic、Service 约定

### 3.1 主要 Topic

| 名称 | 类型 | 说明 |
|------|------|------|
| `/fleet/states_json` | `std_msgs/String` | 机群状态 JSON，含 `uav_id`、位置、电量、`healthy` 等。 |
| `/uav_i/odom` | `nav_msgs/Odometry` | 各机里程计（仿真桩）。 |
| `/sim/obstacles` | `geometry_msgs/PoseArray` | 静态障碍占位。 |
| `/sim/spawn_poses` | `geometry_msgs/PoseArray` | 多机 spawn 建议位姿。 |
| `/allocation/result_json` | `std_msgs/String` | 分配结果 JSON。 |
| `/allocation/mission_id` | `std_msgs/UInt64` | 单调递增任务版本号。 |
| `/planner/coarse_waypoints` | `nav_msgs/Path` | 粗航点（任务点序列）。 |
| `/planner/global_path` | `nav_msgs/Path` | 轨迹优化后全局参考。 |
| `/planner/local_path` | `nav_msgs/Path` | 局部避障输出。 |
| `/uav_i/cmd_path` | `nav_msgs/Path` | 仲裁后下发轨迹（按 `uav_count` 多机发布）。 |
| `/planner/mode` | `std_msgs/String` | `GLOBAL_TRACK` / `LOCAL_AVOID`。 |
| `/scheduler/event_in` | `std_msgs/String` | 动态事件 JSON。 |
| `/scheduler/events_log` | `std_msgs/String` | 事件处理结果与 **响应时延（ms）**。 |

### 3.2 主要 Service

| 名称 | 类型 | 说明 |
|------|------|------|
| `/scheduler/reallocate_tasks` | `std_srvs/Trigger` | 触发增量重分配（由调度器异步调用）。 |
| `/scheduler/inject_event` | `std_srvs/Trigger` | 注入示例事件（简化演示）。 |
| `/traj/query_cost` | `std_srvs/Trigger` | 轨迹节点探活（可扩展为真实代价查询）。 |

### 3.3 事件 JSON 格式（`/scheduler/event_in`）

调度器与分配器解析字段示例：

- **低电量 / 故障 / 通信丢失**：`{"event_type":"LOW_BATTERY","uav_id":"uav_2"}`（`UAV_FAILURE`、`COMM_LOSS` 同理）
- **新增任务**：`{"event_type":"NEW_TASK","uav_id":"uav_1","task":{"id":9001,"priority":3,"task_type":"scan","deadline_sec":180.0,"location":{"x":6.0,"y":-4.0,"z":3.0}}}`

命令行工具：`ros2 run orchard_task_allocation event_injector --help`。

---

## 4. 场景与批跑实验

### 4.1 场景 YAML

位于 [`src/orchard_bringup/config/`](src/orchard_bringup/config/)：

- `scenario_basic.yaml`：无事件，基础静态任务评估。
- `scenario_typical.yaml`：典型动态事件（如低电、新任务）。
- `scenario_stress.yaml`：更高频或组合事件（压力级）。

`events` 列表中每项可含：

- `t`：触发时间（秒，仿真时间轴由 `event_profile_player` 内部单调时钟近似）。
- `type`：`LOW_BATTERY`、`NEW_TASK`、`UAV_FAILURE`、`COMM_LOSS`。
- `uav_id`、嵌套 `task`（新任务时）等。

### 4.2 批跑器 `batch_runner`

1. 读取场景 YAML，将 `events` 写入 `out_dir/event_profile.json`。
2. 若场景事件时间总长超过单次运行时长，会**按比例压缩**到运行窗口的约 10%–80% 区间，保证短时实验仍能触发事件。
3. 依次以 `baseline`、`proposed` 调用同一 launch，并将指标写入 `baseline_run*.csv` / `proposed_run*.csv`。
4. 控制台打印各模式覆盖率、路径长度、`event_response_ms` 的均值摘要。

示例：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run orchard_evaluation batch_runner \
  --scenario src/orchard_bringup/config/scenario_typical.yaml \
  --repeats 5 \
  --run-duration-sec 120 \
  --out-dir /tmp/orchard_batch_run
```

### 4.3 报告生成 `report_generator`

对已存在的 `baseline_run*.csv` / `proposed_run*.csv` 目录生成汇总表与柱状图（需系统已安装 `matplotlib`）：

```bash
ros2 run orchard_evaluation report_generator \
  --input-dir /tmp/orchard_batch_run \
  --output-dir /tmp/orchard_report
```

输出：`summary.csv`，以及 `coverage_mean.png`、`distance_mean.png`、`event_ms_mean.png`。

---

## 5. 构建与日常运行

### 5.1 依赖

- ROS 2 **Jazzy**（Ubuntu 24.04）
- `colcon`、`python3-yaml`（批跑）
- 可选：`python3-matplotlib`（报告出图）

### 5.2 编译

```bash
cd /home/liaw/毕业设计   # 或你的工作区根目录
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5.3 单次启动（Gazebo 可视化 + 多机运动）

```bash
ros2 launch orchard_bringup sim_with_gz.launch.py \
  pipeline_mode:=proposed \
  uav_count:=3 \
  start_gazebo:=true \
  enable_gz_motion:=true \
  metrics_csv:=/tmp/orchard_metrics.csv \
  run_duration_sec:=360000 \
  event_profile_file:=/path/to/event_profile.json
```

`event_profile_file` 为空字符串时不回放事件。默认分配器为事件驱动，不会每个周期重复重置任务。

### 5.4 定时结束（论文批跑）

```bash
ros2 launch orchard_bringup sim_with_gz.launch.py \
  pipeline_mode:=proposed \
  run_duration_sec:=60 \
  metrics_csv:=/tmp/one_shot.csv
```

定时结束后各 Python 节点可能打印 `KeyboardInterrupt` 堆栈属正常现象，**只要 launch 返回码为 0 且 CSV 已写入即可**。

---

## 6. 指标说明（metrics_aggregator）

周期性追加写入 CSV，列包括：

| 列名 | 含义 |
|------|------|
| `elapsed_sec` | 节点运行经过时间 |
| `pipeline_mode` | `baseline` / `proposed` |
| `mission_id` | 当前分配版本 |
| `distance_uav1` | uav_1 累计路径长度估计 |
| `distance_total` | 全部无人机累计路径长度估计 |
| `active_uavs` | 当前有里程计数据的无人机数量 |
| `coverage_ratio` | 基于栅格离散的近似覆盖率 |
| `planner_mode` | 仲裁器当前模式 |
| `event_response_ms` | 最近一次事件重分配响应时延（来自 `/scheduler/events_log`） |

可与开题报告中的「完成时间、能耗/路径、覆盖率、动态响应、轨迹平滑」逐项对应；**轨迹平滑度**可在后续对 `/uav_1/cmd_path` 或 odom 差分增加 jerk 指标列扩展。

---

## 7. 与开题报告模块的对应关系

| 开题模块 | 本工作区实现 |
|----------|----------------|
| 任务分配器 + 行垄启发 | `task_allocator`，`algorithm_mode=proposed` |
| 分离式基线 | `algorithm_mode=baseline` + 直通轨迹 + 关局部规划 |
| MINCO / 关键帧松弛 | `traj_optimizer_node`（简化 MINCO：关键帧子集 + 密化） |
| EgoPlanner | `ego_local_planner_stub_node`（接口保留，可替换真实 EgoPlanner ROS2 移植） |
| 规划仲裁 | `planning_arbitrator_node` |
| 动态事件与重分配 | `dynamic_scheduler` + `event_profile_player` + `event_injector` |
| 仿真场景 / 果园模型 | `world_generator` + `fleet_state_publisher` |
| 多机参数 | `orchard_bringup/config/uav_params.yaml`（P450 类参考） |
| 实验分级 | `scenario_basic.yaml` / `typical` / `stress` |
| 对比实验与数据 | `batch_runner` + `report_generator` |

---

## 8. 已知限制与扩展方向

1. **Gazebo 控制方式**：当前通过 `SetEntityPose` 做运动学跟踪，优点是简单稳定，缺点是不含真实动力学与飞控约束。若需更真实飞行，可升级为 `cmd_vel`/飞控插件闭环。
2. **orchard_msgs**：目录中可能仍残留未参与编译的 `.msg`/`.srv` 文件；实际运行不依赖其生成代码。
3. **真实 MINCO / EgoPlanner**：现为实现与占位，论文写作时可说明为「分阶段集成」；接口 Topic 已稳定，便于替换。
4. **任务完成判定**：当前任务分配覆盖了全果园树位，但尚未对“单棵树已巡检完成”做状态闭环剔除；可继续补充访问阈值判定与完成后从任务池移除。

---

## 9. 文档与代码索引（快速跳转）

- Launch：[`src/orchard_bringup/launch/sim_with_gz.launch.py`](src/orchard_bringup/launch/sim_with_gz.launch.py)
- 分配与事件：[`src/orchard_task_allocation/orchard_task_allocation/`](src/orchard_task_allocation/orchard_task_allocation/)
- 轨迹：[`src/orchard_traj_minco/src/traj_optimizer_node.cpp`](src/orchard_traj_minco/src/traj_optimizer_node.cpp)
- 避障与仲裁：[`src/orchard_ego_bridge/src/`](src/orchard_ego_bridge/src/)
- 仿真：[`src/orchard_sim/orchard_sim/`](src/orchard_sim/orchard_sim/)
- 评估：[`src/orchard_evaluation/orchard_evaluation/`](src/orchard_evaluation/orchard_evaluation/)

---

*生成说明：本文档与工作区当前实现同步；若你增删 Topic 或改 launch 参数，请同步更新本节对应表格。*
