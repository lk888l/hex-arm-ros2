# 机械臂子系统集成

本工程既可独立运行，也可由整机工程组合模型、控制管理器和 MoveIt。
默认机械臂名称及操作入口保持兼容。Meow 真机启动恢复为主分支的
J2 → J4 → J3 展开，到达 `[0,-1.350,1.430,-0.300,0,0]` rad 后完成 3 秒静止验收。
入口容差恢复为 J1/J5 0.03 rad、J2/J3/J4 0.01 rad；实际位置还受硬件 profile 约束。
`align_folded:=true` 显式许可准备对齐。ROS 轨迹时长由本次有效动态限值计算。

## 包与依赖

| 包 | 提供的能力 |
|---|---|
| `hex_arm_protocol` | 唯一的 `robot_api.proto`、可由已安装包定位的协议资源 |
| `hex_arm_msgs` | ROS 消息、管理服务、`StopArm` action |
| `hex_arm_controller` | 独立 Rust 驱动、设备标定和保护逻辑 |
| `hex_arm_hardware` | `ros2_control` 插件及非实时通信 |
| `hex_arm_description` | 由固定上游模型生成的可组合 Xacro |
| `hex_arm_bringup` | 实例配置、启动验收、停止操作及兼容命令入口 |
| `hex_arm_moveit_config` | 单臂规划配置，可适配名称及外部整机模型 |
| `hex_arm_moveit_runtime` | 可选的特定版本 MoveIt 启停适配 |
| `hex_arm_integration_examples` | 双实例和共享控制管理器的可运行 mock 示例 |
| `hex_arm_simulation` / `hex_arm_visualization` | 可选的 Gazebo / 图形依赖集合 |

生产脚本安装在 `hex_arm_bringup/lib/hex_arm_bringup` 对应的包安装目录，
源码属于 `src/hex_arm_bringup/scripts`；仓库顶层 `scripts` 保留兼容入口。
消费者使用 ament 包索引定位资源，不依赖工作空间名称或仓库根目录。

在已有 ROS 2 Jazzy 环境中，仅构建硬件启动层可使用：

```bash
colcon build --packages-up-to hex_arm_bringup
source install/setup.bash
```

依赖包括 Cargo/Rust、Protobuf，以及固定版本 Zenoh C/C++ 1.9.0。
SDK 可通过 `python3 scripts/install-zenoh-cpp.py --prefix /opt/hex-arm-zenoh` 安装，
其他位置通过 CMake `HEX_ARM_ZENOH_PREFIX` 指定。固定版本与 SHA256 校验继续生效。
独立 Cargo 源码构建可设置 `HEX_ARM_PROTO_FILE` 指向已安装的
`share/hex_arm_protocol/proto/robot_api.proto`。

## 实例身份

命名分为 ROS 图名称、URDF 名称和驱动通信身份三个部分：

| 参数 | 示例 | 含义 |
|---|---|---|
| `namespace` | `/cell/left` | 该臂的状态、管理服务和 MoveIt 接口 |
| `joint_prefix` | `left_` | 同时用于关节和 link 名称 |
| `hardware_name` | `LeftArmSystem` | controller manager 内唯一的硬件资源名 |
| `controller_manager` | `controller_manager` 或 `/robot/controller_manager` | 相对当前 namespace，或显式绝对路径 |
| `controller_name` | `left_arm_controller` | 轨迹控制器名称，位于管理器的父 namespace |
| `move_group_name` / `planning_group` | `move_group` / `left_arm` | MoveIt 节点名及规划组 |
| profile 中的 `robot_prefix` | `factory/robot1/left` | Zenoh key 前缀，与 ROS namespace 分开 |
| `zenoh_listen` | `tcp/127.0.0.1:7449` | 本臂独立驱动监听地址 |

两臂必须使用各自的硬件 profile、通信身份、CAN 设备及独立监听端口，或显式连接到
统一 Zenoh 路由器。`zenoh_connect` 与 `zenoh_listen` 二选一；空配置保持原端口 7448。
namespace 不会自动修改驱动通信身份。
多臂同时运行时，必须同时设置不同的 `namespace` 和 `joint_prefix`：ROS namespace
不会改变全局 `/tf` 中的 frame 名称，仅隔离话题仍会出现 `base_link` 等 frame 冲突。

驱动和标定文件始终使用 `joint_1` … `joint_6`。ROS 边界通过每关节
`device_joint_name` 显式映射，可使用 `left_joint_1` 等名称而不改变电机坐标。
所有六个设备关节必须唯一且完整。

默认接口继续位于 `/hex_arm/...`。例如 namespace 为 `/cell/left` 时：

```text
/cell/left/hex_arm/internal/state
/cell/left/hex_arm/driver_state
/cell/left/hex_arm/state_timing
/cell/left/hex_arm/status
/cell/left/hex_arm/stop
/cell/left/hex_arm/deactivate_hardware
```

`diagnostics` 继续汇总到 `/diagnostics`，每项名称和硬件标识包含实例身份。
独立管理器的 FJT action 位于 `/cell/left/<controller_name>/follow_joint_trajectory`；
若复用 `/robot/controller_manager`，则位于 `/robot/<controller_name>/follow_joint_trajectory`。

## 可组合模型和状态

整机 URDF 包中包含已安装的宏，然后在已有父 link 上安装机械臂：

```xml
<xacro:include filename="$(find hex_arm_description)/urdf/firefly_y6.macro.xacro"/>
<xacro:firefly_y6_arm prefix="left_" parent="chassis"
  xyz="0 0.3 0.05" rpy="0 0 0" backend="mock"
  hardware_name="LeftArmSystem" ros_namespace="/cell/left"
  tool_link="tcp" tcp_xyz="0 0 0.10"/>
```

`parent` 为空时不创建安装关节。工具视觉、碰撞几何和惯量由整机模型提供；
`tool_link` 创建一个相对 `link_6` 的固定 TCP frame。固定工具示例见
`hex_arm_integration_examples/urdf/dual_arm.urdf.xacro`。

整机 MoveIt 使用完整 URDF/SRDF、外设关节状态及适合实际安装的碰撞配置。
公共关节状态可以汇集不同设备的数据；本包启动/停机保护仍只消费该臂的私有反馈。
示例中的状态转发保留每路原时间戳，不为失联设备补零或伪造新数据。

挂载工具后同步更新 profile 的 `tip_payload` 质量、质心、身份及校验信息；
这些配置不会因为 URDF 中增加工具而自动改变。现有动力学仍使用机械臂自身模型和固定负载。
更改安装姿态时，重力向量应在机械臂 base frame 中表达；移动平台姿态随时间变化时需要
额外的姿态到重力更新策略，本次没有实现 IMU 姿态融合。

## 复用整机启动组件

`real.launch.py` 与 `moveit_real.launch.py` 支持以下组合参数：

| 参数 | 默认 | 外部组件负责时的设置 |
|---|---|---|
| `start_driver` | `true` | `false`，连接已运行的正确驱动 |
| `start_controller_manager` | `true` | `false`，复用 `controller_manager` 指定的管理器 |
| `publish_robot_description` | `true` | `false`，复用整机模型发布者 |
| `robot_description_topic` | 当前实例的模型话题 | 本包创建管理器但使用外部模型发布者时须显式指定 |
| `robot_description_file` | 本包生成模型 | 可提供已展开的整机 URDF |
| `controllers_file` | 从默认配置按实例生成 | 可提供整机 ROS 参数 YAML |
| `controller_update_rate` | 从本包实际加载的 YAML 读取 | 使用外部管理器时须显式填写其实际更新率（Hz） |
| `shutdown_scope` | `launch` | `arm`，单臂故障只停止该臂拥有的进程 |
| `startup_report_file` / `shutdown_report_file` | 每实例独立日志目录 | 可指定整机审计路径，每臂各用一份 |

外部管理器必须预先加载正确的整机模型、控制器类型和参数，并将机械臂硬件置于
**INACTIVE**（整机 CM 的 `hardware_components_initial_state.inactive` 必须包含本臂
`hardware_name`，防止 CM 默认自动激活）。共享 joint state broadcaster 的关节列表应覆盖
全部需要广播的设备。本包启动客户端等待新鲜反馈，复用已有 joint state broadcaster，
配置本臂轨迹控制器后再执行使能和开机动作。已 ACTIVE 的本臂轨迹控制器会被拒绝，
避免重复启动。单臂停止不关闭共享广播器或其他臂的控制器。

模型从 ROS `robot_description` 话题交给 controller manager；仅给节点设置同名参数
不足以代替模型话题。提供整机 URDF 时，本包核对目标硬件名、关节映射、插件和通信配置。
外部 manager 的更新率由 `controller_update_rate` 显式声明，必须与其实际配置一致；
本包生成模型使用其倒数作为 `command_period_sec`，外部模型的相应参数也会核对。
自建管理器则读取实际加载的 `controllers_file`，不能另外声明不同的更新率。

`shutdown_scope:=arm` 会产生实例内故障事件，请求该臂禁能并停止由本次启动创建的进程。
外部管理器、其他 namespace 和整机状态发布器继续由整机监督器管理。
共享管理器自身崩溃仍会影响其所有硬件；上层应根据设备关系决定进一步联动。

本次支持整机组合的入口为 `real`、`mock`、`moveit_real` 和 `moveit_mock`。
`gravity_comp.launch.py` 是独立手动拖动工具，`gz.launch.py` 是单实例独立仿真，
两者仍使用固定控制器和通信配置；`startup.launch.py` 是直接总线的一次性标定工具，
退出会结束其 launch。这三个入口应单独运行，不要直接 Include 到整机启动进程。
`view.launch.py` 保持模型预览隔离，可同时打开多个窗口。

## MoveIt 兼容方式

默认定制 runtime 仍校验 MoveIt **2.12.4** 和 rclcpp **28.1.21**。
其退出兼容补丁继续保留版本检查。需要使用其他上游版本时，可显式关闭该适配包的编译：

```bash
colcon build --cmake-args -DHEX_ARM_BUILD_MOVEIT_RUNTIME=OFF
```

随后启动时传入 `use_custom_runtime:=false`。该模式使用标准 `move_group`：
真机执行时，等本次开机展开及静止验收通过后才创建它，确保启动期间没有执行 action。
需要在开机阶段使用 MoveIt 规划服务的 CiA402 recipe/trial 暂不适用此模式。
关闭版本适配并不代表已验证所有 ROS/MoveIt 版本；目标环境仍须跑回归。

若整机自己启动 MoveIt，设置 `start_move_group:=false`。执行模式还需要提供
`external_readiness_token`，并由外部的门控 runtime 配置相同 token；本包完成启动后
校验执行接口解锁。普通上游 MoveIt 可采用前述延迟启动方式，由整机根据已验证状态创建。

整机模型模式同时提供 `robot_description_file` 和 `robot_semantic_file`，
使用 `joint_states_topic` 指定整机状态流，并关闭本包的 `publish_world_tf`，
由整机负责 world/base 安装关系。单臂安全客户端仍使用私有状态。

## 就绪和停止接口

默认的 `hex_arm_operations` 发布 `hex_arm/status`（`hex_arm_msgs/msg/ArmStatus`）：
STARTING、READY、STOPPING、DISABLED、FAULT。READY 要求本次启动报告、profile 哈希和
实例身份匹配，以及新鲜、健康、ACTIVE 的驱动状态。`execution_ready` 可作为任务调度条件；
它不替代底层生命周期和执行门控。

正常停止使用 `hex_arm/stop`（`hex_arm_msgs/action/StopArm`）：

```bash
ros2 action send_goal /cell/left/hex_arm/stop hex_arm_msgs/action/StopArm '{mode: 0}' --feedback
```

NORMAL=0 沿现有严格规划回 ready、回折或阻尼流程执行，返回进度及确认结果。
IMMEDIATE=1 请求失能，不发起返回轨迹。取消正常停止会中断辅助进程并请求失能。
反馈失效、失败或确认超时会如实返回失败；`disabled_confirmed` 与 `success` 分别表达
失能确认和整个停止流程是否成功。已经开始的设备侧阻尼/RPC 若尚未结束，取消确认可能超时。

同一实例只接受一个停止目标，本机命令行与 action 共用停止操作锁。
驱动仍使用独占控制会话；跨主机应用或多个正常运动客户端需由整机任务调度器统一分配控制权。
正常停止后需要新的启动验收，网络恢复不会自动重新使能。
显式重启 operations 节点时应重新完成本次启动，旧报告不会授权新的正常停止动作。

`set_mode` 在 ROS 适配层仅接受 DISABLED；激活走启动/生命周期入口。
`set_gravity` 单位为 m/s²，坐标系为机械臂模型的 base frame。

## 时间和通信约定

`internal/state` 使用 SensorDataQoS，仅在收到新反馈时发布。
`JointState.header.stamp` 是 C++ 适配层首次收到该快照的 ROS 时间，**不是电机采样时间**。
`state_timing` 提供驱动进程单调时间、接收时间及本机接收后年龄；
`motor_sample_time_available`、`clocks_synchronized` 当前为 false。
收到旧快照不会不断生成新的状态时间戳。

跨主机时不能直接相减两个进程的单调时间。本次没有增加硬件采样时钟同步或
任意网络延迟下的端到端命令截止时间保证。相机/力传感器精确融合需要在实际设备接入时
建立采样时间到公共时基的映射。硬件 watchdog 和反馈超时继续用 steady clock。
真机入口拒绝 `use_sim_time:=true`。

同 CAN 总线外设需要统一分配节点 ID、主机心跳 ID 和带宽，并登记
`bus.auxiliary_node_ids`。未知 CANopen 心跳仍按原保护策略触发故障。
外设由各自驱动控制，机械臂电机标定及定向启停边界保持独立。

## 无硬件验证

```bash
ros2 launch hex_arm_integration_examples isolated_arms.launch.py
ros2 launch hex_arm_integration_examples shared_manager.launch.py
```

两个示例分别使用独立 manager 和整机共享 manager，均为 GenericSystem mock。
测试覆盖左右动作隔离、错误关节目标拒绝、单臂退出后另一臂继续执行、外部整机模型、
TCP/碰撞几何，以及带嵌套 namespace 的 MoveIt 规划执行。

`scripts/test-package-portability.py` 将包复制到不含仓库根 scripts 的临时工作空间，
通过已安装依赖重新构建，验证按包移植。可选 `--check-controller` 还验证独立 Rust 源码
仅使用安装后的协议构建。这些检查与 mock 回归不替代安装新外设后的真机验收。
