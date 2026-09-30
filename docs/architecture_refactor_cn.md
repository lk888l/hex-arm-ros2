# Firefly Y6 运行架构与迁移

## 本次已实现的范围

本次是可独立验证的第一阶段结构重构，不是整条控制链路重写。
目标是同机 Docker、六轴 Meow 或 CiA402 / SocketCAN、MoveIt 真机执行。
保持既有增益、重力补偿、反馈超时、限位、插值与电机寄存器操作不变。

- 默认生产入口为 `hex_arm_controller`，只保留 profile、通信、
  mock、离线校验、模型路径和停机回执参数。
- 电机边界移入 `src/hex_arm_controller/src/motor/`；
  核心运行逻辑仍通过 `MotorBackend` 依赖接口，可注入 MockBackend。
- 历史单轴诊断迁入 `src/tools/`。默认生产构建包含 CiA402 后端，但不安装
  `hex_arm_commission`；后者仍需显式启用 `HEX_ARM_BUILD_COMMISSIONING`。
- 生产 launch 使用本机 TCP 直连；当前默认构建也包含 CiA402 所需的
  `legacy` Rust feature，保留完整传输能力。UDP 仍用于已有 mock 发现测试。
- `robot_api.proto` 只保留控制器包的一份；Rust 与 Python 都由它生成，
  不手工修改生成文件。没有改变既有消息字段或协议语义。
- Meow 真机执行自动运行 J2 → J4 → J3；CiA402 真机执行从经过标定的当前姿态
  使能并检查保持误差。生产入口不能用 `startup_ready:=false` 绕过协议启动流程。
  仅观察模式仍可保持全程失能。
- 增加原子 JSON 停机回执与独立 Docker 安装产物。

当前生产链路仍是 MoveIt → JTC / ros2_control → C++ 硬件插件 →
Python 适配进程 → Zenoh → 独立 Rust 驱动 → CAN-FD。
Python 尚未移出高频路径；没有宣称已降低通信延迟或周期抖动。

## 模块与状态所有权

```text
hex_arm_controller/
  src/main.rs              生产进程：初始化、信号、任务退出、最终停机
  src/motor/
    mod.rs                 六轴接口、反馈、身份数据
    meow.rs                新固件 Meow / SocketCAN 电机实现
    mock.rs                无真实输出的测试后端
    legacy.rs              旧 CiA402 / USB 实现
  src/runtime.rs           会话、重力补偿、控制周期、故障协调
  src/protocol.rs          Zenoh / Protobuf 边界
  src/startup_recipe.rs    共享启动配置的类型和校验
  src/tools/               可选历史 commissioning 工具
  config/startup.yaml      Rust / ROS 唯一启动动作配置
  proto/robot_api.proto    两种语言唯一协议定义
```

ros2_control 管理硬件与控制器生命周期；Rust 拥有实际使能、故障、
补偿与最终停机结果；适配层负责请求与状态映射。启动客户端只负责
通过标准 FollowJointTrajectory 顺序执行折叠退出，不重新标定编码器。
`backend` 和 `meow_backend` 模块暂留为旧 Rust 导入路径的薄兼容出口。

电机模块沿用当前已经验证过的 Meow 初始化与停机实现，不直接调用
可能隐含清故障、写参数或使能动作的上游“一键初始化”。
没有把 Rust 驱动嵌入 ROS 进程，也没有改变设备侧 watchdog。

## 启动动作契约

启动姿态、目标、持续时间、静止阈值和入口容差统一在
`src/hex_arm_controller/config/startup.yaml`。
Rust 在构建产物内嵌配置；ROS 从该包安装的配置读取。修改后必须重新构建、
一起部署，不能只修改安装目录中的 YAML。

1. 读取绝对编码器反馈并检查静止、在线和断电重启折叠姿态
   `[0,-1.570,1.570,0,0,0]` rad（J2=−1.570、J3=1.570）。
2. 保留原有的可选 J6 对零准备阶段；这不改变编码器标定。
3. 依次执行 J2→−1.350（8 s）→ J4→−0.300（10 s）→ J3→1.430（6 s），
   到达 `[0,-1.350,1.430,-0.300,0,0]` rad 后保持。
4. 不满足折叠入口、目标越界或动作失败时停止，不自动从任意姿态重新规划。

原 ROS 入口与独立 commissioning 工具的折叠容差不同，本次将两套容差
明确命名在同一配置中，保留已验证行为，未悄悄放宽或收紧。
已在 ready 姿态重启 ROS 不等于一次折叠上电：此时启动姿态检查会失败。
模拟初始姿态、SRDF 命名姿态不是电机启动指令的权威来源。

## 构建与旧命令迁移

开发环境仍可用原有 `scripts/build.sh`，默认包含 Meow 和 CiA402 后端，
但不安装 commissioning 二进制。`HEX_ARM_ENABLE_CIA402=OFF` 可构建仅 Meow 驱动。

```bash
# 容器内，无硬件测试
./scripts/build.sh
./scripts/test.sh unit
./scripts/test.sh protocol

# 只有维护历史功能时才启用
colcon build --packages-select hex_arm_controller \
  --cmake-args -DHEX_ARM_BUILD_COMMISSIONING=ON
./scripts/test.sh legacy
```

旧文档中的 `--discover-only`、`--startup-sequence`、`--commission-axis`、
`--diagnose-axis`、`--recover-heartbeat-lost` 均改为传给
`hex_arm_commission`，原显式运动确认参数保留。
`startup.launch.py` 是独立诊断入口，也需要这个可选构建。
生产自动启动不依赖这个工具，仍通过 ROS 标准轨迹控制器执行。

从 ON 切回 OFF 时，旧安装目录可能保留历史文件；不要将增量开发 install
直接当成生产发布包。生产镜像从干净目录构建并检查不含 commissioning。

## 独立 Docker 部署

`compose.runtime.yaml` 是独立配置，不能与开发用的 compose 文件叠加。
镜像只复制非 symlink 的 `/opt/hex-arm` 安装树，不挂载源代码或 build。
只读挂载经过验证的本机硬件 profile；程序用已安装 URDF 覆盖其中模型路径，
不修改增益、零位、重力、限位、总线身份或 payload 参数。

```bash
# 在宿主机仓库目录；构建本身不会启动机械臂
export HEX_ARM_PROFILE=/absolute/path/to/verified.local.yaml
docker compose -f compose.runtime.yaml build

# 以下命令会使能真机，并自动执行折叠退出
docker compose -f compose.runtime.yaml up -d
docker compose -f compose.runtime.yaml logs -f arm

# SIGINT：配置了阻尼且启动已完成时先回安全位再阻尼；最多给 180 秒，日志卷保留
docker compose -f compose.runtime.yaml down
```

宿主机必须已经配置正确的 SocketCAN 接口；容器不自动修改总线时序。
不用 privileged、USB 设备透传或 NET_ADMIN；保留 NET_RAW，使用 host 网络。
不要同时运行开发驱动和生产驱动，或让其他程序控制同一 CAN 总线。
默认不自动重启，避免故障后重新执行启动动作；默认不运行 RViz。

为保持已验证 MoveIt ABI，构建和运行阶段目前复用 `hex-arm-jazzy:local`
基础镜像。发布流水线应将 `HEX_ARM_BUILD_IMAGE` 固定为审核过的 image digest。
该基础镜像仍含开发工具：本次解决“运行依赖工作区链接”，尚未完成基础镜像体积精简。
后续裁剪运行依赖时必须保证兼容补丁的 ABI 校验继续生效。

## 停机结果与故障

每次生产启动在日志卷创建独立 `run.*` 目录，包含
`driver-shutdown.json`。开发 supervisor 也创建每次运行独立的回执路径。

| 回执 state | 含义 |
| --- | --- |
| `starting` | 当前进程尚未提供最终失能确认，不能当作安全退出 |
| `disabled_confirmed` | MotorBackend::shutdown 成功返回，已完成后端确认 |
| `disable_unconfirmed` | 后端停机失败；error 中记录原因 |

文件还包含 schema_version、pid 和时间。缺失、损坏、PID 不匹配、
进程崩溃或停留在 starting 均不能解释为“已失能”。运行时 Fault 仍由
DriverState 表达，不与停机是否确认混为一个状态。

常规 supervisor 的电机失能确认已改为 JSON，不再凭停机日志文字推断；
日志仍用于独立检测 ROS 子进程崩溃和核对进程退出，旧独立 commissioning
入口暂保留历史日志验证。Docker 的退出码本身也不能替代电机回执。
JSON 文件用于进程间审计，不是经过认证的硬件急停机制。

### 正常退出的回位和阻尼

MoveIt 真机标准入口新增外层退出协调器：只在本次自动启动成功且 profile 明确配置阻尼时，
拦截首次 SIGINT，保持 ROS 存活，让 MoveIt 回 `startup_ready`，随后将控制权交给
Rust 的有界终止操作。Rust 负责卸载、反馈检查及最终失能；桥接此时停止转发轨迹，
本次驱动不再接受重新使能。再次 SIGINT/TERM 和异常退出不发起新运动。

这不是新的高频转发进程；外层协调器只处理退出时序。
最终失能回执与柔和退出结果分别保留，柔和过程失败不能被失能确认掩盖。
生产 Compose 已将停机宽限调为 180 s。参数、适用入口和实机验收边界见
[退出阻尼说明](shutdown_damping_cn.md)。

## 后续重构边界

以下明确未在本次实施，不能当作已完成：

1. 把 Python 适配的高频路径收敛到 C++ SystemInterface 的非实时通信模块。
   可采用小型 Rust Zenoh 客户端 C ABI，仅复用通信，不将电机核心合入 ROS 进程。
   先定义会话消失、激活交接、最新值邮箱、反馈新鲜度与退出契约，再替换；
   不能周期性重发旧目标来掩盖上游停写。
2. 独立定义 ROS servo reference 与旧轨迹 chunk 的区别。本次保留 Rust 的
   限速/限加速度重定时行为；没有偷偷把 duration=0 改为无约束直通。
3. 进一步减少控制循环里的分配、共享锁与非实时工作，同时测量 500 Hz
   周期分布、指令延迟与轨迹误差，不能仅根据代码行数宣称性能提升。
4. 退出 MoveIt 自定义入口与 LD_PRELOAD 补丁，需要可重复的退出回归通过。
   当前继续保留 MoveIt 2.12.4 / rclcpp 28.1.21 版本检查。

## 验证范围

无真实 CAN 设备的隔离容器用于编译和测试。已执行 Rust 默认与 legacy
测试、ROS/MoveIt 测试、Zenoh/ROS 协议烟测、supervisor 信号测试；
新增 SIGINT/SIGTERM 真实进程退出回归，但其电机后端是 MockBackend。
生产 Docker 构建还运行 `scripts/test-runtime-install.py`，检查程序、
模型、共享配置、生成绑定及是否存在逃出安装树的 symlink。

2026-09-14 本次验证结果：

- Rust 默认测试 111 项通过，1 项真实 SocketCAN 测试跳过；
  `legacy` 配置 280 项通过、1 项跳过（包含常规核心测试，并非额外 280 项）。
- 默认构建 `cargo fmt --check`、`cargo clippy --all-targets -- -D warnings` 通过。
- ROS/MoveIt 147 项通过；协议烟测、supervisor shell 检查及 3 项 JSON 回执测试通过。
- 镜像在无工作区挂载、无网络、只读文件系统下可读取共享配置、
  导入生成绑定并加载 URDF；Mock MoveIt 提供规划和轨迹 action，SIGINT 后退出码为 0。

镜像 Mock 退出时仍观察到上游 `pal_statistics` 的 ROS context 关闭日志；
控制器与 MoveIt 子进程均报告正常退出。这不是“所有上游退出问题已消失”的证明，
本次没有删除或放宽现有 MoveIt 兼容补丁。

不包含实机运动验收、断网实机停机、端到端延迟和 500 Hz 抖动测量。
重构没有改动 Kp/Kd、重力参数与运动窗口；后续仍应在相同参数下进行实机对比。
