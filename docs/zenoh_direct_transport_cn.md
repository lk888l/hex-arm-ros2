# C++ Zenoh 直连后端

`HexArmSystem` 只使用 C++ Zenoh 直连 Rust 驱动。旧 Python bridge、ROS 命令转发路径和
`transport_backend` 切换参数已经移除；手引导客户端和生成的 Python 协议绑定迁入 `hex_arm_tools`。
本次不修改电机增益、重力补偿、运动限位、反馈超时或 Rust watchdog。

## 构建与版本

C++ 使用官方 **zenoh-cpp 1.9.0 + zenoh-c 1.9.0**，对应现有 Rust 和 Python Zenoh 1.9.0。
CMake 要求版本完全匹配，C++ Protobuf 从唯一协议源
`src/hex_arm_controller/proto/robot_api.proto` 生成，不维护第二份协议文件或手工编解码器。
当前 Jazzy 容器的 Protobuf 为 3.21.12；真实 Rust mock 及 can2 链路用于互操作验证。

在开发容器内安装依赖并构建：

```bash
python3 scripts/install-zenoh-cpp.py --prefix /opt/hex-arm-zenoh
./scripts/build.sh
```

安装脚本校验官方发布包的固定 SHA256，支持 Linux x86_64/aarch64，不运行包安装维护脚本。
可用 `-DHEX_ARM_ZENOH_PREFIX=/another/prefix` 指定 SDK。
新开发镜像与 runtime 镜像构建已包含安装步骤。硬件包安装时复制 `libzenohc.so`，
运行库通过 `$ORIGIN` 查找；正式安装不依赖 SDK 临时目录，运行 manifest 记录该库哈希。

接口依据：[官方 C++ 1.9.0 源码](https://github.com/eclipse-zenoh/zenoh-cpp/tree/1.9.0)、
[Session API](https://zenoh-cpp.readthedocs.io/en/1.9.0/session.html)、
[C 1.9.0 发布](https://github.com/eclipse-zenoh/zenoh-c/releases/tag/1.9.0)。

## 启动与控制权

观测模式：

```bash
ros2 launch hex_arm_bringup real.launch.py \
  hardware_profile:=/absolute/path/to/verified.local.yaml \
  activate_hardware:=false use_rviz:=false
```

MoveIt 默认也使用直连，以下命令只观测和规划：

```bash
ros2 launch hex_arm_moveit_config moveit_real.launch.py \
  hardware_profile:=/absolute/path/to/verified.local.yaml \
  enable_execution:=false
```

真机执行将 `enable_execution` 设为 `true`，会使能机械臂并执行既有启动动作。
启动无需指定通信后端；不能再切回旧桥接。

`/robot_description` 由 bringup 的 `robot_state_publisher` 唯一发布，包含本次选择的
后端、Zenoh 地址和 profile 前缀。MoveIt 仅发布语义描述，不再发布采用 Xacro 默认参数的
另一份硬件描述，避免 ros2_control 按消息到达顺序偶发选错后端。

原有显式激活、校准检查、启动顺序和 MoveIt 执行解锁条件继续适用。
直连冷启动先等待硬件完成配置、处于 INACTIVE 且收到新鲜六轴反馈，再加载/激活控制器；
`hardware_startup_timeout_sec` 默认 30 秒，等待预算覆盖六轴 CAN 初始化。
直连观测模式启动 **INACTIVE** 的 ros2_control，以便插件提供观测和工具接口，不获取会话或使能。

直连的 `ZenohTransport` 是客户端会话唯一管理者：

* 配置时检查 API 主版本、六轴名称及顺序映射、FAULT 超时能力。
* 激活要求新鲜关节反馈、DriverState、RobotStatus，驱动 DISABLED 且没有其他会话持有者。
* 获取会话并请求 ACTIVE 后，必须收到请求之后的新 DriverState 和 RobotStatus；
  除 `session_owned` 外，还核对 `session_holder` 等于本管理者的会话 ID。
* 故障、反馈超时或失去持有者均锁存控制门控；网络恢复不会自动使能或重新获取会话。
* 失能/清理通过 Rust `release_session` 的确认结果判定。Rust 仅在 `disable_all` 完成后确认释放。
  RPC 失败不伪报成功，已知会话 ID 保留以便重试；获取会话回执丢失时不猜测 ID 或继续使能。
* 驱动源时钟倒退按驱动重启处理，旧会话不能用于新驱动，需要重新配置。

`/hex_arm/driver_state`、`/hex_arm/internal/state`、`/diagnostics` 由独立非实时发布线程输出，
管理 RPC 不阻塞该线程。工具仍使用 `/hex_arm/discover_motors`、`/hex_arm/clear_fault`、
`/hex_arm/set_gravity`、`/hex_arm/set_mode`、`/hex_arm/deactivate_hardware`、
`/hex_arm/damped_stop`，均经同一管理者操作。
直连不提供独立的 `activate_hardware` 服务，激活只能走硬件生命周期。
`set_mode` 工具只接受 DISABLED；手引导仍使用既有独占会话入口。

## 实时交接与新鲜度

`read()`、`write()` 不调用 ROS 发布、Zenoh、Protobuf、RPC 或日志格式化。
`write()` 复制六轴目标，记录单调时间、递增周期号和激活代次；`read()` 读取最新有效反馈。
Zenoh 回调直接写反馈邮箱，不等待 ROS/诊断发布周期。

`SnapshotMailbox` 的载荷字及序号都是 lock-free 的 64 位原子量，编译时断言平台支持。
单个串行生产者写入；消费者最多尝试两次，冲突时保留上一有效快照，再按其原接收时间检查超时。
所有访问使用顺序一致原子操作，前后相同偶数版本保证完整快照。
不存在普通内存 seqlock 的 C++ 数据竞争，也没有 CAS 自旋、动态分配或等待生产者的路径。
非实时多源状态在管理者内串行化，该 mutex 不由实时接口获取。
可选 trace 使用有界 try-lock 队列，争用时丢测量记录，不等待磁盘。

| 情况 | 行为 |
|---|---|
| 同一位置连续 `write()` | 每周期新序号，允许正常保持 |
| 上游停止 `write()` | 已消费序号不再发送，Rust 原有 100 ms watchdog 仍有效 |
| 生产比通信快 | 邮箱仅保留最新快照，不补发积压周期 |
| 发送前年龄超过 50 ms | 丢弃，该序号不能再次变为可发送 |
| 时间在未来、激活代次不符、生成于激活之前 | 拒绝 |
| 已触发超时/会话失效门控的断线或故障恢复 | 门控保持关闭，显式恢复生命周期后只接收新代次命令 |
| 重新激活但未再 `write()` | 不重发上次目标，watchdog 到期故障 |

`command_max_age_sec` 默认 0.05，只允许调小；命令时长从 controller_manager 更新率导出。
这些保护针对**插件来源、客户端邮箱及转发队列**。Rust watchdog 仍按接收时间工作，
不提供任意跨主机传输延迟下的端到端来源截止时间保证，已发出的 Zenoh/TCP 数据也没有撤销能力。
同机单调时间不能直接用于跨时间命名空间/跨主机的新鲜度判断。
控制任务冻结而底层 CAN worker 仍续发旧目标的既有 Rust 边界，本次没有修改。

## 可复现验证与测量

```bash
./scripts/test.sh unit
python3 scripts/test-direct-transport.py
python3 scripts/benchmark-transport.py run --output .tmp-bench-zenoh
```

默认预热 30 秒、采样 300 秒、三轮。由 `pluginlib` 加载实际 C++ 硬件插件，
在 100 Hz C++ 周期中调用 `read()`/`write()`，连接真实 Rust `--mock` 进程。
这覆盖插件及通信路径；周期负载生成器不等同于 controller_manager/MoveIt 的全部调度负载。
所有自动 fixture 都包含 C++ 插件且不打开 CAN。`./scripts/test.sh protocol` 顺序验证下列 11 个场景。

场景包括 `baseline`、`trajectory`、`management`、`diagnostic`、`cpu`、
`stopped-command`、`delay-recovery`、`disconnect`、`restart`、`reactivate`、`reactivate-stream`。
`disconnect` 通过本机 TCP 代理实际切断连接，保持 Rust 进程运行；
`reactivate` 在重新激活后故意停写，断言没有旧目标被接受；`reactivate-stream` 验证新周期可恢复。

报告包含 write → Rust 接受、CAN 入队（实机才有）、每次 read 的反馈年龄、周期耗时、
周期间隔与相对 10 ms 的绝对抖动，以及 p50/p95/p99/max、未匹配/丢弃记录。
“反馈年龄”在此是 **Rust 状态发布 → C++ read**，不是电机原始采样年龄。
重复读取同一反馈也计入年龄，避免首次消费延迟掩盖停顿。
验证记录中的桥接对照来自移除前的历史测量；分析器仍能读取这些归档 CSV，运行器只启动直连。

本次结果见 [2026-10-10 验证记录](commissioning_evidence/2026-10-10-zenoh-direct-validation.md)。
