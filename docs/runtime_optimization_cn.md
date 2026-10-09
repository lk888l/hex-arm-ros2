# 运行架构、测量与发布验证

本轮继续使用 Python bridge 和独立 Rust 驱动，保持已有增益、运动限值、反馈超时和 watchdog 阈值。
这里记录第二阶段实现；第一阶段迁移记录见 [运行架构与迁移](architecture_refactor_cn.md)。
本轮验证结果及真机边界见 [2026-10-09 验证记录](commissioning_evidence/2026-10-09-runtime-optimization-validation.md)。

## 状态与并发边界

Rust 的 `ArmRuntime` 唯一拥有 `RuntimeData`、电机后端、模式串行锁和关闭 latch。
`runtime/` 中的 session、mode、command、control、gravity、safety、shutdown 模块借用同一所有者；
插值器仍由控制任务私有持有。测试单独放在 `runtime/tests.rs`，没有复制业务状态或新增独立会话锁。

* 缓存身份查询不获取模式锁。真实刷新只允许已初始化、确认 Disabled、没有停机或待失能操作时执行，排队前和取得锁后都检查。
* Meow 初始化得到的身份缓存被后续查询复用，避免重复扫描；缓存不替代实时反馈在线和新鲜度检查。
* 指令最终提交在数据写锁内复核会话。控制输出和重力启动 ramp 更新复核会话及激活周期，旧控制周期不能更新新激活状态。
* 硬件模式操作返回后，提交不得覆盖操作期间新锁存的 Fault；失败执行确认失能回滚，保留故障及失能重试。
* 全部生产协议端点声明成功后才启动处理任务、报告 ready。主进程监护每个具名协议任务；异常退出关闭请求入口并进入统一停机。
* 停机先等待控制和已接受的硬件操作，再确认后端失能，最后取消通信任务。不能提前取消正在执行硬件转换的 RPC。

Python 的 `HexArmBridge` 是唯一会话与生命周期状态所有者。
management/stream 方法借用该所有者，readiness/mapping 提供验证与映射。
命令、关节状态、DriverState、运行门控保持原周期；诊断默认 5 Hz，由独立 worker 格式化，故障和状态变化会唤醒它。
真实入口从 `controllers.yaml` 的 `controller_manager.update_rate` 同时导出命令时长与状态发布周期。

## 性能采集

`HEX_ARM_TRACE_DIR` 设置时，C++、Python、Rust 写入固定容量队列，由后台输出 CSV。
未设置时关闭。队列满时丢弃采集记录并计数，不等待日志写入。
trace 收尾有时间上限，不允许磁盘阻塞无限延长停机。

时间戳使用同机 Linux `CLOCK_MONOTONIC`。ROS stamp 只作关联键，不与 Rust 进程启动后的时间相减。
跨时间命名空间或跨主机的日志不能直接合并作单向时延测量。
trace 开启时使用既有 Protobuf Header 的序号关联源命令与反馈，默认 Header 行为不变；它不新增源命令新鲜度保护。

正向采集覆盖 C++ write/发布交接、Python 接收/Zenoh put、Rust 解码/接受/消费、锁等待、Meow 邮箱与 SocketCAN send。
反向采集覆盖 Rust 状态发布、Python 接收/ROS 发布、C++ 接收/读取。
报告区分没有消费的目标、首次发送、重复发送、发布跳过、采集丢失和完整关联覆盖。
SocketCAN send 返回表示入队完成，不能解释为真实总线发送或电机应用完成。

在已构建并 source 的 ROS 容器中运行隔离 mock 基线：

```bash
python3 scripts/benchmark-transport.py run \
  --output .tmp-transport-baseline --scenario baseline
```

默认每次预热 30 秒、采样 300 秒，重复三次。可选场景为
`trajectory`、`management`、`diagnostic`、`cpu`、`stopped-command`、`delay-recovery`。
短时验证可设置 `--warmup 1 --duration 3 --repetitions 1`。
报告包括 p50/p95/p99/max、每阶段间隔、最大连续空窗及覆盖信息。

该自动 mock fixture 运行 Rust 驱动、Python bridge 和测试目标源，不运行 C++ 硬件插件，也不打开 CAN；
报告会明确标记覆盖不足。完整栈可设置 `HEX_ARM_TRACE_DIR` 后采集，用下述入口分析：

```bash
python3 scripts/benchmark-transport.py analyze /absolute/path/to/trace \
  --output /absolute/path/to/report.json
```

基准期间避免构建和无关负载；对比使用相同配置、轨迹、容差和环境。
本轮没有将 C++ 直连 Zenoh 纳入默认运行链路，也不根据短测宣称性能提升。

## MoveIt 与发布契约

继续严格限制 MoveIt 2.12.4 / rclcpp 28.1.21，启动门控与退出兼容分别维护。
解锁失败撤销已加载能力、停止 executor，并以非零状态完成统一清理。
信号线程使用 RAII 收尾，清理执行能力前停止仍在运行的轨迹。
测试失败注入仅存在于 BUILD_TESTING=ON 的构建中。

升级候选镜像使用相同的 mock 门控、规划、执行、取消和 SIGINT/SIGTERM 回归入口：

```bash
./scripts/verify-moveit-upgrade.sh
```

MoveIt 2.12.4 的直接 ExecuteTrajectory action 取消路径存在上游限制：执行等待占用其回调组，
取消回调无法及时处理。回归分别验证直接执行中的信号退出，以及通常 MoveGroup action 的主动取消。
不能据此宣称直接 ExecuteTrajectory 取消已经修复。

开发运行镜像与正式发布分别使用：

```bash
./scripts/build-runtime-image.sh
./scripts/build-runtime-image.sh --release
```

正式发布要求源码已提交且工作树干净，基础镜像必须匹配 `docker/release-base.json` 中审核过的 digest；
未经审核的 ABI 继续由 CMake 拒绝。开发构建允许本地标签并明确记录身份。
安装检查要求自定义 move_group、退出补丁共享库、驱动及审计脚本完整。

构建 manifest 记录镜像、源码、依赖版本和配置哈希；运行 manifest 记录实际 profile、模型、启动配置、
规划配置、外部参数文件、启动参数和代码产物身份。
生产 Compose 与常用 real-launch 监督入口都会生成 `run-manifest.json`。
开发安装标记为 development-install，不伪装成已发布镜像。

## 验证与保留边界

`scripts/test.sh unit` / `legacy` 包含新增发布、分析器及 supervisor 检查，另有真实进程 mock 回归。
真机验收报告应绑定本次配置；旧速度或扭矩预算的验收记录不能替代新配置复测。

本轮故障注入明确保留了两个后续安全策略问题：

* 指令 watchdog 按接收时间判断新鲜度，短于超时窗口的陈旧来源命令仍可能被接受。
* 仅控制任务冻结而 CAN 后台继续运行时，旧目标与 heartbeat 仍会续发；独立控制进度保护尚未实现。

此外，热替换电机需要单独设计身份变化后的重新初始化和校准失效流程。
控制任务分开读取模式和指令的既有竞争也可能令旧周期给已失能或新激活状态锁存故障；
本轮输出与 ramp 的 owner/epoch 保护防止旧正常目标写入，后续还需给故障提交增加条件保护，避免误停机。
这些问题需要独立规格与真机验收，不能通过改大超时、放宽容差或周期重发旧目标解决。
