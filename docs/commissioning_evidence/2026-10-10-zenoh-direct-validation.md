# 2026-10-10 C++ Zenoh 直连验证

实现及使用方式见 [C++ Zenoh 直连后端](../zenoh_direct_transport_cn.md)。
本记录是开发安装验证，没有提交源码或发布镜像。
初次验证保留旧后端为默认；随后切换为 Zenoh，并按用户要求移除旧桥接。
本文件分阶段保留证据，当前版本状态及追加验证见末节。
原始日志、CSV、逐轮 manifest 与实机采样保留在工作区 `.tmp-direct/`。

## 软件验证

* 受影响的 ROS 包完成回归，最终 `colcon test-result` 为 **389 tests，0 errors，0 failures，0 skipped**。
* 新增协议异常测试 4 项通过：ACTIVE RPC 拒绝后的释放回滚、成功回执但无新 ACTIVE 状态、
  `session_owned=true` 但持有者 ID 不符、获取会话回执丢失。
  这些情况均没有控制命令发送；未确认的获取不猜测会话 ID，不继续使能。
* 通信分析器 9 项、运行包检查 9 项通过。
* 邮箱双读者/单写者压力测试检查 100,000 次写入的完整性；UBSan 下 4 项测试通过。
  TSan 在当前主机/容器报 `unexpected memory mapping`，单进程关闭 ASLR 也被运行环境拒绝，
  因此**没有 TSan 通过结论**。实现用全原子载荷和顺序一致版本校验避免普通内存 seqlock 数据竞争。
* `BUILD_TESTING=OFF` 的独立 Release 安装通过，`libzenohc.so` 随硬件包安装，ELF RUNPATH 为 `$ORIGIN`。
  本次没有重新生成完整 runtime Docker 镜像。

版本组合为 zenoh-cpp / zenoh-c / Rust Zenoh / Python Zenoh **1.9.0**，C++ Protobuf **3.21.12**。
开发容器固定 SDK 路径为 `/opt/hex-arm-zenoh`，避免依赖临时下载目录。

## 故障与恢复

真实 C++ 插件通过 pluginlib 加载，使用隔离 Rust mock，不打开 CAN。
`bridge`、`zenoh` 均通过以下场景：

* 同位置保持、缓慢目标变化、缓存管理查询、诊断订阅、CPU 压力。
* 停止 `write()` 后出现 Rust `0x1003` watchdog 故障。
* 超时后恢复 `write()`，门控仍关闭，没有恢复命令接受。
* 实际 TCP 断开/恢复；驱动进程停顿/恢复；驱动退出并重启。
* 显式重新激活后不再写入：旧邮箱目标没有被接受，watchdog 正常到期。
* 显式重新激活后产生新周期：保持正常，不发生误故障。

断线/重启不自动重获控制权。源时钟倒退会要求重新配置。
驱动重启后的旧会话释放不能伪报确认，独立 Rust 停机回执仍是监督退出的重要依据。

## 三轮完整插件基准

每个后端预热 5 秒、采样 30 秒，重复三轮；100 Hz C++ 周期、同一 Rust mock profile。
测量期间不构建、不施加额外 CPU 压力；两个后端使用相同原子邮箱实现。
以下 p99 区间是三次独立运行的 p99 范围，max 是三次运行中最大的**已记录值**，单位均为 ms。

| 指标 | bridge p99 | zenoh p99 | bridge max | zenoh max |
|---|---:|---:|---:|---:|
| Rust 发布反馈 → 每次 C++ read | 14.646–20.635 | 1.444–2.133 | 20.680 | 11.526 |
| C++ write → Rust 接受 | 1.355–1.625 | 1.115–1.134 | 2.092 | 1.225 |
| C++ read/write 周期代码耗时 | 0.0046–0.0084 | 0.0047–0.0053 | 0.0210 | 0.0571 |
| 周期相对 10 ms 的绝对抖动 | 0.0191–0.2735 | 0.0178–0.0257 | 0.5818 | 0.1147 |

直连的反馈路径减少了等待 Python 定时发布的延迟。100 Hz 各线程的相位会影响数值，
本次 30 秒采样短于脚本默认的 300 秒，不把结果推广为任意负载或严格实时上限。
bridge 第三轮 C++ trace 因 try-lock 争用丢弃 8 条记录，其他五轮为 0；
这不是命令丢弃，但意味着不能把该轮观测 max 当作完整无损记录的上界。
这里统计的是状态发布后的通信年龄，不是电机原始采样年龄。

原始数据：`.tmp-direct/performance-bridge/`、`.tmp-direct/performance-zenoh/`。

## can2 直连实现构建验证

使用既有低速 profile `firefly_y6.meow.can2.expanded-20260930.local.yaml`，没有修改增益或限位。

```text
profile SHA256:
1c5ee58f95bc72fd57d414fbf2087995cc4188d345c766f24d7a7b3ec6d8e6d9
测试插件 SHA256:
bd6afe62052d48c9cc83ca33d52acf2d7945a829dbd32b5fdb7c7c7b4745fd0b
```

只读 SDO 核对六轴身份及折叠位置后，启动真实 `real.launch.py` 的直连观测分支。
新增冷启动门控约 **14.31 秒**完成，确认 INACTIVE 和新鲜反馈；随后通过 controller_manager
硬件生命周期激活，保持当前测量位置 **3 秒**，再显式失能及释放会话。
未加载轨迹控制器，没有执行展开或 MoveIt 运动。

* 299 个独立 ROS 观测帧，最大位置偏差 **0.003212 rad**，最大速度 **0.02865 rad/s**。
* 硬件失能/释放得到确认，launch 退出码 **0**；Rust 停机报告为 **`disabled_confirmed`**。
* 退出后再次只读核对，六轴均为 **mode 0、error 0、heartbeat consumer 0**。
* Jazzy controller_manager 在上下文关闭时仍可能打印已有 pal_statistics 线程日志；
  本次各进程正常退出，未发生 SIGKILL 升级。

该 3 秒保持窗口的同机完整链路 trace：

| 指标 | 关联数 | p99 | max |
|---|---:|---:|---:|
| C++ write → Rust 接受 | 300 | 1.133 ms | 1.157 ms |
| C++ write → CAN send 返回 | 300 | 3.198 ms | 3.203 ms |
| Rust 发布反馈 → C++ read | 299 | 4.519 ms | 4.537 ms |

采集丢弃数为 0，正反向关联均包含实际 C++ 插件。
CAN send 返回表示发送入队，不表示电机已执行。数据位于 `.tmp-direct/can2-direct-final/`，
最后的 SDO 复核位于 `.tmp-direct/can2-final-read.json`。

本轮验证只覆盖折叠位置保持与通信/生命周期，不替代完整 MoveIt 实机运动验收。
此前回折超速边界未在本次修改；Rust 的接收时间 watchdog 与底层 CAN worker 控制进度问题也仍保留。

## 默认切换与 MoveIt 描述竞争修复

按用户要求，bringup、MoveIt real launch、Xacro、C++ 插件和基准入口默认值统一为 `zenoh`。
旧桥接仍可显式选择 `transport_backend:=bridge`。四个受影响包已重新编译安装。

用户报告的 09:35:56 UTC 启动在 35 秒后因 `fresh INACTIVE hardware unavailable before startup deadline`
退出。MoveIt 曾与 bringup 的 robot_state_publisher 同时发布 `/robot_description`，但 MoveIt 的
描述采用 Xacro 默认后端、地址和前缀；ros2_control 只接受最先到达的一份，导致它可能配置成
没有 Python bridge 配合的桥接插件。现由 robot_state_publisher 唯一发布硬件描述，MoveIt
继续发布语义描述。`pal_statistics` 的 invalid context 日志发生在随后的清理阶段。

本轮验证：

* 重跑 hardware、description、bringup、MoveIt config 四个包，共 **294 项测试通过**；
  增加了 MoveIt 不发布竞争硬件描述的回归。工作区累计结果为 390 项、零失败。
* 基准分析器原有 **9 项测试通过**。
* can2 运行完整 `moveit_real.launch.py`，省略 `transport_backend`，设置
  `enable_execution:=false use_rviz:=false`。复用真实启动脚本的只读就绪检查，
  **14.37 秒**获得 INACTIVE 硬件及新鲜六轴反馈。
* 实测描述后端为 `zenoh`，前缀与该 profile 一致；描述发布者只有 robot_state_publisher，
  没有 Python bridge。驱动保持 DISABLED、无会话持有者，未请求使能或运动。
* launch 退出码 0，Rust 停机报告为 `disabled_confirmed`，检查后无遗留控制进程。
  清理阶段仍可见既有 pal_statistics 日志，未将其掩盖。

本轮插件 SHA256 为 `ea48ca4e3f64bfcabbfbead9cf27b51219d7b0c55d058aed57d0aa5c0faf54ce`。
原始结果保留在 `.tmp-direct/default-zenoh-observe/`，回归日志为
`.tmp-direct/default-backend-tests.log`。此处未重新测量上一节性能或重跑真机运动启动。

另一次较早的 09:35:23 UTC 用户启动曾通过直连配置及使能，但启动保持速度峰值
**0.0528 rad/s** 超过既有 **0.02 rad/s** 门槛并失能退出。
这是独立的运动启动问题；本轮未修改速度门槛、电机增益或运动验收条件。

## 移除旧桥接后的验证

旧 `hex_arm_bridge` 包、C++ 的 ROS 命令转发/反馈订阅/桥接服务客户端、launch 双分支、
Xacro 和启动入口的 `transport_backend` 参数均已删除。旧包的 build/install 产物已清理，
实际容器中的 ROS 包索引和 Python import 均找不到旧包。

手引导工具及生成的 Python 协议绑定迁入 `hex_arm_tools`，协议源仍只有 Rust 包的一份。
手引导 launch 使用 INACTIVE 的 C++ 插件发布观测数据，由原有独立工具独占手引导会话。
MoveIt 控制仍只有 C++ 直连管理者取得控制会话。停机工具改用
`/hex_arm/deactivate_hardware`、`/hex_arm/damped_stop`；
冷启动参数改名为 `hardware_startup_timeout_sec`。

* 九个包完成构建安装，五个受影响包的 **296 项测试通过**；当前工作区累计结果
  **319 tests，0 errors，0 failures，0 skipped**。桥接专属测试删除，手引导测试迁移保留。
* 实际 C++ 插件 **5 项协议异常/清理测试通过**，覆盖观察配置反复重建、错误恢复、
  上下文关闭及销毁期间零会话获取、零使能、零命令。
* `./scripts/test.sh protocol` 的 **11 个完整插件/Rust mock 场景全部通过**，覆盖停写、
  延迟恢复、实际 TCP 断开、驱动重启、重新激活后拒绝旧命令及接受新命令。
  运行器只支持直连且始终包含 C++ 插件；归档 CSV 分析功能保留。
* 运行包检查 **10 项**、基准分析器 **9 项**通过；安装检查拒绝旧桥接产物。
* 手引导完整 launch 的 mock 检查：硬件 INACTIVE、没有加载轨迹控制器，
  `/hex_arm/internal/state` 唯一发布者为 `hex_arm_system_io_0`，原有 stop 服务确认失能，
  launch 退出码 0，Rust 报告 `disabled_confirmed`。
* can2 完整 MoveIt **观测启动**约 **14.76 秒**就绪，确认新停机服务存在、旧桥接服务不存在；
  全程未请求使能或运动，驱动 DISABLED、没有会话持有者。launch 退出码 0，Rust 报告
  `disabled_confirmed`，检查后无遗留控制进程。

本轮插件 SHA256：`b9b3b9ce76d20628e0c6ecda9af7da7bcd6a174de009dfb93ab1d47c48a65149`。
原始结果位于 `.tmp-direct/zenoh-only-can2/`、`.tmp-direct/zenoh-only-handguiding/`、
`.tmp-direct/zenoh-only-tests.log` 和 `.tmp-direct/zenoh-only-protocol.log`。
未重新发布 Docker 镜像，也未重跑真机运动启动或修改既有速度门槛。
