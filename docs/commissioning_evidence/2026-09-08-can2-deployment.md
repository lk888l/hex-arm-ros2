# 2026-09-08：can2 真机部署进度与复测记录

此文件保留上午及暂停时的历史状态。下午恢复后的配置范围、J4 Kp=110 真机结果和
当前启动方式见 [接续部署记录](2026-09-08-can2-kp110-deployment.md)。

用户确认应用本地参考更新、受限 `calibrated=true`、J6 回零范围，并要求默认
按 **J2→J4→J3** 启动。本机无夹爪、无附加负载。

## 暂停时状态：J4 Kp=110 已配置，等待现场测试

操作者已实机验证 J4 Kp=110，并明确要求直接采用。本次仅将
`config/hardware/firefly_y6.meow.can2.local.yaml` 的 J4 `default_kp` 从 80 改为 110，
下次 ROS 驱动启动时使用；没有下发电机命令或进行真机复测。
当前 Kp 为 `[80,80,120,110,80,80]`，Kd 全部 15。零偏、重力补偿、限位和
J2→J4→J3 的启动顺序保持不变。下面的历史 ROS 成功/失败记录均发生在 J4 Kp=80 时。

操作者说明上次 can2 无心跳可能源于线材接触不良等外部原因，恢复通信及进一步
测试留到下次进行；本次未确认该原因，也不将它认定为驱动软件故障。
下次恢复连接后，沿用已批准的 J4 Kp=110，继续验证顺序启动、J4 跟踪及 MoveIt 执行。

## 上次 ROS 真机复测：仍有一项跟踪问题待复测

前面的完整启动、保持和 MoveIt 小步试验成功，但**最新启动协调器版本尚未完成
稳定性验收**。11:08 复测中 J2 成功，J4 在约 4.45 秒时出现 0.02035 rad
路径误差，超过原有 0.02 rad 阈值，FJT 中止并有序停用控制器和硬件。
11:13 独立 SDO 确认六轴 mode=0、error=0、heartbeat consumer=0，见
[该次停机读取](2026-09-08-can2-j4-abort-disabled.json)。没有提高增益或放宽误差阈值。
[失败摘要](2026-09-08-can2-j4-abort-result.json) 对应本地全量 `log/can2-j4-abort-20260908.json`；状态流最大间隔 10.682 ms，
但当时没有电机目标帧记录，暂不能区分驱动插值延迟、传输延迟与电机实际跟踪误差。

11:18 后续被动记录尚未开始，can2 的 USB 设备已重新枚举，接口成为未配置的 DOWN。
驱动在使能前拒绝启动。随后已恢复 1M/4M CAN-FD 参数及 UP 状态，但没有收到心跳，
SDO 身份读取也超时，已请操作者检查电源/接线。所有真机 owner 已退出。
恢复连接后需先只读核对身份、位置，再同步记录 CAN 完成 J2→J4→J3 和保持复测。
不能把下面历史成功记录等同于最终版本的重复启动已经通过。

## 已完成的真机验证

六轴身份与 Meow 读数正常。基于本次禁用状态的稳定读数记录 J1～J5 折叠参考，
J6 保留原零偏；见 [参考及授权记录](2026-09-08-can2-reference.json)。

通过真实 Rust → CAN、ROS bridge、ros2_control、MoveIt 链路完成：

1. J6 从 −0.219 rad 平滑回到 0，用时 10 秒。
2. J2 到 −1.350，用时 8 秒。
3. J4 到 −0.300，用时 10 秒。
4. J3 到 3.000，用时 6 秒。
5. 保持 10 秒，1000 个样本中最大关节误差 0.002628 rad，最大速度 0.001311 rad/s。
6. MoveIt 严格碰撞检查 21 个规划点后，执行 J2 +0.008 rad 成功。
7. 客户端有序停用控制器、硬件进入 INACTIVE；supervisor 确认完整失能、解除心跳及退出。

保持末尾实测位置约为 `[0,-1.351798,3.000960,-0.297372,0.000058,-0.000623]` rad。
当前 ROS 路径/目标误差仍为 0.02/0.005 rad，没有通过放宽容差取得上述结果。
状态流最大间隔 12.665 ms，指令流最大间隔 10.518 ms；100 ms 看门狗保留。
[真实结果](2026-09-08-can2-real-result.json)、[独立失能读取](2026-09-08-can2-first-disabled.json)。
完整本地记录：`log/can2-real-commission-20260908.json`。

## 较早版本的自动入口成功及保持交接

10:48 的 `real-launch moveit ... enable_execution:=true` 真机试验成功：
自动执行 J2→J4→J3，保持 10 秒最大误差 0.002657 rad，RViz 正常打开并连接规划服务。
启动客户端成功退出后，ros2_control 继续持有目标位置。
独立 20 秒监测取得 2001 个位置样本、2000 个驱动状态样本，全部 ACTIVE、无故障；
最大误差 0.002523 rad，最大命令年龄 3.991 ms、反馈年龄 20.557 ms。

随后先停用控制器及硬件，再 Ctrl-C supervisor，确认 `VERIFIED clean controller exit`。
10:54 独立 SDO 再次确认六轴 mode=0、error=0、heartbeat consumer=0；无残留 owner。

- [正式自动启动结果](2026-09-08-can2-auto-startup-result.json)
- [交接后的保持结果](2026-09-08-can2-handoff-result.json)
- [最终失能状态](2026-09-08-can2-final-disabled.json)
- 最终本地参考由 [失能后复核](2026-09-08-can2-auto-reference.json) 及
  [端点微小修正](2026-09-08-can2-endpoint-reference.json) 记录；11:13 折叠端点的
  [J3 修正记录](2026-09-08-can2-final-endpoint-reference.json) 另外修正了 0.000112 rad 的摆放差异。
  没有自动把任意姿态重设为零。

本次自动入口调试还处理了两项部署问题：安装脚本的可执行属性，以及内层 bringup
的 `use_rviz=false` 覆盖外层 RViz 请求。外层现在保存自己的开关值，避免窗口被静默跳过。

第一次自动入口在发送轨迹前触发一次 100 ms 命令看门狗；驱动正常失能，独立检查见
[该次禁用状态](2026-09-08-can2-auto-watchdog-disabled.json)。当时客户端只有一个状态样本，
不足以将中断唯一归因于某个线程。已移除默认的可靠指令诊断订阅；可选
`--record-commands` 使用 best-effort，不让记录端增加可靠读者的阻塞风险。
Fast DDS 官方文档说明 [可靠写入可能阻塞](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/dds_layer/publisher/dataWriter/publishingData.html)。
改动后正式自动入口及上述保持复测成功，没有放宽看门狗或伪造命令续期。
更长时间及反复冷启动的稳定性仍需后续验收。

## 启动协调器的后续修正

10:57 的纯模拟复测同样出现使能后的命令看门狗，说明之前的诊断订阅调整不能
证明已解决根因。旧入口在硬件使能后另起 spawner 加载控制器，首次 DDS 发现和
插件配置发生在电机活动期间。新入口由单一客户端预先完成所有控制器加载、配置、
动作服务匹配和通信预热，随后使能硬件、成组激活控制器并执行启动序列。
配置失败时不会请求使能；使能之后失败会主动停用控制器及硬件。
100 ms 看门狗和全部命令边界保持原值。

该时序及 MoveIt 本机限制交集已通过模拟规划执行；11:08 真机也成功完成初始化、
激活和 J2，之后遇到上述独立的 J4 跟踪错误。后续应以被动 CAN 记录定位，
不能仅凭几次成功启动认定看门狗或跟踪问题彻底消失。

## 启动命令

宿主 Ubuntu 图形终端中运行：

```bash
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh up
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.local.yaml \
  enable_execution:=true
```

`startup_ready` 默认 true，但仅在明确设置 `enable_execution:=true` 时执行。
先在硬件 INACTIVE 时配置控制器和准备通信，再使能并成组激活。随后处理 J6 回零，再依次执行 J2、J4、J3；每段都检查到位。
目标为 `[0,-1.350,3.000,-0.300,0,0]`，保持检查通过后继续维持使能，可在 RViz
使用 `startup_ready` 命名状态及 MoveIt 规划执行。等待日志
`startup_ready reached and verified; controller continues holding` 后再操作规划。
启动报告保存到该次 ROS 日志目录的 `startup-ready.json`。
MoveIt 和 RViz 的关节规划范围自动取 commissioning 与硬件 profile 的交集，
速度、加速度也只取更小者，避免普通窗口生成超出本机权限的路径。

默认不传 `enable_execution` 时仅观察/规划。追加 `startup_ready:=false` 可使能后
原位保持；追加 `use_rviz:=false` 可在本地图形会话中不打开 RViz。
Ctrl-C 后等待 `VERIFIED clean controller exit`，不要在退出尚未确认时启动第二个 owner。

## 当前本机参数与范围

- `can2` 由 sysfs 绑定物理通道 2，USB `1209:2323`，完整适配器及电机身份校验保留。
- CAN-FD 1M/4M，采样点 0.8/0.8，SJW 5/3，restart-ms=0。
- Kp `[80,80,120,110,80,80]` Nm/rad，全部 Kd=15 Nm·s/rad。
- 重力比例 `[0,1,1.05,0.7,0,0]`，无 payload；J3 PD 450‰，其余 500‰，总输出 650‰。
- 六轴速度 0.1 rad/s、加速度 0.1 rad/s²；J6 准备轨迹最多使用其中一半。

| 关节 | profile 指令范围 rad |
|---|---|
| J1 | −0.01～0.01 |
| J2 | −1.57～−1.33 |
| J3 | 2.98～3.14 |
| J4 | −0.32～0.01 |
| J5 | −0.01～0.01 |
| J6 | −0.25～0.02 |

本机 profile 是 Git 忽略文件，包含设备身份与零偏；完整行程、不同负载和其余大范围
运动尚未验收。目前成功试验仅覆盖折叠退出、指定 ready 姿态保持及其邻域 MoveIt 执行；J4 间歇跟踪误差、长时和重复冷启动稳定性仍待验收。
启动参考仍依据实物折叠姿态 J1～J5=`[0,-1.570,3.140,0,0]`，不是把当前任意姿态
自动当作零点。失能后的自然摆放可能改变 J1/J5，若超出本机窗口，重新核对参考。

所有 owner 停止后，可以只读检查：

```bash
python3 scripts/read-meow-state.py --interface can2 \
  --profile config/hardware/firefly_y6.meow.can2.local.yaml \
  --output /tmp/hex-arm-disabled-state.json
```

切换 CAN 接口时，通过 `scripts/bind-can-profile.py` 生成新的本地绑定文件，再把
`HEX_ARM_CAN_IFACE` 与 profile 一起换成新接口；名称和物理通道均没有写死。

## 软件验证

最终 controller/bringup/MoveIt 构建成功；Rust 272 项通过、1 项原有测试忽略，
bringup 25 项和 MoveIt 25 项通过，CAN 绑定工具 3 项通过。
新增独立接收时序抖动的 J4 插值测试（32 组序列）也通过，但没有复现真机错误；
该结果不代表已经证明真机故障原因。CAN 重连为 DOWN 时现在给出明确配置提示。
自动启动与 RViz 修复后的针对性测试通过；
规划交集新增“只收缩、不放大”和“不相交则拒绝”检查。
最终编译版本的 [完整模拟自动启动](2026-09-08-coordinator-final-mock-result.json) 通过：
J6 准备、J2→J4→J3、10 秒保持、停用和进程退出均成功。该次强制 MockBackend，
使用不存在的 CAN 名称，不会触碰 can2。真实硬件保护始终由 Rust 独立执行。
前期 Rust 插值、桥接和退出路径的检查见同目录历史记录。

## 下一轮被动 CAN 记录

在启动真机前，从另一终端开始接收记录。该工具不发送 CAN 报文，也不创建 ROS
参与者。使用新的输出文件名；输出包含参考零偏、接收单调时钟、CAN ID 和原始数据。

```bash
python3 scripts/record-meow-pdo.py --interface can2 \
  --profile config/hardware/firefly_y6.meow.can2.local.yaml \
  --seconds 90 --output /tmp/hex-arm-pdo-trace.jsonl
```

重点对照 J4 RPDO `0x204` 的前两个 float32（位置 Rev、速度 Rev/s）与 TPDO
`0x184` 的首个 int32 Q8.24 Rev，并与 `startup-ready.json` 的时间/位置记录对齐。
同时保留 J2/J3 和其它轴数据，以排查多轴共同插值和总线问题。
