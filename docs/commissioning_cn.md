最新 Meow 固件请使用 [MIT 部署说明](meow_mit_deployment_cn.md)。本页 0x6040/0x6060、压缩 MIT 和单轴诊断步骤属于旧 CiA402 后端。

# 有监督的硬件调试

[English](commissioning.md) | **中文**

在机械臂已可靠固定、工作空间已清空，并且独立的物理急停已经测试通过之前，
不要启动真机。WSL2 和 Docker 都不是安全系统。

当前支持的现场路径是 Linux SocketCAN。现场链路的完整契约是：仲裁段
`1 Mbit/s, SP=0.8, SJW=5`，数据段 `4 Mbit/s, SP=0.8, SJW=3`，CAN-FD 开启，
自动 bus-off restart 关闭（`restart-ms 0`）。打开总线前，在本地 Ubuntu 宿主机
严格配置全部字段：

```bash
sudo ip link set dev can0 down
sudo ip link set dev can0 type can \
  bitrate 1000000 sample-point 0.8 sjw 5 \
  dbitrate 4000000 dsample-point 0.8 dsjw 3 \
  fd on restart-ms 0
sudo ip link set dev can0 up
ip -details -statistics link show dev can0
```

最后一条命令必须逐项显示上述 bitrate、sample-point、SJW、`fd on`、
`restart-ms 0` 和 `ERROR-ACTIVE`，且 TEC/REC、累计 CAN 错误以及 netdev
错误计数均为 0。历史累计的 netdev `dropped` 可以非零，但必须在相隔 0.2 秒的
两次预检采样中保持不变；只要增长就会 fail-closed。真机 profile 的
`bus.expected_link` 会在打开 CAN socket 之前逐项核验这些字段和 USB 适配器指纹；
任一不符都会 fail-closed。

也可以在**本地 Ubuntu 宿主机**使用现有工具设置速率：

```bash
can-config set can0 4M
ip -details -statistics link show dev can0
```

`can-config set can0 4M` 会依次 DOWN、改时序、UP，因此禁止在 ROS 驱动、MoveIt、
电机 GUI 或其他进程正在使用该接口时运行。它只修改宿主机 SocketCAN 控制器，不会
修改任何电机固件的 CAN 速率；当前工具也不主动写 `restart-ms`，所以其后仍必须确认
`restart-ms 0`，否则使用上面的完整 `ip link` 流程重新配置。

旧的用户态 `gs_usb` 后端固定为 1M/5M，与现场链路不兼容。使用 SocketCAN 时，
USB-CANFD 由宿主机 `gs_usb` 内核驱动管理；不得同时打开 `hex-motor-gui` 的 direct
`gs_usb` 模式，也不得让另一个容器直通并独占同一 USB 设备。切换工具前先完整退出
当前控制程序。

## 安全观察入口

仅查看离线模型时使用以下命令。它不打开 USB、不加载 ros2_control，也不会使能电机：

```bash
ros2 launch hex_arm_bringup view.launch.py
```

### 在 `can0` 上只读发现身份

本地 Ubuntu 的普通容器使用 host network，可以直接看到宿主机 `can0`；不需要
映射 USB，也不需要 `HEX_ARM_REAL=1` override：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh shell
```

机械臂已可靠固定、物理急停触手可及、功率输出级保持禁用，并且电机电子部分已
上电、能够发送心跳后，在已经构建的容器内执行：

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/hex_arm_ros2/install/setup.bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --discover-only --transport socket-can --interface can0 \
  --expected-node 1 --expected-node 2 --expected-node 3 \
  --expected-node 4 --expected-node 5 --expected-node 6 \
  --auxiliary-node 15 --timeout 2 --sdo-timeout-sec 0.25
```

每个 `--expected-node` 指定一个机械臂节点，因此要重复填写；如果完全省略这些
参数，默认期望节点 1 到 6。现场接线已经确认采用直接映射：节点 1 到 6 分别就是
`joint_1` 到 `joint_6`。节点 15（十六进制 `0x0f`）是夹爪电机，只作为辅助设备
allowlist，六轴驱动和 MoveIt 规划组绝不会对它初始化、使能、失能或发送指令。
但配置 `tip_payload` 后，任何机械臂驱动配置之前仍必须精确匹配它的 `0x1018`
身份，因为重力补偿不能忽略已安装设备的质量。缺少预期节点、身份读取失败或
出现未声明节点时，命令都会失败。

`--discover-only` 监听 CANopen 心跳，并且只允许身份 SDO upload 请求。传输层安全门
会拒绝 NMT、PDO、宿主机心跳发送、SDO download、控制字和共享电机命令。该分支
不加载 profile 或 URDF、不初始化驱动器、不启动 Zenoh/ROS，也不清除故障。它不会
修改驱动器配置，但并非电气上的完全被动监听，因为会发送 SDO upload 请求。

发现报告成功只能证明节点存在且 CANopen 身份可读。节点/关节直接映射已由机械臂
配置确认，但方向、执行器零点、关节限位、力矩缩放或 TCP 仍未完成动作标定。

### 固定宿主链路和适配器指纹

`expected_link` 还要求 `driver=gs_usb`、USB VID:PID `1209:2323`、精确的 32 位十六
进制 serial 和 channel。可在宿主机检查：

```bash
readlink -f /sys/class/net/can0/device/driver
udevadm info --attribute-walk --path="$(readlink -f /sys/class/net/can0/device)"
cat /sys/class/net/can0/dev_id
cat /sys/class/net/can0/dev_port 2>/dev/null || true
```

从 `device` 向上找到包含 `idVendor`、`idProduct` 和 `serial` 的最近 USB 父目录，
把实际值写入本机被 git 忽略的 profile。当前已发现适配器的 serial 是
`C9E29601798421B29AC2D419C12D9502`、channel/dev_id 为 0；更换适配器或接口后必须
重新读取，不能照抄。预检仍要求当前和累计的 CAN/netdev 错误计数全部为 0；历史
RX/TX `dropped` 只有在两次相隔 0.2 秒的采样中完全不变时才允许非零，计数变化表示
仍在主动丢包并会使检查失败。应查明原因并重新建立干净链路，不能绕过检查。

### 身份确认后的禁用状态观察

将 `config/hardware/firefly_y6.example.yaml` 复制为被 git 忽略的本地 profile，
记录并复核每一项身份和 `expected_link`，并保持
`bus.direct_joint_mapping: true`；这样节点/关节一旦错位会在 profile 校验阶段失败，
其他通用 profile 仍可明确选择不启用该约束。结构与指纹复核后可设置
`validated: true`，即使轴参数仍是调试候选并保持 `calibrated: false`，也可以启动
禁用状态观察。

打开 CAN 前先执行离线验证：

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  --validate-profile-only
```

该命令解析完整 profile、加载 URDF 动力学模型，并拒绝接近或跨越未经验证单圈 seam
的命令窗口；不会打开 USB/SocketCAN、初始化驱动器或启动 Zenoh，也不要求
`calibrated: true`。通过只是必要条件，不代表动作标定完成。随后从宿主机使用受监督
Docker 入口启动禁用状态观察。显式接口/通道必须与 profile 一致；下面使用当前的
`can2`：

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch bringup \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

`activate_hardware` 默认是 `false`。该模式保持 Rust 控制器为 DISABLED，不启动
`ros2_control_node`、hardware spawner 或轨迹控制器；RViz 直接显示桥接反馈。
该模式会打开 profile 指定的 SocketCAN 接口、执行 NMT/SDO/PDO 初始化并启用心跳
监控，因此不能替代首次 `--discover-only` 检查。驱动器初始化绝不会自动执行 fault reset；如果存在 CiA402
Fault，启动会 fail-closed，必须先排除物理原因，再进行显式恢复。重启进程不会清除
电机故障。`validated` 在这里仅表示 profile 结构、节点身份和宿主链路可信，不表示
运动标定完成；`calibrated: false` 的 profile 会被激活门明确拒绝。

宿主辅助脚本会隔离并在后台保持原始 Compose exec，同时为本次启动生成随机 token；
容器只把该 token 绑定到本次 launch 的独立 PGID。收到 `Ctrl-C`、SIGTERM 或终端
挂断后，宿主用第二个非 TTY、目标固定的 `docker exec` 校验 token/PGID 文件，只向
该负 PGID 转发 INT/TERM，然后继续等待原 Compose exec。若请求早于 PGID 发布，
supervisor 会在启动 ROS 前取消；陈旧、畸形或路径注入状态会被拒绝。只有 Rust
控制器已经到达 ready/DISABLED、进入有序关闭路径，且 ROS launch 报告控制器 clean
exit，才报告停止已验证；控制器 clean return 的可执行契约包含“六轴确认失能→心跳
consumer 解除”。失败会返回非零并保留审计日志。该入口不会按进程名全局结束任务，
也不会向独立的 `can1` 进程组发信号。保持命令连接直到看到结果；正常停止不要使用裸
`docker exec ... bash -lc 'ros2 launch ...'`、`pkill`、容器重启或 Compose down。

这里必须区分两个对象：`0x6041` bit 3 表示**当前 CiA402 Fault**，`0x603F`
是可能在故障解除后继续保留的 last-error 诊断。现场出现的
`0x603F=0x8130, 0x6041=0x0231` 表示驱动器当前为 non-Fault、non-OE，不能仅为
擦除历史码而执行 fault reset，也不会阻断禁用观察模式。TPDO1/TPDO2 中的
last-error 仍会保留在诊断快照中。

控制器正常退出（包括单轴 commissioning 的成功、错误、SIGINT 和 SIGTERM 路径）
会先对六轴执行失能，并要求命令后的新 TPDO2 全部确认 non-OE；只有这一步完整成功，
才会在主站心跳仍然发送时逐轴写入并回读 `0x1016:01=0`，最后再次用 SDO 确认
non-OE。失能未确认时不会解除 `0x1016`，而是保留驱动器心跳看门狗。

只有当 `0x6041` 的当前 Fault bit 确实为 1，且对应 last-error **恰好**为
`0x8130` 时，才可在排除物理原因后运行专用恢复命令。参数中的节点集合必须与现场
当前 0x8130 Fault 集合完全一致：

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  --recover-heartbeat-lost 1,2,3,4,5,6 \
  --allow-heartbeat-fault-reset
```

该命令只允许节点 1–6，节点 15 及其他辅助节点不能成为恢复目标；它会复核六轴身份、
辅助节点 allowlist 和非预期节点，且不会初始化、切换模式或使能电机。复位使用明确的
`0 -> 0x80 -> 0` 控制字边沿。成功还要求复位后的新 TPDO2 显示 non-Fault/non-OE、
恢复期间没有新的六轴 EMCY、随后六轴再次完成失能确认，最后才解除六轴的主站心跳
consumer。若当前状态是 `0x0231 + 0x8130`，不要运行此命令，直接启动禁用观察即可。

### 旧 `hex-ros2-arm` 复用审计

旧 `hex-ros2-arm` 仓库的 `e4901be` 提交及其相邻 MoveIt 包的 `3f3985d` 提交只作为
commissioning 证据保存，不作为可直接执行的真机 profile。以下内容已经进入当前架构：

- 关节方向候选 `[-1,-1,+1,+1,+1,+1]`；
- 电机/ROS 力矩换算候选 `[0.85,0.85,0.85,1,1,1]`；
- `arm` 规划组、`base_link` 到 `link_6` 运动链、KDL 求解器，以及
  `firefly_arm_controller/follow_joint_trajectory` 接口。

它们在当前实体机械臂通过逐轴正负方向测试之前仍只是候选。旧仓库没有 CANopen
身份、编码器零位记录、实测行程限位、温度策略、can2 链路指纹或满足容差的真机执行
报告，因此不能据此设置 `calibrated: true`。

旧 Python 桥不会复用：它在构造时立即取控并进入 ACTIVE，只按轨迹经过时间报告
FollowJointTrajectory 成功，不校验路径/目标误差，退出时的 best-effort release 也弱于
当前“反馈确认失能→解除心跳 consumer”路径。旧 `6 rad/s`、`10 rad/s²`、默认
`20/1.5` 增益、未经安装方向复核的固定重力、全零 `home`，以及禁用全部 21 对碰撞的 SRDF 同样不会导入。
当前 can2 实测、窄 commissioning 限位、严格碰撞检查和逐轴调参结果具有更高优先级。

### 当前姿态、方向和零偏证据

当前断点记录的近似 URDF 姿态为：

```text
q_ref ~= [0.000, -1.570, 3.140, 0.000, 0.000, 0.000] rad
```

结合当前断点保留的逐轴只读 `0x6064` 快照与旧项目的方向符号候选
`[-1,-1,+1,+1,+1,+1]`，按
`q_ros = direction * 2*pi*q_motor_rev + zero_offset_rad` 拟合得到一次性零偏候选：

```text
zero_offset_rad ~= [-0.025651, 0.010217, 1.545484,
                     0.000144, 0.026979, 0.121463]
```

这只是“近似参考姿态 + 逐轴只读快照 + 旧配置”组合得到的 commissioning 证据，不是动作
验证结果，也不能据此设置 `calibrated: true`。joint_2 接近 URDF 下限，joint_3 接近
上限，所以 SRDF 只提供内缩到 joint_2=-1.56、joint_3=3.13 rad 的规划参考
`commissioning_start`；不得把
任一数组当作自动 home、在未标定真机上执行，或解释成机械硬限位。

### `0x6064` 单圈反馈和修正后的 joint_2 窗口

电机 `0x6064` 是规范化单圈 `[-0.5, 0.5)` rev。旧的 joint_2=`+1.570` 参考制造了
错误的 seam 阻断；修正为 `-1.570`、`direction=-1` 和新零偏后，完整 URDF 范围约
映射到 `[-0.3310,+0.2515]` rev，处在现有 seam guard 内。控制器仍保留 fail-closed
窗口校验，但不得再用已经废弃的零偏声称 joint_2 必然跨圈。

joint_2 当前位于临时 URDF 下限。本地硬件、URDF 和 MoveIt 命令下限仍统一为
`-1.570`。profile 现已把严格命令包络与只读测量余量分开：本机 J2 的测量余量为
`0.001 rad`，只用于表示断电机械止挡的柔顺/回差和编码器散布，不会授权低于
`-1.570` 的目标。单圈反馈解算和实测状态监控会采用该余量；压缩 MIT 映射、
ROS/MoveIt 限位、被选中轴的 commissioning 目标以及普通命令仍使用原严格边界。
反馈超出显式测量包络仍会 fail-closed。J2 第一次只能执行向范围内的正向小往返；禁止从边界发负向
动作。完整双向标定仍需要先实现有界单向内移，再从内部姿态验证正负方向。当前阻断
真机 MoveIt 的是这个未完成标定，而不是旧 seam 结论。

1. 将 profile 选择的接口（当前为 `can2`）配置并确认在 1M/4M，再执行上面的精确
   `--discover-only` 命令。不得在该链路上使用旧的 1M/5M 直连 USB 路径，也不得
   给容器授予 `privileged` 权限。
2. 记录六个 CANopen 节点的节点 ID、厂商、产品、版本和序列号身份指纹；将节点
   15 单独记录为已知辅助设备，并在建模其质量时把同一精确指纹绑定到
   `tip_payload`。出现非预期、重复、缺失或无法识别的节点时，立即停止流程。
3. 创建一个 `*.local.yaml` 配置文件。按已确认的 node 1→joint_1 到
   node 6→joint_6 录入并保持 `bus.direct_joint_mapping: true`；node 15 只放入
   `auxiliary_node_ids`。旧的方向符号、上述
   零偏候选和历史力矩系数只能作为 commissioning 证据，不能替代逐轴验证。
4. 将物理急停放在随手可及的位置，使用实际可行的最低力矩限制逐轴验证。确认 ROS
   正方向运动后，记录 `direction`；继续下一轴之前，先让当前关节回到安全姿态。
5. 记录电机零点，并根据
   `q_ros = direction * 2*pi*q_motor_rev + zero_offset_rad` 计算
   `zero_offset_rad`。将关节 3 视为已知的特殊情况；不要通过修改 URDF origin
   来掩盖执行器零点问题。
6. 在低速下验证两侧软件限位，且不要接近机械硬限位。力矩缩放必须来自受控标定，
   不能照搬历史 GUI 默认值。
7. 只有 profile 结构、链路指纹、节点身份和候选字段经过复核后，才能设置
   `validated: true`；只有全部六轴都通过方向、零点、限位和力矩检查，并且所有
   `tip_payload` 都已获得实测/复核惯量且设为 `inertial_calibrated: true` 后，才能
   设置 `calibrated: true`。
8. 依次执行：禁用状态发现、专用单轴低速跟踪、六轴保持、最后一条短轨迹。joint_2
   未完成向内重新定位和双向验证前停在单轴阶段，不进入 MoveIt 执行。出现温升、诊断过期、噪声、
   方向异常或任何锁存故障时，立即中止。

Rust 控制器现在已经实现了专用单轴入口。该分支不打开 ROS 或 Zenoh；profile 只要
是 `validated: true` 即可使用，允许仍为 `calibrated: false`。它没有隐含位移或默认
动作，关节、带符号位移、完整往返时长和真机动作确认都必须显式给出。例如，在现场
已经独立确认 joint 1 的 `+0.01` rad 是安全方向之后，执行：

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  --commission-axis joint_1 \
  --delta-rad 0.01 \
  --duration-sec 2.0 \
  --allow-motion
```

`--allow-motion` 不是 dry-run 开关：提供它就表示允许选中的实体电机使能和动作。
命令拒绝 `abs(delta) > 0.03` rad，轨迹为平滑的“起点→delta→起点”，并同时要求
`duration >= pi*abs(delta)/velocity_limit` 与
`duration >= sqrt(2*pi^2*abs(delta)/acceleration_limit)`。当前 commissioning
硬件 profile 的 `acceleration_rad_s2` 暂定为 0.1；使能电机前会独立校验峰值速度和
峰值加速度。
程序先严格预检 profile 选择的接口（当前为 `can2`）、核对全部身份和辅助节点 allowlist，再初始化并确认六个驱动
全部处于 Disabled；之后只配置和使能选中的节点。使能前，共享帧六个槽位已经全部
写入来自当前反馈的 hold；选中轴使用限幅后的低增益和 URDF 重力前馈。运行期间持续
检查反馈新鲜度、故障、位置、速度、力矩、跟踪误差和链路健康。正常结束、Ctrl-C 或
任何错误都会进入带重试且由反馈确认的全轴 disable；无法确认 disable 时命令以失败
退出。一次只调一个关节，delta 的符号必须由现场有人监督的物理布置决定，不能照抄
示例。程序不会因为最后停在起点附近就误判成功：实测带符号峰值必须沿请求方向达到
至少 50% 的请求位移，且始终不能超过 `abs(delta)+0.005` rad；回到起点的误差采用
更严格的动态容差（请求位移的 25%，上限 0.005 rad、下限 0.001 rad）。成功日志会
输出 `measured_peak_delta_rad`，可作为方向调试记录。

本地现场 profile 才是逐轴参数的权威记录，不要从文档正文复制增益或力矩限制。
当前断点 J1..J6 的 Kp/Kd 依次为 `60/2.5`、`80/2.5`、`2/0.3`、`30/1`、
`30/1`、`20/1`，且全部 `velocity_rad_s <= 0.1`；只有 J3 的
`torque_permille=250`，其余为 200，全部轴的 `kp_kd_torque_permille=100`。使用
trial GR80 载荷和修正后的 -Z 基座重力候选时，观测姿态下未缩放模型约为
`[0,+1.865,-5.957,-1.157,0,0] Nm`。这只是模型审计，不是实测保持力矩；当前
J2 在修正后的 25% 试验中，`+0.005 rad` 请求只到达 `0.000534 rad`；50% 试验仍只到达
`0.000810 rad`，而实测力矩峰值已达 `1.264 Nm`，因此停止盲目加力。J2/J3/J4
目前保持为 `0.25/0/0`。这些值都只是
commissioning 候选，不是已认证的安全值。桥发送空 `kp`/`kd` 时由 Rust 使用这些逐轴
默认值；这些值是关节侧 SI 增益，单位分别为 `Nm/rad`、`Nm*s/rad`。桥发送空
`tau_ff` 时由 Rust 根据 URDF、实测关节位置和重力向量自动计算
重力前馈，并把固定 `tip_payload` 的质量/质心合并到 `link_6`。空数组代表“委托
Rust”，显式六个零则代表关闭该自动前馈，二者不要混淆。任何非空外部 `tau_ff`
同样会绕过载荷感知的重力模型和 `gravity_compensation_scale`。逐轴
`torque_scale` 对全部产矩 MIT 项始终生效：关节侧 `tau_ff`、Kp、Kd 下发时乘入
电机力矩域，电机实测力矩返回时再除回关节侧标定域。ROS/MoveIt 桥发送空数组，
所以会走自动载荷感知路径。

2026-08-20 的专用 J2 诊断又给出了一个更窄的链路证据：在保持位置不变时，
`0.025..0.250 Nm` 附加前馈的共享 RPDO 目标与恢复目标均由电机
`0x2004:02/03` 逐位读回；峰值总前馈约 `0.716 Nm`、实测力矩约 `0.706 Nm`，
相对本轮起点只正向移动 `0.000513 rad`，峰值速度 `0.00143 rad/s`，CAN 和温度门
均正常。它证明驱动确实消费了命令，也证明此力矩仍低于静态负载；不能把它当作
joint_2 动作、重力比例或机械限位标定通过。

随后又进行了一次单独二次授权的高阶诊断，每级增加 `0.10 Nm`。当附加前馈达到
`1.00 Nm`（总前馈 `1.466 Nm`）且编码器正向位移达到 `+0.001002 rad` 时，工具自动
停止继续升力；峰值速度为 `0.00618 rad/s`，实测力矩为 `1.412 Nm`。请求中剩余直到
`1.50 Nm` 的级别没有被发送：程序立即恢复基线，先确认 J2 退出产矩状态，再确认其余
五轴，最后才解除全部心跳消费者。结束后的被动位置约为 `-1.568797 rad`，仍在
commissioning 窗口内。因此该姿态下有用的静态脱离区间是附加
`0.90..1.00 Nm`；不要重复 `1.50 Nm` 阶梯，也不要继续尝试 `1.75 Nm`。该结果证明
J2 能产生一个很小的正向响应，也排除了“软件漏发抱闸释放命令”这一解释；它仍不等于
位置跟踪、重力比例、零位或机械行程限位标定通过。
本轮仅使用了一次进程内的 can2 历史计数精确确认：
`error_warning=10`、`error_passive=10`，其余计数为零，运行期完整基线既未增长也未
复位。该确认不是 profile 配置；正常控制和普通 commissioning 仍要求 CAN 历史错误
计数绝对为零。
随后用 J2 比例 `0.65` 做了一次固定 `+0.005 rad / 4 s` 位置诊断，但在轨迹开始前
就安全中止：首个使能后样本已正向偏移 `0.000936 rad`，速度为 `+0.03004 rad/s`，
超过未放宽的 `0.020 rad/s` 硬门。程序恢复基线、确认六轴失能并解除全部心跳消费者，
以非零码退出；运行后 CAN 与 USB 复核仍正常。因此本地 profile 已回退到此前稳定的
`0.25`。不要原样重跑 `0.65`，也不要放宽速度门。下一次试验必须把低前馈安全使能与
轨迹内平滑、受限的前馈渐入分开设计并重新审计。
本轮保留的原始日志元数据和中止样本见
[J2 位置诊断中止证据](commissioning_evidence/2026-08-20-can2-j2-position-abort.md)。
经审计的后续试验消除了该使能阶跃：J2 先以比例 `0.25` 安全使能并通过稳定门，再完成
了有反馈证明的 375 步、1.5 秒固定位置渐入直到 `0.65`。渐入本身的峰值偏移和速度仅为
`0.000712 rad` 与 `0.001149 rad/s`。但在终点比例持续保持时，关节虽已几乎静止，强制
二次稳定门仍收敛到 `+0.000735 rad`，超过未改变的 `0.000500 rad` 位置容差。因此控制器
恢复已登记基线并安全关闭，5 mrad 轨迹的任何部分以及非基线位置读回都没有开始；运行
后 CAN/USB 再次正常。该结果只验证渐入机制，不验证 J2 跟踪或重力标定。本地 profile
继续保持 `0.25`，不能重放 `0.65`，也不能靠放宽稳定门使其通过。完整工件与遥测见
[J2 重力渐入稳定门证据](commissioning_evidence/2026-08-20-can2-j2-gravity-ramp-stability-gate.md)。
最后又执行了一次单独授权的固定位置截断辨识：从保留的 `0.25` profile 值开始，仅按反馈
增加微小重力前馈，代码中不存在位置轨迹分支。位移在比例 `0.363305` 首次越过
`+0.300 mrad`；该目标被拒绝，程序回退到前一个有反馈证明且量化字不同的比例
`0.362088`，并通过精确 `0x2004:02/03` 读回。冻结保持一秒期间，平均位移为
`+0.326 mrad`；末 250 ms 的位置跨度只有 `0.0044 mrad`，峰值速度为
`0.000212 rad/s`。随后六轴均正常失能，CAN/USB 计数保持干净。这只是该单调上升路径下
的截断点，不是已标定的重力比例；单次试验无法分离机械限位预载、静摩擦和滞回。因此 J2
继续冻结在 profile 比例 `0.25`，不要把 `0.362088` 写入 profile，也不要重放本试验。
完整工件见
[J2 截断保持辨识证据](commissioning_evidence/2026-08-20-can2-j2-censored-gravity-hold.md)。
随后仅执行了一次独立锁定的 J1 `+0.005 rad / 4 s` 首次位置诊断。低权限
`30/20`、严格 OE 证明、变化目标与返回目标的两次压缩目标精确读回均通过，清理也确认
六轴全部失能；但跟踪未通过：正向峰值只有 `0.064 mrad`，低于 `2.5 mrad` 门槛，
命令峰值处的实测/估算力矩为 `0.260/0.297 Nm`。CAN/USB 与温度门保持干净。不要
重放该位置试验或增大位移；再次考虑 J1 位置动作前，应先使用单独授权、固定位置、按反馈
截断的小力矩辨识。完整记录见
[J1 首次位置诊断证据](commissioning_evidence/2026-08-20-can2-j1-first-position.md)。
随后只执行了一次单独授权的 J1 固定位置正向截断力矩辨识：位置码保持不变，仅以
`0.025 Nm` 为一级增加关节侧前馈。`0.450 Nm` 首次越过 `+0.300 mrad` 截断门，
因此该级被拒绝，控制器回退到前一个有反馈证明的 `0.425 Nm` 目标，并通过精确
`0x2004:02/03` 读回。冻结保持一秒期间平均位移为 `+0.308718 mrad`；末 250 ms
的位置跨度仅 `0.002436 mrad`，峰值速度为 `0.000226 rad/s`。随后六轴均确认非产矩，
can2/USB 保持干净。这只是本次单向上升路径下的正向响应区间，不是可复用的偏置或标定值。
不要把 `0.425 Nm` 写入 profile、重放这次上升过程，或用它放行 J1 位置跟踪。J1 继续
冻结，直到另行审计并完成负方向固定位置辨识。完整记录见
[J1 截断力矩辨识证据](commissioning_evidence/2026-08-20-can2-j1-censored-torque.md)。
本轮可追溯的工件、硬件与测量摘要见
[2026-08-20 J2 证据记录](commissioning_evidence/2026-08-20-can2-j2-breakaway.md)。

硬件 profile schema v2 强制要求
`gravity_vector_base_m_s2: [gx, gy, gz]`。它表示 URDF `base_link` 坐标系下的
重力加速度（m/s²），不是 world 坐标标签，也不是逐关节方向；模长必须在
8..12 m/s²。v1 profile 和缺少该字段的 v2 profile 都会被拒绝。迁移旧配置时必须先
复核实体底座安装方向，再加入显式向量并把 `schema_version` 改为 `2`，程序不会静默
回退到 `-Z`。当前本地现场 profile 将 `[0.0, 0.0, -9.81]` 记录为修正后的 commissioning
候选，并保持 `calibrated: false`，直到有人监督的保持测试及正、负小动作验证完成。
单轴 commissioning 与 ROS/MoveIt runtime 都用这个相同向量计算 `G(q)`。
`SetGravity` 只能在当前独占会话中临时覆盖；release、shutdown 以及每次新的 acquire
都会恢复 profile 值，该服务不会修改 YAML。

每个关节的 `gravity_compensation_scale` 只对该轴的 URDF 重力估计做乘法缩放；专用
单轴 commissioning 与 ROS/MoveIt 使用的正常 Rust runtime 会采用完全相同的值。
允许范围为 `0.0..=2.0`：`0.0` 关闭该轴的模型重力前馈，`1.0` 使用未经缩放的
URDF 估计，大于 `1.0` 则增强前馈。旧 profile 缺少该字段时兼容默认为 `1.0`，但
现场 profile 应显式记录。它是针对实际负载和安装姿态进行真机辨识的参数，不是随意
调手感的旋钮；它与关节标定域/电机域换算使用的 `torque_scale` 完全不同。后者对
所有带力矩量纲的 MIT 项生效：前馈、Kp、Kd 下发时缩放，实测力矩返回时反向换算。
不能通过修改其中一个来掩盖另一个的错误，也不能用逐轴缩放来翻转或修补基座重力方向：必须
先由 `gravity_vector_base_m_s2` 求出 `G(q)`，之后才应用逐轴标量。每次改变重力
缩放后，都要重新执行有人监督的保持测试和正、负两个方向的小动作测试；六轴验证
完成前保持 `calibrated: false`。

### MoveIt 真机入口

新的真机 MoveIt launch 默认只观察和规划，并固定使用 0.1 rad/s、0.1 rad/s² 及
当前姿态附近窄位置窗的 commissioning planning limits：

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

真机/mock 共用的 ros2_control 配置现采用 fail-closed 容差：每轴路径误差 0.02 rad、
目标误差 0.005 rad、停止速度 0.02 rad/s，目标等待时间保持 1.0 s。因此 joint_2
若在目标阶段仍欠跟随 0.016 rad，会明确失败，不能再被报告为执行成功。这些容差只负责
发现跟踪失败，不会解决当前 joint_2 欠跟随，也不会解除真机执行锁闭。

默认 `enable_execution:=false` 会同时保持 `activate_hardware=false` 和
`allow_trajectory_execution=false`。只有 profile 已完成逐轴标定、设为
`calibrated: true`，并且修正后的 joint_2 已完成双向跟踪验证后，才允许显式使用：

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  enable_execution:=true
```

该入口的配置和 mock 回归已做离线自动测试；本文不声称已经在真机上执行过动作或
MoveIt 轨迹。本地 GR80 trial payload 记录了 identity `link_6` 安装下的 0.41 kg
及合并质心，来源 URDF SHA-256 为
`f74b3e76b14175c788c5ef70dd0c1941958d229a8461da6f20e17543e4ba1114`。这些是明确的
trial 占位而非实测值；`inertial_calibrated: false` 会阻止 profile 标为 calibrated，
也就禁止真机 MoveIt 执行。当前 `link_6` 仍只是规划法兰末端，真实 TCP、已标定工具
惯量、夹爪变换和最终工具碰撞几何仍待完成。

只有完成上述检查并准备明确使能时，才允许使用：

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch bringup \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  activate_hardware:=true
```

故障恢复永远不会自动使能机械臂。每次故障后，都必须先检查并排除物理原因，再
进行任何显式复位；初始化和进程重启都不会复位故障。激活仍然必须由明确的
ros2_control lifecycle 操作触发。
