# 2026-09-07：can0 真机部署预检

后续通信已恢复，当前结果见 [Meow 通信与顺序启动记录](2026-09-07-meow-startup.md)。
以下保留通信恢复前的预检快照。

基线：ROS 仓库 `18e3bc3`，参考 GUI `d20b7a4`。
环境：本地 Ubuntu 24.04，ROS 2 Jazzy 在 `ros2-jazzy-arm` 容器内。
截至 14:39（Asia/Shanghai），软件构建和离线集成通过，但未收到电机回复。
本轮没有发送使能、目标、NMT、SDO 参数写入或故障复位命令；
没有改变 CAN 配置，也没有修改已有硬件 profile。

## 已完成

- 用 `./scripts/docker-dev.sh up` 修复旧容器过期 Xauthority 挂载；
  当前容器能访问 `can0`，X11 与 NVIDIA OpenGL 检查通过。
- 容器内 `./scripts/build.sh`：9 个包构建成功，安装的控制器已更新。
- `./scripts/test.sh unit`：Rust 259 项通过、1 项真实适配器测试默认跳过；
  ROS/MoveIt 100 项通过，含严格碰撞检查、仅规划与 mock 轨迹执行。
- 单独绑定当前适配器运行该真实 CAN 只读预检：1 项通过。
- `./scripts/test.sh protocol`：Rust mock → Zenoh → ROS bridge 生命周期及反馈通过。
- `python3 scripts/test-import-gui-mit-profile.py`：5 项通过。
- GUI 内置 Firefly Y6 v1.1.0 与 ROS URDF 的关节和惯量 XML 相同；
  park 映射、重力比例、增益、力矩换算已有回归覆盖。

以上没有覆盖实际 Meow PDO 配置、500 Hz 六轴反馈、使能/失能、
硬件心跳超时、重力渐入或带载跟踪。

## 现场通信

| 项目 | 实测 |
|---|---|
| 接口 / 通道 | `can0` / 0 |
| 适配器 | HexMeow Quad CAN-FD，`gs_usb`，`1209:2323` |
| USB 序列号 | `45DE24DA66E442869CF51CF944CB1B66` |
| 链路 | UP，MTU 72，FD，ERROR-ACTIVE |
| 仲裁段 / 数据段 | 1 Mbps / 4 Mbps；采样点均 0.800；SJW 5 / 3 |
| restart-ms / 错误计数 | 0 / 未观察到错误计数增长 |
| 被动监听 | 4 秒内无帧 |
| 只读发现 | 宿主机旧二进制及容器最新二进制均报告缺失节点 1–6 |
| 主动只读探测 | 节点 1 的 `0x1018:01` upload，经典 CAN、FD+BRS 各一帧，均无回复 |
| netdev 完成计数 | RX=0、TX=0；内核 CAN 层另记录了 2 次提交 |

两次请求均为 `0x601` / `40 18 10 01 00 00 00 00`。
内核接受请求不证明线上成功发送或收到 ACK；接口 UP、错误为零也不证明电机在线。
优先核实电机供电、实际连接的物理 CAN 通道、线束/终端及电机通信配置。
当前证据无法进一步区分这些原因，也无法识别实际电机固件。

## 尚缺的部署条件

### 1. 确定固件协议

用户提供的 [OD-08 对象字典](https://docs.hexmeow.com/hex-motor/od-08/)
定义 CiA402：`0x6040` 控制字、`0x6060=5` MIT、`0x2003` 未压缩 MIT、
`0x6064` Float32 Rev。Meow 后端对应另一套字典：
`0x4401/0x4402` 模式、MIT=4、`0x4102` MIT、`0x4564` Q8.24 Rev。
应以当前 `0x1018` 身份及只读对象返回选择协议，不能只凭文档版本猜测。

本地 SDK `hex-motor 0.3.1` 识别 Meow vendor 为 `0x00686578`，
4310/4342 product 为 `0x6C64BC78` / `0x6C64BCAA`。
旧本地 profile 则记录 `0x4859444C` / `0xAAAA0001,0xAAAA0002` 的 CiA402 设备。

### 2. 创建匹配当前设备的 profile

- `firefly_y6.discovered.local.yaml` 仍为旧 CiA402、`can2` / 通道 2、
  USB `C9E29601798421B29AC2D419C12D9502`，且 `calibrated: false`。
- `firefly_y6.meow_mit.example.yaml` 的六轴身份、USB 序列号仍是占位符，
  `validated: false`、`calibrated: false`；实际运行校验会拒绝它。
- `scripts/docker-dev.sh` 默认序列号也是旧适配器，现场调用必须显式绑定本次适配器。

需要重新发现六轴身份和节点到关节映射，创建新的本地 profile。
本轮没有复制旧身份作为当前读数，也没有提升验证/标定标志。

### 3. 确认零点、重力、增益和末端

模板 park 编码器值来自 GUI 的一次参考。必须在确认的物理参考姿态读取新鲜位置，
核对六轴方向、限位、基座重力方向，不能将任意摆放姿态当作 park。
GUI 默认 `gravity_enabled=false`、`capture_gravity_park_on_start=true`；
ROS Meow 模板的非零重力比例与固定零点需要先完成相应确认。

GUI 默认附加质量为 0，旧 profile 的 GR80 0.41 kg 又明确是未标定占位；
需确认真实夹爪/工具质量、质心、安装及 TCP/碰撞几何。
Meow 峰值、出厂 torque_factor 和 gain_factor 应从当前电机读取。
650‰ 总输出与 500‰ PD 只留下 150‰ 前馈预算，代码还要求 15% 前馈余量；
模板 5 Nm 前馈限幅不保证在所有姿态均满足电机预算。

### 4. 打通 Meow 逐轴验收

`src/hex_arm_controller/src/main.rs` 明确拒绝 Meow profile 使用旧
`--commission-axis`、`--diagnose-axis`、`--recover-heartbeat-lost`。
新 Meow runtime 可做六轴保持与轨迹，旧逐轴标定 CLI 不适用。
需要先用匹配固件的 GUI 做可记录的逐轴验收，或补充 Meow 有界标定入口，
再验证 ROS 通信、重力渐入、跟踪和正常停止；不能仅修改 `calibrated` 跳过验收。

### 5. 处理 park 碰撞和运动范围

本轮严格模型测试确认参考姿态 `[0,-1.57,3.14,0,0,0]` 有
`link_1–link_5`、`link_2–link_4` 两对网格接触。
这描述 URDF 参考姿态；本轮没有读到当前机械臂姿态。
需结合实物检查 collision mesh，或建立经验证的无碰撞起始姿态。
`firefly_y6.plan_only.srdf` 的豁免只用于离线验证，不能当作真机验收。

`moveit_real.launch.py` 固定使用 commissioning 窄范围：
J2 `[-1.57,-1.30]`、J3 `[2.85,3.14]` rad，速度上限 0.1 rad/s。
它适合初次验收；GUI 全行程 waypoint 不能直接在该入口执行。

## 恢复通信后的入口

宿主机项目目录：

```bash
./scripts/docker-dev.sh up
HEX_ARM_CAN_IFACE=can0 \
HEX_ARM_CAN_CHANNEL=0 \
HEX_ARM_CAN_SERIAL=45DE24DA66E442869CF51CF944CB1B66 \
  ./scripts/docker-dev.sh doctor

docker exec ros2-jazzy-arm \
  /workspaces/hex_arm_ros2/install/hex_arm_controller/lib/hex_arm_controller/hex_arm_controller \
  --discover-only --transport socket-can --interface can0 \
  --expected-node 1 --expected-node 2 --expected-node 3 \
  --expected-node 4 --expected-node 5 --expected-node 6 \
  --auxiliary-node 15 --timeout 3
```

成功识别并准备经过复核的 profile 后，先使用受监督观察入口：

```bash
# 路径须替换为已完成身份、协议、安装参数复核的 profile。
HEX_ARM_CAN_IFACE=can0 \
HEX_ARM_CAN_CHANNEL=0 \
HEX_ARM_CAN_SERIAL=45DE24DA66E442869CF51CF944CB1B66 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  enable_execution:=false
```

观察入口会配置电机 PDO 等易失参数，须在通信和身份核对后执行；
它不是纯 SDO 只读扫描，本轮没有执行。首次使能、低速运动、真实失能及失联验证仍待完成。

本地日志：`log/deployment-20260907-{build,unit,protocol,can-preflight,discovery}.log`。
protocol 成功时无文本输出，本轮退出码为 0。doctor 和链路记录另存于
`/tmp/deployment-20260907-doctor.log`、`/tmp/deployment-20260907-can-evidence.json`。
