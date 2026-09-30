# CiA402 真机部署与验收

适用：旧版 CiA402 固件、SocketCAN、ROS 2 Jazzy。本机 2026-09-29 使用
`can2`，六轴固件 revision 9，无夹爪及附加载荷。
Meow 的寄存器、增益、零偏和启动步骤不能直接用于这版固件。

当前状态：完成本条机械臂的软件零偏校准后，J2 已通过 +0.24 rad、J6 已通过
双向 0.24 rad、J4 已通过 −0.12 rad 的独立轴往返。**完整六轴真机执行尚未验收**。
候选参数、其余轴结果和部署限制见
[扩大行程记录](commissioning_evidence/2026-09-29-can2-expanded.md)。
[校准前记录](commissioning_evidence/2026-09-29-can2-cia402.md)保留为历史证据，
其中旧 J6 位置窗口不再适用于本条机械臂的新零偏。

## 无界面环境

在宿主机仓库目录执行：

```bash
export HEX_ARM_HEADLESS=1
export HEX_ARM_CAN_IFACE=can2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

此模式使用 `compose.headless.yaml`，不依赖 DISPLAY、Xauthority、DRI 或 NVIDIA。
SocketCAN 仍通过宿主网络访问，设备序列号、物理通道和位时序预检保持生效。
切换容器配置前先正常结束现有控制程序；不要同时运行多个 CAN 控制端。
`HEX_ARM_HEADLESS=1 ... real-launch` 自动添加 `use_rviz:=false`。

容器内构建：

```bash
./scripts/build.sh
source install/setup.bash
```

CMake 默认启用生产 CiA402 后端。独立关节调试程序需要显式构建：

```bash
./scripts/build.sh --packages-select hex_arm_controller \
  --cmake-args -DHEX_ARM_ENABLE_CIA402=ON -DHEX_ARM_BUILD_COMMISSIONING=ON
```

`cargo` 直接构建时需加 `--features legacy`。不要把纯 Meow 构建作为 CiA402 驱动使用。

## 先读取，再初始化

停止所有控制端后，在可访问 can2 的宿主机或容器执行：

```bash
python3 scripts/read-cia402-state.py --interface can2 \
  --profile config/hardware/firefly_y6.cia402.can2.commissioning.local.yaml \
  --output /tmp/cia402-state.json
```

该工具仅发送 SDO upload，读取身份、状态字、模式、故障码、温度和连续编码器样本；
不配置 PDO、不写入对象、不使能。退出码非零表示读取失败、身份不一致、存在故障，
或无法确认失能。JSON 在失败时也保留已读节点和错误。`passed` 只表示这次只读检查通过，
不代表标定、静态保持或轨迹执行通过。检查 `heartbeat_consumer` 可追踪退出是否解除心跳监控。

本机 profile 保存在 Git 忽略的 `config/hardware/*.local.yaml` 中。
2026-09-29 的 commissioning 和各轴调参配置保持 `calibrated: false`，不能用于六轴使能。
校准后的 commissioning 配置使用 J6 `−0.25..0.25 rad` 调试窗口；实际通过范围见证据记录。
历史 survey 的 `2.06..2.16 rad` 窗口只对应校准前坐标，不能混用。

旧 schema v2 配置需要迁移至 schema v3、`joint_coordinate_version: 2`。
J3 的零偏和位置上下界均减去 `1.57 rad`，其余轴保持原坐标；这只是坐标变换，
不是重新标定。确认实际末端载荷后设置 `tip_payload`，换适配器后通过
`scripts/bind-can-profile.py` 更新设备绑定。不能因为文件成功迁移就设置 `calibrated: true`。

离线验证当前配置，不访问 CAN：

```bash
ros2 run hex_arm_controller hex_arm_controller \
  --profile config/hardware/firefly_y6.cia402.can2.commissioning.local.yaml \
  --validate-profile-only
```

## MoveIt 观察与位置窗口

更换机械臂或重新校准编码器时，先使用
[默认折叠姿态零点校准工具](zero_calibration_cn.md)重新生成各轴 `zero_offset_rad`，
并用 `--check` 独立采样复查。一次零点计算不继承旧机械臂的整机验收结论。

宿主机使用监督启动入口：

```bash
HEX_ARM_HEADLESS=1 HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.cia402.can2.commissioning.local.yaml \
  position_limits:=hardware enable_execution:=false
```

| 参数 | 位置范围 | 速度/加速度 |
|---|---|---|
| `position_limits:=commissioning`（默认） | 历史窄窗口与本机 profile 的交集 | commissioning 与本机 profile 的较小值 |
| `position_limits:=hardware` | URDF 与本机 profile 的交集 | 同上，当前上限均为 0.1 SI |

这样可以验证新姿态或分阶段增加的硬件窗口，避免永远被最早的窄窗口卡住。
修改这个参数不会修改电机 profile、零偏、标定标志或速度限制。
观察模式使用已有的 plan-only 碰撞模型，其规划结果不能作为严格碰撞验收证据。
真机执行仍使用严格 SRDF，并要求已标定 profile。

CiA402 的执行启动流程是在实测姿态使能，先验证 3 秒静止保持，再激活轨迹控制器，
继续验证 1 秒。位置误差阈值 0.005 rad、速度阈值 0.02 rad/s；
不完整、重复关节名、非有限反馈会被拒绝。只有通过后才交出轨迹执行能力。
Meow 的 J2→J4→J3 自动展开流程没有移植为 CiA402 的已验证动作。

## 独立轴调试和验收顺序

换臂后的参数准备、重力补偿与可重复的分阶段测试入口见
[分阶段验收说明](cia402_staged_commissioning_cn.md)。

`hex_arm_commission --commission-axis` 每次只使能一个轴，走平滑往返并执行反馈保护，
退出时确认六轴失能、解除心跳监控。它允许身份已核实但尚未完成全臂标定的配置。
返回成功表示通过这一个工况的既有验收；失败的轨迹和参数不能自动推广。

后续应先完成证据记录中尚未通过的重力保持和运动工况，检查折叠姿态的严格碰撞报告。
再按多个姿态、顺序展开、严格碰撞规划、多轴慢速轨迹、连续运行依次验收。
只有六轴都完成验证才生成独立的 calibrated 部署配置；扩大位置范围和提升速度分别验证。

软件回归入口：

```bash
./scripts/test.sh legacy
./scripts/test.sh protocol
./scripts/test.sh mock
```

宿主机 Docker 配置和退出监督检查：

```bash
bash scripts/test-docker-gpu-config.sh
bash scripts/test-supervised-real-launch.sh
```
