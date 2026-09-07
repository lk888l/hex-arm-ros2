# 最新 Meow 固件：MIT 位置控制与重力补偿部署

这条路径对齐 `hex-gui` 的 Other → MIT-pp-test 六轴测试。参考代码为
`src/sixMotorMitTest.ts`、`src-tauri/src/six_motor_mit.rs` 和
`src-tauri/src/meow_calibration.rs`（2026-09-05 检查），使用发布版
`hex-motor 0.3.1` 的 Meow 协议原语。未连接实机执行动作。

## 为什么旧参数不能直接沿用

本项目旧后端使用 CiA402：`0x6040` 控制字、`0x6060=5`、`0x6064` Float32
单圈反馈和压缩 MIT。上位机当前六轴测试使用另一套 Meow 协议：

| 内容 | Meow 最新固件路径 |
|---|---|
| 模式命令 / 显示 | `0x4401` / `0x4402`，Disable=0，MIT=4 |
| MIT 目标 | 未压缩 `0x4102`，显式关闭 `0x4103:01` |
| 实测位置 / 时间 | `0x4564` Q8.24 圈数 / `0x4713` 16-bit 微秒时间戳 |
| 峰值力矩 / 总输出限制 | `0x4576` Nm / `0x4572` 峰值千分比 |
| 实测力矩 | `0x4577` 峰值千分比 |
| Kp/Kd 单位转换 | `0x4102:07` 只读增益尺度 |
| 出厂力矩校准 | `0x4001` 数据与 CRC |

用户提供的 [OD-08 文档](https://docs.hexmeow.com/hex-motor/od-08/) 展示的是
`0x2003/0x2004`、`0x6040/0x6060` 这一套对象字典；不能把地址零散替换后
继续使用旧状态机。新配置必须显式设置 `bus.protocol: meow`。
未写此字段的旧 profile 继续使用 `cia402`，不会自动切换协议。
旧单轴诊断及 heartbeat-lost 恢复 CLI 不接受 Meow profile。

旧现场记录还含 J3/J4 关闭重力、很小的前馈权限和不同的增益。
这些与上位机稳定测试不等价。此前“无力/大力仍抬不起”的具体实机原因无法由
离线代码证明；当前修正解决协议、量纲和参数路径的不一致，不能用继续加大
力矩替代姿态、方向和校准核对。

## 对齐的参数与单位

| 参数 | joint_1 → joint_6 |
|---|---|
| 方向 | `[-1,-1,+1,+1,+1,+1]` |
| park 电机位置 Rev | `[0.000269,0.249714,0.251193,-0.001590,-0.004028,0.000018]` |
| park URDF 角度 rad | `[0,-1.57,3.14,0,0,0]` |
| 重力比例 | `[0,0.3,0.7,0.7,0,0]` |
| 重力前馈限幅 Nm | `[0.2,5,5,1,0.2,0.1]` |
| Kp / Kd | 全轴 `80 Nm/rad` / `15 Nm·s/rad` |
| PD / 总输出上限 | 全轴 `500‰` / `650‰`（电机峰值） |
| MIT 更新率 | `500 Hz` |
| 启动前馈渐入 | `5 Nm/s`，完成后直接跟随每帧实测姿态 |

坐标关系为 `q = direction * 2π * motor_rev + zero_offset_rad`，其中
`zero_offset_rad = park_q - direction * 2π * park_motor_rev`。
park 是物理参考姿态，**不是使能后立即跳转的目标**。使能先保持新鲜实测位置。
模板中的 park 是上位机的一次参考；重新上电、换电机或重设编码器后必须复核。
GUI 第一条测试 waypoint 的 M2=0.25 Rev 对应约 -1.571797 rad，超出本 URDF
的 -1.57 下限；因此没有把 GUI 的运动 waypoint 直接复制为 ROS 轨迹。

补偿使用全六轴实测 `q` 计算耦合 `G(q)`，先逐轴乘比例、限幅，再转换方向。
`kp_raw = round(Kp * 2π / gain_factor)`，Kd 同理。
仅 Tff 乘出厂 `torque_factor`，反馈力矩除该 factor；它与 gain_factor 不同。
Meow profile 强制 `torque_scale: 1`，避免再叠加旧经验系数。
出厂区有效时使用其校准；读取成功但未标定/无有效数据时与 GUI 一样明确记录
`torque_factor=1` 回退，SDO 通信错误则拒绝启动。
`limits.torque_nm` 是主机前馈命令限值，不是全部 PD+FF 输出的物理力矩上限；
总输出由电机 `650‰` 限制，PD 支路由 `500‰` 限制。
驱动在使能前检查完整重力目标，并在每次更新检查
`ceil(1.15 * abs(Tff_wire) / peak * 1000) + PD_permille <= max_permille`。
余量不足会明确报错，不能通过压低前馈或静默饱和来掩盖；降低重力比例、载荷或
重新核对逐轴权限前，先确认单位、标定和真实姿态。

ROS 桥保持 `kp/kd/tau_ff` 空数组，让 Rust 采用 profile 增益与自动补偿。
非空 `tau_ff` 是客户端负责的显式前馈，会绕过自动重力与启动渐入。
本改动是在 ACTIVE 位置控制中补偿重力，不启用自由拖动的 GRAVITY_COMP 模式。

## 重建控制器

在选定的 ROS2 Jazzy 运行环境中重新构建，避免 `ros2 run` 仍执行现有 install
目录里的旧二进制。该环境需要 Rust/Cargo 与项目 ROS2 依赖：

```bash
# 在仓库根目录；WSL 和容器分别使用各自的实际路径
./scripts/build.sh
source install/setup.bash
```

不要交叉 source 绑定于另一挂载点的 symlink-install。如果已有 build/install
由 `/workspaces/hex_arm_ros2` 容器生成，原生 WSL 应另用独立 build/install 目录，
例如 `./scripts/build.sh --build-base build-wsl --install-base install-wsl`，然后
`source install-wsl/setup.bash` 并相应设置 profile 的 urdf_path。

## 准备 profile

在 **WSL Ubuntu 终端**、仓库根目录执行。新模板留有身份占位，不能直接使能：

```bash
python3 scripts/import-gui-mit-profile.py \
  --profile config/hardware/firefly_y6.meow_mit.example.yaml \
  --output config/hardware/my_arm.local.yaml
```

如果有原硬件 profile，可作为 `--profile` 输入。工具保留身份、适配器、重力向量、
载荷和已有运动/前馈限值，只导入所列 MIT 参数和坐标映射；旧前馈限值太低时会
明确报告。用 `--park-positions-rev M1 M2 M3 M4 M5 M6` 提供新鲜 park 读数。
该读数必须采自上述实体折叠姿态，不能将任意摆放位置当成 park。
输出必须是新的 `*.local.yaml`，不覆盖已有文件，也不伪造 validated/calibrated。

先用现有 `--discover-only` 读取 `0x1018` 身份，填全六轴 vendor/product/revision/
serial 和 USB 适配器指纹。该发现路径只有心跳监听和 SDO upload，两种协议共用。
Meow 产品代码必须与 SDK 支持的 4310/4342 身份相符。固件升级后重新读取 revision，
不要沿用旧 fingerprint。节点15仍是辅助节点，不会被初始化、使能或发送机械臂目标。
如果带夹爪/工具，按实际质量、质心填写 `tip_payload`；GUI 默认附加质量为0，
不能据此删除实物载荷。历史未标定的0.41kg占位也不能直接标记为已标定。

完成身份、安装方向和 profile 结构复核后设置 `validated: true`，暂保留
`calibrated: false`。在运行控制器的环境中修正 `urdf_path`：容器为
`/workspaces/hex_arm_ros2/...`，原生 WSL 为实际 `/home/kk_wsl/...`。
离线检查（不会打开 CAN）：

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /absolute/path/my_arm.local.yaml --validate-profile-only \
  --check-pose-rad 0 -1.57 3.14 0 0 0
```

输出每轴 q、电机圈数、原始 G(q)、限幅后重力和物理增益。确认 motor_rev 与
park 读数相符、J2/J3/J4 的符号正确且前馈限值足够。Meow 命令窗口按 Q8.24
反馈的 `[-128,128)` 圈验证，不再错误套用旧单圈 seam 约束；URDF 关节限位仍有效。

## WSL2 实机启动

控制器必须能在**自己的 Linux 网络命名空间**看到 CAN 接口。先在实际运行
控制器的 WSL/容器里执行 `ip -details -statistics link show can0`。
Docker Desktop 的 host network 并不代表 Ubuntu-24.04 发行版的网络命名空间；
若容器看不到该 CAN 接口，应在拥有接口的 WSL 中原生安装/构建 ROS2 Jazzy 工作区，
或使用同一 WSL 中的 Docker Engine。不能靠传入一个 can0 字符串解决隔离问题。

USB 适配器应先交给 WSL，并由 SocketCAN 驱动创建接口。配置/检查时序后再启动
owner；当前 profile 契约为 1M/4M、SP=0.8、SJW=5/3、FD on、restart-ms=0。
上位机 MIT 测试和 ROS 控制器必须先后独占适配器。新后端使用 SocketCAN，
不会沿用旧 gs_usb 固定1M/5M配置。

在已构建并 source 的原生 WSL 终端中，先只观察/规划：

```bash
ros2 launch hex_arm_moveit_config moveit_real.launch.py \
  hardware_profile:=/absolute/path/my_arm.local.yaml enable_execution:=false
```

观察启动会验证身份、配置禁用状态下的 PDO 并读取校准；不会自动清错或写入
永久参数，且要求六轴原本处于 Disable、heartbeat consumer 无其他主站占用。
通信错误、非法尺度、模式异常、反馈过期都会拒绝激活。确认 RViz 姿态、
反馈方向、零点、工具惯量和严格碰撞模型后完成实机标定，再设置 calibrated。
执行入口仍需显式开启：

```bash
ros2 launch hex_arm_moveit_config moveit_real.launch.py \
  hardware_profile:=/absolute/path/my_arm.local.yaml enable_execution:=true
```

使能后先等待控制器终端的 `gravity_ready` 日志（同时发布事件，表示前馈渐入已完成），再发送小幅、低速轨迹。
渐入期间保持激活位置，提前发送的自动补偿运动命令会被拒绝；保持命令仍须持续
刷新看门狗。此事件表示前馈已收敛，仍应观察机械臂实际静止再动作。MoveIt 继续采用已有
commissioning limits；模板里 GUI 的0.2Rev/s不是第一次运动的推荐速度。
park 位姿的旧 collision mesh 接触问题仍需核对，不能为了执行复用 plan-only
碰撞豁免。停止时在 launch 终端按 Ctrl-C，等待控制器确认六轴 Disable、
逐轴解除 heartbeat consumer 并退出；不要结束控制器进程来代替正常失能。
若失能无法确认，停止主站心跳并保留 consumer，让电机 heartbeat 保护生效，
故障保持锁存；传输/后台任务失败须退出并排查后重新启动，不能靠清错假报恢复。

若 CAN 对容器确实可见，可沿用 `scripts/docker-dev.sh real-launch` 的受监督
入口；接口、channel、profile路径须与实际运行环境一致。部署中的物理急停应可用。

## 离线回归

```bash
python3 scripts/test-import-gui-mit-profile.py
cargo test --locked --manifest-path src/hex_arm_controller/Cargo.toml
```

新后端协议/目标编码、力矩校准与预算、启动重力限幅/渐入/量化边界
以及旧后端回归在无 CAN 硬件环境验证。ROS、MoveIt 与实机运动验收是后续独立步骤。

本次在 WSL 临时 Rust 1.88 工具链验证：控制器 Rust 测试 259 项通过，
1 项真实适配器测试按要求跳过；profile 导入器 5 项通过；实际运行
`--validate-profile-only --check-pose-rad` 的输出与上面的 GUI park 参考一致。
当前 WSL 未安装 ROS2，Docker Desktop 因本机 socket 错误无法启动，因此本次
没有执行 colcon/ROS2/MoveIt 集成测试，也没有使能或移动真实电机。
