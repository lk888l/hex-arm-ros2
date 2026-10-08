# 替换机械臂的 Meow 真机入口

本页对应当前 can2、无附加载荷的替换臂。历史旧臂与 CiA402 成绩不作为本臂新版固件的验收依据。
试验细节和失败记录见[升级验收记录](commissioning_evidence/2026-09-29-can2-meow-upgrade.md)。

## 当前状态（2026-09-30）

操作者确认此前 CAN 故障源于手动关闭动力电源；小范围 MoveIt 真机执行已测试正常，
也已在 `hex-gui` 通过重力补偿和 MIT 命令测试大范围动作。仅重力补偿模式已测试正常，
需要补偿的关节比例为 1。旧轮次的 `disable_unconfirmed` 原始回执保留，不能作为新一轮的失能证明。

本次新增正式动态配置和全范围部署 profile，并通过三个大范围组合目标、返回 ready、
受控回折和确认失能的真机闭环。实测速度峰值 **0.890203 rad/s**、保持最大位置误差
**0.011987 rad**。完整结果见[本次部署验收](commissioning_evidence/2026-09-30-can2-moveit-deployment.md)。
完整 URDF 边界和附加载荷尚未穷举验收。

本日证据：[扩大部署结果与失败记录](commissioning_evidence/2026-09-30-can2-meow-expanded-result.json)。
原始审计、反馈及配置清单位于 `.tmp-meow-expanded/`；本次原始记录位于 `.tmp-moveit-deployment/`。

## 当前配置

`config/hardware/firefly_y6.meow.can2.moveit_deployment.local.yaml` 绑定本臂身份和 can2 适配器，
由已测试的 `hand_guiding_full_range.local.yaml` 派生。旧 `replacement.local.yaml` 保留作窄窗口基线。
六轴方向和软件零偏保持原标定，不因刷固件或摆放变化重新归零。

- Kp：`[100,100,150,110,80,80]` Nm/rad；Kd：六轴 15 Nm·s/rad。
- 重力比例：六轴 1.0，保留渐入与力矩限幅。
- 用户指定六轴总扭矩上限与 PD 配置上限均为 1000（100%），额外前馈预留比例为 0。
  PD 每帧取得前馈占用之后剩余的总额度，不再预留固定 450/500‰ 后因预算相加不足而退出。
  J2 重力前馈仍限幅 ±5 Nm。先前真机验收使用总上限 650；新的额度分配已离线验证，
  尚未重新实机验收。
- 部署 profile 移除旧窄窗口试调的 J2/J3 手动运动前馈，使用重力模型与 PD 跟踪。
- 六轴 `0x4001:01..07` 实读全零，没有可用的出厂摩擦记录。
  旧前馈是本臂试调值，不能当成出厂标定或直接用于另一条机械臂。
- 用户于 2026-09-30 指定到位、保持及回折位置验收为 **0.015 rad**；动态跟踪门限仍为 0.04 rad。
- 保持速度峰值已按用户要求恢复 **0.02 rad/s**；显式使能瞬态仍为前 0.25 秒 0.15 rad/s、
  后续重力渐入 0.05 rad/s。短暂试用的 0.08 rad/s 保持门槛已撤回。
- 命令速度/加速度：六轴 1.2566370614 rad/s、1.2566370614 rad/s²，来自 GUI 的
  0.2 Rev/s、0.2 Rev/s² 乘 `2π`；J2/J4 实测反馈速率额外允许 0.02 rad/s。
- 位置范围：J1 ±2.86、J2 [−1.57,2.09]、J3/J4 ±1.57、J5 ±1.54、J6 ±2.79 rad。
  MoveIt 再与 URDF 取交集；规划和执行使用严格碰撞模型。
- 折叠入口 J2/J3 各允许相对 ±1.570 rad 参考偏差 0.01 rad，配合本 profile
  的 `measured_position_margin_rad: 0.01` 接受边界附近的静止反馈。Meow 使能时
  的保持目标与首次轨迹起点收回到命令范围，后续回折使用已记录的合法保持参考。
  ROS 初始零速度回显仅在首次运动前匹配该保持参考时接受；实际反馈和软零偏保留。

## 动态限值与单位

位置和动态分别选择，默认均为 `commissioning`。正式部署同时传入
`position_limits:=hardware dynamics_limits:=hardware`；此时速度为硬件 profile 与 URDF 的较小值，
加速度来自硬件 profile，不再混入 commissioning 的 0.1 上限。启动日志打印最终 SI 限值。

本次正式入口使用 `dynamics_limits:=custom` 加载 `joint_limits_deployment.yaml`：
规划速度 1.256637 rad/s、加速度 0.6 rad/s²，默认缩放均为 1.0。
100 Hz ROS 命令再经 500 Hz Rust 有界插值；规划与插值采用完全相同加速度上限时，
首轮真机动作触发了 J2 的 0.04 rad 跟踪保护。0.6 的独立规划配置为插值和实际跟踪保留余量，
硬件 profile 仍独立约束所有命令。该余量经过多轴帧抖动回归测试，真机表现以本轮结果为准。

| 数据 | GUI 六轴 MIT / 电机协议 | ROS、MoveIt、Rust profile |
|---|---|---|
| 位置 | Rev | rad，应用方向与软件零偏 |
| 速度 | Rev/s | rad/s = Rev/s × `2π`，应用关节方向 |
| 加速度限值 | Rev/s² | rad/s² = Rev/s² × `2π` |
| Kp、Kd | GUI 输入为 Nm/rad、Nm·s/rad | 同为 SI；写入电机时乘 `2π` |

GUI `ControlPanel.tsx` 的 MIT 输入本来就是 SI；其写入协议时除 `2π`，不能再乘一次。
六轴测试的 `max_velocity_rev_per_s`、`acceleration_rev_per_s2` 才是需要导入换算的 Rev 参数。
ROS 零位以本臂已标定 profile 为准，不能复制 GUI 默认 Park 角度覆盖软件零偏。

在仓库或容器工作目录生成新的本机配置（输出文件必须尚不存在）：

```bash
python3 scripts/prepare-moveit-profile.py \
  --source config/hardware/firefly_y6.meow.can2.hand_guiding_full_range.local.yaml \
  --output config/hardware/firefly_y6.meow.can2.moveit_deployment.local.yaml \
  --urdf src/xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf \
  --velocity-rev-s 0.2 --acceleration-rev-s2 0.2 \
  --full-urdf-range --clear-motion-feedforward
```

已有 SI 数值可改用 `--velocity-rad-s` 和 `--acceleration-rad-s2`。该工具不连接 CAN，
保留源配置的身份、标定、重力比例和 PD 增益；随后使用生产 Rust 的 `--validate-profile-only` 验证。
生成器也保留源配置的扭矩预算；重新生成上述部署 profile 后，需将六轴
预算设为当前指定的值：

```yaml
torque_permille: 1000
kp_kd_torque_permille: 1000
meow_torque_budget:
  feedforward_reserve_ratio: 0.0
  pd_allocation: remaining
```

`remaining` 使用校准后的电机侧前馈绝对值计算占用，每帧下发的 PD 限幅为
`min(PD配置上限, 总上限 − 前馈占用)`。例如前馈占 131‰、总上限 1000‰，该帧 PD 可用 869‰。
前馈本身不被预算分配器缩小，总上限始终约束前馈与 PD 的合计。
没有显式配置 `meow_torque_budget` 的旧 profile 保留 15% 余量与 `fixed` 分配方式。
新增字段需要重新构建 `hex_arm_controller`；当前工作区已经安装支持它的驱动。

需要规划侧单独收紧时，使用 `dynamics_limits:=custom planning_limits_file:=/绝对路径/limits.yaml`：

```yaml
default_velocity_scaling_factor: 1.0
default_acceleration_scaling_factor: 1.0
joint_limits:
  joint_1: &limits
    has_velocity_limits: true
    max_velocity: 0.8
    has_acceleration_limits: true
    max_acceleration: 0.5
  joint_2: *limits
  joint_3: *limits
  joint_4: *limits
  joint_5: *limits
  joint_6: *limits
```

custom 中六轴速度、加速度必须完整、有限且为正；它们仍与硬件和 URDF 上限取较小值。
该 YAML 只选择动态参数，位置范围由 `position_limits` 单独控制。
规划请求中的 velocity/acceleration scaling 是无单位比例；RViz 设为 1.0 才能使用上述完整动态限值。

## 启动

先在控制进程退出时，确认机械臂处于文档折叠入口、身份和无故障状态。
在项目 Docker 环境中构建最新 controller、hardware、bringup、moveit_config 后，使用监督入口：

```bash
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.moveit_deployment.local.yaml \
  enable_execution:=true position_limits:=hardware dynamics_limits:=custom \
  planning_limits_file:=/workspaces/hex_arm_ros2/src/hex_arm_moveit_config/config/joint_limits_deployment.yaml \
  align_folded:=true allow_enable_transient:=true
```

无界面场景追加 `use_rviz:=false`。测试容器内使用的 ROS_DOMAIN_ID 为 189；启动、验收、停止客户端必须相同。
真机 RViz 现在会在启动验收完成后自动打开。启动动作和 10 秒 ready 静止验收期间，
MoveIt 的 `/move_action` 与 `/execute_trajectory` 尚未开放；严格碰撞检查服务保持可用。
验收通过后才开放执行，并校验本次启动报告、自动启动 RViz，避免提前执行新轨迹打断静止验收。
无界面客户端应等待 `MoveIt startup verified: execution available`，
并保存 `startup_ready reached and verified` 打印的 **本次** `startup-ready.json` 路径。
使能瞬态许可、重力渐入、静止保持和完整 J2→J4→J3 顺序仍是强制前置检查。

若启动失败后看到 `context is invalid` 等 ROS/RViz 退出日志，先查看 `startup-ready.json` 的
`passed/error` 与 `driver-shutdown.json` 的 `disabled_confirmed`。监督入口会分别报告启动失败与
电机失能结果，失能成功不会把失败的启动变成成功。
2026-09-30 的 RViz 提前执行竞态及修复验证见[启动门控记录](commissioning_evidence/2026-09-30-moveit-startup-gate.md)。

## 受控停止与完整小步验收

保持监督启动进程运行，在同一容器、同一 ROS_DOMAIN_ID 的另一个终端中执行：

```bash
source install/setup.bash
python3 scripts/commission-meow-ros.py --allow-motion \
  --profile config/hardware/firefly_y6.meow.can2.moveit_deployment.local.yaml \
  --startup-report /tmp/本次运行目录/startup-ready.json \
  --output /tmp/meow-stop-result.json
```

将示例路径替换为本次启动打印的完整路径；不要使用旧运行的报告。
此工具核对配置 SHA256、成功启动步骤、真实硬件与新鲜反馈，必要时严格规划回 ready，
再检查固定回折路径，按 J3→J4→J2 使用本次 MoveIt 的正常速度、加速度上限回折，最后停止控制器并确认硬件 INACTIVE。
开机展开、准备对齐和回折的时长均按各轴行程自动计算，正式部署使用 1.256637 rad/s、0.6 rad/s²；
返回 ready 使用 1.0/1.0 缩放。启动报告保存本次动态限值，关机时继续使用相同限值。
更新后重新构建并重新启动工程；旧启动报告不再授权新的回折流程。开机后的 10 秒静止验收仍保留。
出现失败会取消目标并请求失能，报告中的 `passed` 不会伪装为成功。
同一时间只使用一个运动客户端；运行本工具期间不要在 RViz 或其他终端发送轨迹。

添加 `--test-moveit` 会先执行六轴逐轴小步往返、10 秒保持，再回折并失能。
六轴偏移分别为 `[-0.02,+0.015,-0.015,-0.015,-0.02,+0.015]` rad，每次均返回 ready。
所有 MoveIt 轨迹维持严格碰撞检查；只有固定折叠路径允许文档中已有的两个模型接触对。
`--campaign` 和 `--test-moveit` 的速度、加速度比例默认均为 1.0，可通过
`--velocity-scaling` / `--acceleration-scaling` 单独调整；接受 (0,1] 的有限值。
报告记录所用比例、规划时长及逐轴规划速度/加速度峰值。固定展开和回折使用已验证的独立时序。

工具确认控制器停止、ROS 硬件 INACTIVE 后，再退出监督启动进程。
还须检查 `driver-shutdown.json` 为 `disabled_confirmed`，并在控制进程退出后独立读取六轴模式/故障；
ROS INACTIVE 本身不能证明电机已失能。若 CAN 断链且回执为 `disable_unconfirmed`，应支稳机械臂并关闭动力电源。
当前配置没有启用旧臂的 `shutdown_damping`。监督入口新增了正常 Ctrl+C 的 Meow 回折分支：
仅本次启动成功且仍保持时调用同一受控回折工具，再退出驱动；TERM/HUP、二次中断和异常退出
仍立即进入失能流程。该自动分支已通过软件测试；本轮验证了验收工具回折失能后的正常
Ctrl+C 退出。运行中直接由 Ctrl+C 发起完整自动回折尚未单独验收。
显式回折工具的基线闭环已通过；故障停机仍可能因重力回落。

## 换臂

重新核对电机身份、方向、零偏、负载、力矩换算及摩擦数据；逐阶段验收后再批准该臂配置。
不要直接复制本臂的零偏、手动摩擦前馈或验收标志。

## 自定义扩大行程清单

`commission-meow-ros.py --campaign /path/to/campaign.json` 可在本次成功启动后执行
逐轴或多轴目标，再回到 ready、受控回折并失能。该参数与 `--test-moveit` 互斥。
清单绑定启动时使用的硬件 profile SHA256；修改位置窗口、增益或重力参数后需要生成新清单。

```json
{
  "schema_version": 1,
  "profile_sha256": "所选硬件配置文件的 SHA256",
  "goals": [
    {
      "label": "shoulder_out",
      "position_rad": [0, -1.29, 1.43, -0.3, 0, 0],
      "hold_sec": 2
    }
  ]
}
```

每个目标必须在所选 profile 内；上述示例要求 J2 窗口已包含 −1.29 rad。
清单接受 1～100 个六轴绝对目标，每个目标保持 0.5～60 秒；末项不是 ready 时自动添加回 ready。
首先检查全部目标的严格碰撞有效性，再逐段规划，并在发送执行前检查整条规划轨迹的位置、
速度、加速度和碰撞。执行截止时间按规划时长加 10 秒确定，最少 30 秒，规划时长最多 120 秒。
到位后检查连续静止和保持；失败取消动作并请求失能，不继续后续目标。
报告保存每个目标的实际位置、保持误差/速度、执行结果、原始反馈和回折步骤。
这些结果验证指定的离散路径与姿态，不能将配置窗口直接解释为整个六维空间已通过。

## 本日扩大范围与候选参数

以 ready `[0,-1.35,1.43,-0.3,0,0]` 为参考，`expanded-kd30-20260930` 配置通过了
0.06、0.12、0.24 rad 三档的 54 段逐轴往返：J1/J5/J6 双向，J2 正向，J3/J4 负向。
这些动作在当时的 0.01 rad 保持误差门槛下通过；随后第一组组合姿态及返回也通过。
第二组组合姿态的轨迹执行成功，但保持期间 J4 速度约 0.0575 rad/s 触发 0.02 rad/s 检查，
整轮失败并失能，独立 CAN 回读确认六轴静止、无故障。不能将这轮标为完整扩大部署通过。

配置位于 `config/hardware/`，共同前缀 `firefly_y6.meow.can2.`：

| 后缀 | 参数变化 | 验证状态 |
|---|---|---|
| `replacement.local.yaml` | 原小范围配置 | 本日完整 12 段往返、保持、回折和失能确认通过 |
| `expanded-kd30-20260930.local.yaml` | 扩大窗口；J1 Kd=30，其余 Kd=15 | 上述 0.24 rad 离散往返通过，组合保持未全部通过 |
| `travel048-20260930.local.yaml` | 进一步扩大窗口；J1/J2/J4 Kd=30；温限 85°C | 候选，最终启动 CAN 故障，0.48 rad 尚未实机执行 |

三个配置的原有零偏、方向、重力比例与限幅、运动前馈均保持原值。
后两个配置中的 `calibrated: true` 用于用户授权的监督调试准入，不表示整个新增窗口通过验收。
候选清单已按用户减少重复测试的要求缩减为四个大幅度组合目标，离线严格端点检查通过。

温度门槛现在可在本臂 profile 中显式配置：

```yaml
controller:
  max_measured_temperature_c: 85.0
```

省略时仍为 70°C；接受有限的 40～100°C 配置，非有限温度反馈始终触发保护。
此参数作用于生产控制器的电机/驱动温度保护，不改写固件内部保护设置。

早前扩大窗口的检查为 Python 107 项及 Rust 温度相关 9 项通过；上述候选仍保留自己的验收状态。
本次正式部署检查汇总为 ROS 302 项通过、Rust fmt/clippy 及测试通过，五个部署包构建成功。
三个组合目标、返回 ready、受控回折和最终失能的真机结果见[本次验收记录](commissioning_evidence/2026-09-30-can2-moveit-deployment.md)。
