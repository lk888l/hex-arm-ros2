# 从默认断电摆放位置校准软件零点

`scripts/calibrate-zero.py` 用于旧版 **CiA402 固件 + SocketCAN** 的 Firefly Y6。
更换机械臂、重新校准电机或调整编码器安装后，可读取当前机械臂的数据，重新计算
六轴 `zero_offset_rad`，生成独立的本机配置。依赖 Python 3 和 PyYAML；可在宿主机
或能访问 SocketCAN 的开发容器中执行，无需启动 ROS。

参考姿态直接读取 [startup.yaml](../src/hex_arm_controller/config/startup.yaml)
的 `folded_position_rad`，当前为 **`[0, -1.570, 1.570, 0, 0, 0]` rad**。
这里沿用工程的 `1.570`，不替换为 `π/2`。使用当前 schema v3、关节坐标版本 2，
J3 已居中；旧 schema v2 配置会被拒绝，避免重复减去 J3 的 `1.57`。

## 单独核对关节方向

一个折叠参考只能计算零偏，不能证明 `direction` 正确。更换机械臂或固件后，
应将独立观察到的实物运动方向与原始编码器变化比较，不能用“命令负向、编码器也负向”
循环论证 URDF 方向。旧版 CiA402 的失能路径保留 `0x2040=1` 短接制动，
通电失能时的手动阻力也不能直接当作机械卡死；不要强行反拖或随意关闭制动。

`scripts/check-joint-direction.py` 只离线比较两份 `read-cia402-state.py` 的静止失能快照。
操作者必须独立确认实物运动对应 URDF 的正/负方向，并用合适的支撑建立两个参考位置。
输入必须是同一条臂、同一 CAN 接口，选中轴的原始编码器位移在 0.02～0.35 rad，
其余轴不超过 0.01 rad；跨单圈边界、身份变化、使能、故障和非静止数据均拒绝。

```bash
python3 scripts/check-joint-direction.py --profile /path/to/arm.local.yaml \
  --before /path/to/before.json --after /path/to/after.json \
  --axis joint_3 --expected-sign -1 --output /path/to/direction-result.json
```

`--expected-sign` 来自实物与 URDF 的独立对应关系，不来自当前 profile 的符号。
工具输出推断符号、当前符号是否一致及输入哈希，不打开 CAN、不修改配置或标定标志。
若方向需要修正，应回到已确认折叠参考，重新计算零偏并重新验收；不能只改符号沿用旧偏置。

## 准备参考位置

1. 结束 ROS 真机控制、调试工具及其他占用 CAN 的控制程序。
2. 将机械臂按已确认的实物参考摆放到默认断电折叠位置，尤其检查 J1/J4/J5/J6 的朝向。
   使用可靠的标记、定位夹具或测量依据；单凭“看起来折叠了”不能确定六轴角度。
3. 保持机械臂静止，接通读取编码器所需电源，六轴电机保持失能。

`--confirm-folded-pose` 表示操作者确认了上述实物参考。工具不能从一个未知零点的
编码器读数推断真实姿态，也不会根据上一台机械臂的编码器快照判断姿态。
默认采样 21 轮，逐轮读取六轴位置、状态字和故障码；最后再检查状态与身份。
每轴取中位数，最大允许采样跨度默认为 `0.001 rad`。使能、故障、超时、移动、
单圈跳变及身份不匹配均导致失败，不生成配置。

## 预览计算结果

仓库根目录执行，`--profile` 换成作为起点的本机 CiA402 配置：

```bash
python3 scripts/calibrate-zero.py --interface can2 \
  --profile config/hardware/firefly_y6.cia402.can2.survey.local.yaml
```

默认只打印结果。未传 `--confirm-folded-pose` 时，输出明确标为假定折叠参考下的
计算结果。可以用 `--report /tmp/zero-preview.json` 保存 JSON；文件必须尚不存在。

| 输出列 | 含义 |
|---|---|
| `Raw Rev` | 电机 `0x6064` 编码器实测值，中位数，单位圈 |
| `Before rad` | 按所选方向和原有 `zero_offset_rad` 换算的关节角度 |
| `zero_offset_rad` | 此次根据折叠参考计算的新偏置 |
| `After rad` | 用同一实测编码器值和新偏置换算的关节角度 |
| `Span rad` | 采样最大值减最小值，换算为 rad |

换算与 Rust 控制器保持一致：

```text
q_rad           = direction × 2π × encoder_rev + zero_offset_rad
zero_offset_rad = folded_position_rad − direction × 2π × median(encoder_rev)
```

`Raw Rev` 是传感器测量；`Before/After` 是软件坐标。`After` 在这批样本上对齐参考值
是计算本身的结果，不是独立测量证明。机械摆放误差会成为零偏误差；小采样跨度只说明
读数稳定，不能证明摆放准确。单一静止姿态也无法识别 `direction` 是否正确。

## 保存新配置

更换机械臂时，显式使用 `--replace-arm` 将当前读到的四项电机身份写入新配置。
重校准原机械臂且身份未变时，可以省略此选项。

```bash
python3 scripts/calibrate-zero.py --interface can2 \
  --profile config/hardware/firefly_y6.cia402.can2.survey.local.yaml \
  --replace-arm --confirm-folded-pose \
  --output config/hardware/firefly_y6.cia402.can2.zero-calibrated.local.yaml
```

生成的文件包括：

- 新的 `*.local.yaml`：六轴零偏、当前电机身份和从 sysfs 读取的 CAN 适配器绑定。
- `<output>.calibration.json`：原始样本、状态、身份比较、前后坐标、偏置变化、限位检查、
  时间和输入/参考/输出文件 SHA256。可用 `--report` 指定其他路径。

输入配置和已有输出文件受到覆盖保护。再次校准请使用新文件名。
工具仅发送 SDO upload，不写电机零点、不修改固件、不使能或移动机械臂。
即使电机身份编号和以前相同，更换机械结构或重新校准后仍应重新建立参考；身份匹配
无法证明机械安装和编码器基准未变。

如果没有可复用的 schema v3 配置，可以从通用模板开始，并明确给出已确认的方向。
下面的方向是本工程此前使用的排列，更换接线、安装或型号后需核实：

```bash
python3 scripts/calibrate-zero.py --interface can2 \
  --profile config/hardware/firefly_y6.example.yaml \
  --directions -1 -1 1 1 1 1 \
  --replace-arm --confirm-folded-pose \
  --output config/hardware/firefly_y6.new-arm.local.yaml
```

模板中的 node 0 仅在 `direct_joint_mapping: true` 时按 joint_N → node N 展开。
模板的力矩、增益等占位参数仍需完成实机配置；电机产品型号文字无法确认时明确标注
`model unverified`，不会沿用其他产品的旧名称。

## 重新读取，检查已有偏置

保持或重新摆到同一实物参考位置，用新配置执行一次独立采样：

```bash
python3 scripts/calibrate-zero.py --interface can2 \
  --profile config/hardware/firefly_y6.cia402.can2.zero-calibrated.local.yaml \
  --confirm-folded-pose --check --tolerance-rad 0.005 \
  --report /tmp/zero-check.json
```

此时表格显示 `Current rad`、`Existing offset` 和 `Ref error rad`。
检查使用文件中已有的偏置；任一轴与参考相差超过 `0.005 rad`（约 `0.29°`）即失败。
检查不自动改写偏置，也不接受 `--replace-arm` 或 `--directions` 来掩盖变化。
CAN 接口、适配器和六轴身份必须匹配。普通启动前如需用这个命令核对零点，也必须先
建立同一实物参考，任意姿态下不能使用折叠姿态检查。

退出码：`0` 表示当前操作通过，`1` 表示采样/检查/保存失败，`2` 表示参数或输入配置错误。
JSON 的 `passed` 只针对 `mode` 所示操作：预览通过不表示姿态已确认；零偏检查通过
不表示全行程验收通过。采样或检查失败时，指定的 JSON 仍保存已获取的数据和错误。

## 与真机部署和限位的关系

生成的配置固定为 **`validated: false`、`calibrated: false`**。这两个标志控制整套配置
和六轴运行的准入，单个参考姿态的零偏计算不能代替方向、运动范围、静态保持、
跟踪、重力及末端负载验证。

工具保留输入的关节角限位和调参值，并报告两类兼容问题：折叠参考不在沿用窗口内，
或新偏置将窗口映射到单圈编码器的边界保护区。JSON 同时给出新偏置对应的原始 Rev
上下界，负方向轴的两端会正确排序。检查窗口使用控制器当前的 `0.01 Rev` 边界余量。

历史 survey 中 J6 的 `2.06..2.16 rad` 是上一轮局部试验窗口；新参考 J6=0 时会提示
不兼容。应针对当前机械臂重新确定部署窗口。一次折叠采样无法测定完整机械行程，
也不会自动扩大或平移限位。完成窗口及其他参数验证后，再按
[CiA402 部署流程](cia402_deployment_cn.md)完成配置准入和小范围动作验证。

离线回归测试：

```bash
python3 scripts/test-calibrate-zero.py
```

测试涵盖双向换算、当前 J3 坐标、固件 f32 换算精度、只发送 SDO upload、使能/故障/
缺失节点/抖动/单圈跳变的拒绝、新旧身份处理、输出保护及独立复查发现零点漂移。

本机实际校准与复查结果见
[2026-09-29 can2 零点校准记录](commissioning_evidence/2026-09-29-can2-zero-calibration.md)。
