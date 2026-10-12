# 新版 Meow 固件的 J2～J5 软件零点校准

`scripts/calibrate-zero.py` 使用新版 **Meow 固件 + SocketCAN**，仅计算、保存和复查
**J2、J3、J4、J5** 的 `zero_offset_rad`。**J1/J6 不参与软件校准**，其方向、零偏和
限位沿用输入配置；无需为本次校准把 J1/J6 对齐到零位。
工具依赖 Python 3 和 PyYAML，可在宿主机或能访问 SocketCAN 的开发容器中运行，无需 ROS。

默认固件、通用模板和运行入口均为 Meow。校准拒绝显式的旧版 CiA402 profile，
使用 schema v3 / 关节坐标版本 2，不进行旧 J3 坐标迁移。

## 参考与读取方式

参考来自 [startup.yaml](../src/hex_arm_controller/config/startup.yaml) 的
`folded_position_rad`，取其中 **J2～J5 = `[-1.570, 1.570, 0, 0]` rad**。
沿用工程的 `1.570`，不替换为 `π/2`。

1. 结束 ROS 真机控制、GUI MIT 和其他占用 CAN 的控制程序。
2. 根据可靠的标记、定位夹具或测量依据，将 J2～J5 摆放到折叠参考位置。
3. 保持机械臂静止，接通编码器读取电源，六轴电机保持失能。

`--confirm-folded-pose` 表示操作者已确认 J2～J5 的实物参考。单次编码器读数不能
确定未知零点的真实姿态；上一台机械臂的读数也不能作为当前机械臂的参考。

默认采样 21 轮，只读取 J2～J5 的 `0x4564` **有符号 Q8.24 多圈位置**，单位圈。
每轮同时读取六轴 `0x4402` 模式显示和 `0x453F` 故障码，要求均为 0；
采样前后核对六轴 `0x1018` 身份。J1/J6 仅参与这些状态与身份检查，不采样其位置。
四个校准轴分别取中位数，最大允许跨度默认为 `0.001 rad`。
使能、故障、超时、移动或身份不匹配会导致失败，不生成候选配置。

## 预览

在仓库根目录执行，使用当前机械臂的 Meow 本机配置：

```bash
python3 scripts/calibrate-zero.py --interface can2 \
  --profile config/hardware/firefly_y6.meow.can2.local.yaml
```

默认只打印 J2～J5 的结果；未确认实物参考时标注为假定参考下的计算值。
可用 `--report /tmp/zero-preview.json` 保存 JSON，目标文件必须尚不存在。

| 输出列 | 含义 |
|---|---|
| `Raw Rev` | `0x4564` Q8.24 解码后的实测中位数，单位圈 |
| `Before rad` | 按输入方向和旧零偏换算的关节角度 |
| `zero_offset_rad` | 根据本次 J2～J5 折叠参考计算的新零偏 |
| `After rad` | 同一读数使用新零偏换算的关节角度 |
| `Span rad` | 采样最大值减最小值，换算为 rad |

换算与 Rust 驱动一致：

```text
encoder_rev     = signed_i32(0x4564) / 2^24
q_rad           = direction × 2π × encoder_rev + zero_offset_rad
zero_offset_rad = folded_position_rad − direction × 2π × median(encoder_rev)
```

`After` 对齐参考是计算结果，不是独立实测证明。摆放误差会进入零偏；读数稳定不能
证明摆放准确，一个静止姿态也不能判断 `direction` 是否正确。

## 保存新配置

```bash
python3 scripts/calibrate-zero.py --interface can2 \
  --profile config/hardware/firefly_y6.meow.can2.local.yaml \
  --confirm-folded-pose \
  --output config/hardware/firefly_y6.meow.can2.zero-calibrated.local.yaml
```

更换机械臂时加 `--replace-arm`，允许把六轴身份绑定为当前观察值。
这不会重新计算 J1/J6 的方向或零偏。需要先在输入配置中核实这两个轴的既有坐标参数。
没有本机配置时，可从 `config/hardware/firefly_y6.example.yaml` 或
`firefly_y6.meow_mit.example.yaml` 开始，核实模板中的安装参数，再使用 `--replace-arm`。

`--directions` 只接受 **四个** 已独立核实的符号，依次对应 **J2 J3 J4 J5**，
例如 `--directions -1 1 1 1`。省略时保留输入方向，J1/J6 始终不受此参数影响。
离线 `import-gui-mit-profile.py` 的 `--park-positions-rev` 同样只接受 M2 M3 M4 M5
四个读数并保留 J1/J6 的坐标参数。

生成文件包括：

- 新的 `*.local.yaml`：更新 J2～J5 零偏、六轴身份及 sysfs 中的 CAN 适配器绑定。
- `<output>.calibration.json`：原始读数、六轴状态与身份、四轴校准结果、限位提示及输入/输出哈希。
  报告 `schema_version: 2`，明确列出 `calibration_joints` 和 `preserved_joints`。

已有文件受到覆盖保护。工具仅发送 SDO upload，不写入电机零点、不刷固件、不使能或移动机械臂。
生成配置仍为 **`validated: false`、`calibrated: false`**，需要完成本臂的部署验证后使用。

## 独立复查

保持或重新建立相同的 J2～J5 实物参考，用候选配置重新采样：

```bash
python3 scripts/calibrate-zero.py --interface can2 \
  --profile config/hardware/firefly_y6.meow.can2.zero-calibrated.local.yaml \
  --confirm-folded-pose --check --tolerance-rad 0.005 \
  --report /tmp/zero-check.json
```

只比较 **J2～J5** 已有零偏对应的参考误差；任一校准轴超过 `0.005 rad` 即失败。
J1/J6 的当前角度不参与判定。接口、适配器和六轴身份仍必须匹配。
检查不会修改零偏，也不接受 `--replace-arm` 或 `--directions`。

退出码：`0` 当前操作通过，`1` 采样/检查/保存失败，`2` 参数或配置错误。
失败时仍保存指定的 JSON 证据；`passed` 仅对应本次操作，不代表全行程和部署验收通过。

## 方向与限位

`scripts/check-joint-direction.py` 离线比较两份新版 `read-meow-state.py` 生成的 Meow
静止失能快照。先用同一 profile 分别执行下列读取，独立观察并建立两个位置：

```bash
python3 scripts/read-meow-state.py --interface can2 --profile /path/to/arm.local.yaml \
  --output /path/to/before.json
# 建立第二个已独立确认运动方向的参考位置后，以 --output /path/to/after.json 重新读取。
python3 scripts/check-joint-direction.py --profile /path/to/arm.local.yaml \
  --before /path/to/before.json --after /path/to/after.json \
  --axis joint_3 --expected-sign -1 --output /path/to/direction-result.json
```

`--expected-sign` 来自实物与 URDF 的独立对应关系。选中轴位移须为 0.02～0.35 rad，
其余轴不超过 0.01 rad；工具不会改写任何方向或零偏。

校准保留输入限位与调参值，提示折叠参考超出窗口或新零偏对应的电机目标超出
Meow **`[-128, 128)` Rev** 表示范围。多圈位置不使用旧 CiA402 的 ±0.5 Rev 单圈保护区。
一次折叠采样不能确定完整机械行程，也不会扩大限位。部署流程见
[Meow 部署说明](meow_mit_deployment_cn.md)。

离线回归：

```bash
python3 scripts/test-read-meow-state.py
python3 scripts/test-calibrate-zero.py
python3 scripts/test-check-joint-direction.py
python3 scripts/test-import-gui-mit-profile.py
```

旧版 CiA402 的历史结果保留在
[2026-09-29 校准记录](commissioning_evidence/2026-09-29-can2-zero-calibration.md)，不作为新版 Meow 的校准流程。
