# Meow MIT：当前实机部署说明

当前状态、已采用参数和后续优化统一维护在 [实机部署进度](commissioning_cn.md)。
本页说明现有控制协议与配置入口，不保留历次构建日志和试验流水。

## 电机控制

当前 Rust 驱动以 500 Hz 向六轴发送 MIT 混合控制目标。
每轴通过一个原子的 20 字节 CAN-FD RPDO 传递位置、速度、前馈力矩、Kp、Kd 和 PD 限幅。
电机根据位置/速度误差与前馈产生力矩，Rust 根据实测姿态计算重力补偿并执行主机侧限制。

| 对象 | 用途 |
|---|---|
| `0x4401` / `0x4402` | 模式命令 / 显示；Disable=0，MIT=4 |
| `0x4102` | 未压缩 MIT 目标 |
| `0x4103:01` | 压缩目标开关；当前关闭 |
| `0x4102:07` | 从电机读取的增益换算因子 |
| `0x4564` | Q8.24 圈数位置反馈 |
| `0x4576` | 峰值力矩，Nm |
| `0x4572` | 总输出上限，峰值千分比 |
| `0x453F` | 详细错误码 |

ROS 使用 rad、rad/s、Nm、Nm/rad、Nm·s/rad；电机位置/速度使用 Rev、Rev/s。
方向与零偏换算统一在 Rust 层完成。增益因子与前馈力矩校准分别读取和处理。
总输出限幅通过 `0x4572` 设置；MIT 数据包最后的保留字段不用于设置总输出限制。
当前无有效出厂力矩记录时采用与上位机一致的 unity fallback，不写入或重做出厂标定。

## 本机运行

使用现有 `config/hardware/firefly_y6.meow.can2.local.yaml`。
该文件保留本机身份、零偏、重力系数、增益、力矩权限、位置窗口及 CAN 适配器信息。
`firefly_y6.meow_mit.example.yaml` 是其他设备建立配置的模板，不能直接替代本机标定。

```bash
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.local.yaml \
  enable_execution:=true
```

每次断电后重新上电，并在执行自动启动前，先摆放到折叠入口
`[0,-1.570,1.570,0,0,0]` rad，即 J2=−1.570、J3=1.570。该入口先完成控制器配置与
通信准备并核对绝对编码器反馈，再使能并顺序移动 J2→−1.350、J4→−0.300、
J3→1.430；最终安全启动姿态为 `[0,-1.350,1.430,-0.300,0,0]` rad。
等待启动完成日志后使用 MoveIt。权威数值见
[`startup.yaml`](../src/hex_arm_controller/config/startup.yaml)。
本机已配置退出阻尼：首次 Ctrl-C 会先回安全启动位、再阻尼下落，保持供电，
等待柔和阶段结果及 supervisor 的失能确认。再次 Ctrl-C 请求立即停机。
新增参数尚待实机验收，入口及异常行为见[退出阻尼](shutdown_damping_cn.md)。完整启动前提与参数见
[启动与停止](commissioning_cn.md#日常启动与停止)。

## 更换 CAN 接口

接口可配置，并用 USB 序列号和物理通道核对设备。控制停止且新接口已按本机
1 Mbps 仲裁 / 4 Mbps 数据速率配置后，例如从 can2 换到 can0：

```bash
python3 scripts/bind-can-profile.py \
  --interface can0 \
  --profile config/hardware/firefly_y6.meow.can2.local.yaml \
  --output config/hardware/firefly_y6.selected.local.yaml
HEX_ARM_CAN_IFACE=can0 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.selected.local.yaml \
  enable_execution:=true
```

绑定工具只读取 sysfs，并生成新配置，保留电机身份与标定；输出文件必须尚不存在。
没有运行中热切换总线。上位机和 ROS 控制端先后独占总线。

## 继续开发

先停止真机控制，在容器工作区运行 `./scripts/build.sh`，然后 `source install/setup.bash`。
协议、模拟控制与真机的验证分别执行；离线通过后再在当前运动窗口做实机回归。
控制架构见 [README](../README_cn.md#控制接口与架构)，具体优化顺序见
[后续优化](commissioning_cn.md#后续优化顺序)。
