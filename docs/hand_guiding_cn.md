# 手扶拖动：重力补偿与阻尼

[English](hand_guiding.md) | **中文** · [返回 README](../README_cn.md#hand-guiding)

以下命令在已构建并加载 ROS 环境的容器工作区 `/workspaces/hex_arm_ros2` 中执行。

专用 `gravity_comp.launch.py` 从当前实测姿态进入手扶拖动：`Kp=0`、目标速度为零，
输出为经过 profile 比例及限幅处理的重力矩，加上 `−Kd × 实测速度` 阻尼。
它不执行折叠展开、位置保持或轨迹跟踪，也不启动 MoveIt 或轨迹控制器；ros2_control 的硬件保持 INACTIVE，仅通过 C++ 直连提供反馈和诊断。
松手后阻尼会使运动减速，但不会锁定位置；重力模型、载荷和补偿比例有误差时仍会漂移。

## 启动

先停止其他机械臂控制程序。在容器工作区构建并加载新入口：

```bash
./scripts/build.sh --packages-up-to hex_arm_bringup
source install/setup.bash
ros2 launch hex_arm_bringup gravity_comp.launch.py \
  hardware_profile:=/workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.hand_guiding_full_range.local.yaml \
  activate_hardware:=true \
  damping:='[2.0, 2.0, 2.0, 2.0, 2.0, 2.0]'
```

## 本臂配置与范围

当前臂使用 `replacement.local.yaml`，其适配器及六轴身份与当前 can2 实读值匹配；
`firefly_y6.meow.can2.local.yaml` 是旧臂配置。只重绑 CAN 适配器不会迁移电机身份或零位标定。
上面的 `hand_guiding_full_range.local.yaml` 基于本臂配置，将六轴位置限位扩大到 URDF 范围，
并使用独立的六轴手扶速度保护门槛 2.0 rad/s：
J1 `[-2.86, 2.86]`、J2 `[-1.57, 2.09]`、J3/J4 `[-1.57, 1.57]`、
J5 `[-1.54, 1.54]`、J6 `[-2.79, 2.79]` rad。原 `replacement.local.yaml` 保留窄窗口。
新配置属于用户授权的全范围拖动调试候选；继承的标定标志不代表新增范围已经实机验收。
单轴模型限位不能保证关节组合无碰撞，也不等于已核实的机械硬限位。
六轴重力补偿比例均为 1.0，力矩上限沿用原试调值，大幅改变姿态后可能支撑不足，
需要持续扶稳，不能把松手悬停作为已实现的保证。配置文件直接加载，无需重新编译。

## 阻尼参数

`damping` 顺序为 J1～J6，单位为 N·m·s/rad，必须为六个有限正数。
默认各轴为 2.0；增大时松手减速更强，拖动也更费力，减小时阻力和松手减速能力一起降低。
改变参数需要先停止，再重新启动。持续漂移应核对重力模型、安装方向、末端载荷及每轴
`gravity_compensation_scale`；原先配合位置 PD 使用的补偿比例不一定能实现零刚度悬停。
不要通过把现有轨迹控制的 `default_kp` 改为零来启用这个功能。

## 进入条件与保护

进入模式需要已验证、已标定的 schema v3 profile、六轴新鲜反馈、关节在限位内，
且各轴实测速度不超过 0.02 rad/s。使能前及重力渐入期间应手扶支撑，等待日志
`hand_guiding_ready` 后再拖动。渐入速率沿用 `gravity_startup_slew_rate_nm_s`，未配置时为
5 N·m/s；渐入后重力补偿直接跟随实测姿态。限位、速度、力矩和温度保护保持生效，
拖动专用 profile 设置 `controller.hand_guiding_velocity_limits_rad_s` 为六轴各 2.0 rad/s
（约 114.6°/s），仅在 `GRAVITY_COMP` 模式中替代普通实测速度门槛；该数组是绝对门槛，
不额外叠加 J2/J4 的速度余量。运动命令和其他模式仍使用 `joints[].limits.velocity_rad_s`。
省略此参数时，手扶模式沿用每轴命令速度加实测余量；数组顺序为 J1～J6，必须包含六个
有限正数且不超过 6 rad/s。阻尼不能保证速度受限，因此仍保留超速保护和非有限反馈拒绝。
超速会故障失能；这不是位置锁定或行程不足。
`controller.hand_guiding_position_margin_rad` 在本拖动配置中为 0.012 rad（约 0.69°），
只在进入/运行 `GRAVITY_COMP` 时替代普通位置反馈余量，其他模式及外部位置命令保持严格限位。
未配置时沿用各轴 `measured_position_margin_rad`；该参数接受有限的 0～0.012 rad，
并要求反馈范围小于一整圈，以满足现有使能位置一致性检查及单圈坐标约束。
反馈处于余量内时，仅把零刚度 MIT 位置目标字段限制到合法命令范围；`Kp` 仍为零。
重力计算始终使用实际编码器角度，不对角度截断；超过反馈余量或非有限读数仍触发保护。
J2 在折叠位置接近模型下限 −1.57 rad，只能向范围内拖动。
本模式没有 MoveIt 碰撞检查，需要在有间隙的工作范围内操作。

## 停止

停止前先扶稳机械臂，再从另一个已 source 的终端执行：

```bash
ros2 service call /hex_arm_gravity_comp/stop std_srvs/srv/Trigger '{}'
```

服务成功表示所有轴已确认失能且会话已释放，整组 launch 随后退出。
Ctrl-C 同样触发停止，但不会自动回位或折叠；失能后机械臂需要外部支撑。
管理节点每 50 ms 续期，驱动在 500 ms 未收到有效续期后锁存故障并失能，
失败时继续重试失能。管理进程崩溃、冻结以及通信中断都不会留下无限期的补偿输出。
故障后不自动恢复；支撑机械臂并停止该 launch，查明原因后重新启动。
驱动退出确认写在当前 ROS 日志目录的 `hand-guiding-shutdown.json` 中，
只有 `disabled_confirmed` 表示确认失能。

## 观察与模拟

省略 `activate_hardware:=true` 时只观察状态；`use_rviz:=true` 可打开姿态显示。
不接机械臂时可先验证软件链路（mock 不模拟实际重力或拖动手感）：

```bash
ros2 launch hex_arm_bringup gravity_comp.launch.py \
  hardware_profile:=/workspaces/hex_arm_ros2/src/hex_arm_controller/test/firefly_y6.mock.yaml \
  mock:=true activate_hardware:=true
```

