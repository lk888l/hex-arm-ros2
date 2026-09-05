# MoveIt 图形化 mock 仿真

[English](moveit_simulation.md) | **中文**

此流程会验证 MoveIt、OMPL、碰撞检测、逆运动学、`FollowJointTrajectory`
以及现有的 ros2_control `GenericSystem`。它不会访问 USB/CAN 硬件，也不是物理仿真。

## 本地 Ubuntu 24.04 启动（当前机器）

先在 Ubuntu 桌面的宿主机终端中启动容器并检查图形转发：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

进入提示符主机名为 `hex-arm-dev` 的 `ros2-jazzy-arm` 容器后执行：

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh  # 首次使用或源码改动后执行
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

最后一条命令会启动 `move_group` 和 RViz，正常情况下 Ubuntu 桌面会弹出 MoveIt
界面。

## WSL2 启动（仅 Windows 10/11）

WSL2 是 **Windows Subsystem for Linux 2**：Ubuntu 运行在 Windows 内部，图形
窗口由 WSLg 显示。在 Windows 的 Ubuntu/WSL 终端（不是 PowerShell）中执行：

```bash
cd /home/kk_wsl/ros2_ws/code/hex_arm_ros2  # 仓库不在此处时请替换路径
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

进入同一个 `ros2-jazzy-arm` 容器后，MoveIt 命令与本地 Ubuntu 相同：

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh  # 首次使用或源码改动后执行
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

如果现有 `hex_arm_bringup mock.launch.py` 使用相同的 `ROS_DOMAIN_ID`，请先停止
该会话；两个 launch 文件有意使用相同的控制器名称。如果 RViz 报
`could not connect to display :0`，请在宿主机运行
`./scripts/docker-dev.sh doctor`。脚本会在本地 Ubuntu 检查 Xauthority，在 WSL2
使用 WSLg 配置；不要用 `xhost +` 绕过访问控制。自检通过后重新进入容器再启动
MoveIt。

在 RViz 中选择 `arm` 规划组。拖动交互标记，或选择名为
`commissioning_start` 的状态；先点击 **Plan**，确认预览正确后，仅在 mock 硬件上
点击 **Plan & Execute**。该命名状态是内缩后的规划参考，不是已标定的真机 home。
mock 执行链路如下：

```text
MoveIt RViz -> move_group -> firefly_arm_controller
             -> ros2_control GenericSystem -> /joint_states -> RViz
```

默认 `sim` 配置保留 URDF 中的 `6.0 rad/s` 上限，并使用 0.1 的 MoveIt 默认
缩放系数。无需连接硬件，即可预览首次实机调试所采用的保守低速策略：

```bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py \
  limits_profile:=commissioning
```

可用配置如下：

| 配置 | 每轴最大速度 | 用途 |
|---|---:|---|
| `sim` | 6.0 rad/s | 纯软件图形化测试 |
| `commissioning` | 0.1 rad/s | 首次有人监督的实机调试（另有窄位置窗） |
| `verified` | 6.0 rad/s | 仅在分阶段真机验证完成后使用 |

MoveIt 限位是规划限位，不是真机安全边界。真机启动必须使用已经验证的本地硬件
profile，并保证其中的 `velocity_rad_s` 和 `acceleration_rad_s2` 都不大于所选
MoveIt profile。Rust 控制器已用逐轴三次 Hermite 段替代位置线性插值：流式命令
重定向时位置与速度连续，并解析检查三次曲线的全部速度、加速度极值；请求时间过短
时会自动延长到同时满足硬件 profile 两项限制。加速度可以在限值内不连续（当前不
限制 jerk）；非法、非有限或非正限值会使 profile 加载失败。MoveIt 仍需自己的
加速度限值进行时间参数化：`sim` 的 10.0 rad/s² 仅用于软件可视化，
`commissioning` 为 0.1 rad/s²，`verified` 在实测前暂定为 0.2 rad/s²。

新的 SRDF 没有复用历史上禁用全部 21 对连杆的矩阵。默认严格
`firefly_y6.srdf` 只排除六对运动学相邻连杆，因为它们共享关节/接口几何。实体
机械臂已经安全处于修正后的实测折叠姿态 `q~=[0,-1.570,3.140,0,0,0]` 时，FCL 还会从来源
collision mesh 报告两对接触：

- `link_1` 与 `link_5`；
- `link_2` 与 `link_4`。

这两对不属于严格可执行矩阵。只有真机 MoveIt 在
`enable_execution:=false` 时才会加载单独的 `firefly_y6.plan_only.srdf`，并且只有这份
plan-only 覆盖会为上述两对增加 `PlanOnlySurveyedFold` 例外。mock MoveIt 与
`enable_execution:=true` 的真机 MoveIt 始终保留严格 SRDF，因此所有 15 对非相邻
连杆在任何可执行路径上都会继续检查。它们并非“永远不可能碰撞”：独立的 MoveIt
`collisions_updater` 在完整 URDF 关节范围采样 100,000 个姿态，没有发现任何永久
碰撞的非相邻对。因此，这个 overlay 只是等待重新测量/导出准确碰撞网格前的
可视化/调试补丁，不是物理安全结论。其余十三对非相邻连杆即使在 plan-only 模式下也
仍然启用检查；离线 FCL 回归在此前错误的
`q=[0,+1.570,3.140,0,0,0]` 姿态仍会暴露五对未放宽接触。任何基于 `firefly_y6.plan_only.srdf` 得到的规划都只可
用于预览，不能直接复用为执行轨迹；真机执行前必须在严格 SRDF 下重新规划，并先修正
碰撞几何与实体起始姿态。第一版暂时使用 `link_6` 作为规划末端，直到获得经过标定的
固定 TCP/工具坐标系。旧的越界 `ready` 已删除；`commissioning_start` 贴近观测姿态，
但把 joint_2/joint_3 设为 -1.56/3.13 rad，即分别向临时限位内缩 0.01 rad。它只用于规划参考，绝不能
在未标定机械臂上执行。

## 真机 MoveIt：默认只规划

本地 Ubuntu 宿主机必须先按
[commissioning 清单](commissioning_cn.md)把 profile 选择的 SocketCAN 接口严格
配置为仲裁段 1M/SP0.8/SJW5、数据段 4M/SP0.8/SJW3、FD、`restart-ms 0`，再启动
普通容器：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
```

从宿主机通过受监督辅助命令启动新的真机入口。当前示例为 `can2`、channel 2，两者
都必须与 YAML profile 一致：

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

该命令不会自动使能硬件。收到 `Ctrl-C` 后，宿主会用第二个目标固定的
`docker exec` 校验本次随机 token 对应的 PGID 文件，只向该独立进程组发信号；同时
继续等待原 Compose exec 返回 Rust 控制器“确认失能→解除心跳 consumer”的 clean
exit 结果。应保持连接直到打印验证结果。独立的 `can1` launch 不会收到信号。不要
再把真机 launch 包在裸 `docker exec ... bash -lc` 后面。

默认 `enable_execution:=false`：被包含的 real bringup 保持
`activate_hardware=false`，`move_group` 同时设置
`allow_trajectory_execution=false` 并加载 `firefly_y6.plan_only.srdf`。因此 RViz 可以
显示 `/hex_arm/internal/state` 并规划，但 Execute 不会进入硬件。结构、节点身份和
`expected_link` 已复核的 `validated: true, calibrated: false` profile 可以用于这种
禁用状态观察；激活仍会被明确拒绝。node 1 到 6 直接对应 joint_1 到 joint_6。
node 15 仍完全排除在驱动控制之外，但配置 `tip_payload` 后必须精确匹配其身份，并把
固定质量/质心纳入 `link_6` 重力模型。

当前近似姿态 `q=[0,-1.570,3.140,0,0,0]`、旧方向候选和一次快照拟合的零偏只用于
commissioning 对照，不是 home 或动作验证。修正遗漏的 joint_2 负号后，其完整 URDF
范围处在单圈 seam guard 内；但它仍从下限起步，必须先有界向内重新定位并完成双向
跟踪验证，才可对真机使用 `enable_execution:=true`。即使 profile 已标为 calibrated，
也不能跳过该独立门槛。

未来关闭这些调试项后，执行入口才是：

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  enable_execution:=true
```

`enable_execution:=true` 的真机入口会切回严格 `firefly_y6.srdf`，同时仍固定使用
0.1 rad/s、0.1 rad/s² 以及当前观测姿态附近的窄位置窗。下游
JointTrajectoryController 对六轴统一使用 0.02 rad 路径误差、0.005 rad 目标误差、
0.02 rad/s 停止速度容差和 1.0 s 目标等待时间；因此 joint_2 在目标阶段残留
0.016 rad 误差时 action 必须失败，不能判成功。该加固不表示已可执行：标定、joint_2 跟踪、
跟踪、载荷、TCP 和严格碰撞门槛仍全部锁闭。

桥以空 `kp`/`kd` 委托
Rust 使用逐轴低增益默认值，并以空 `tau_ff` 委托 Rust 自动计算重力前馈；显式六个
零的 `tau_ff` 含义不同，会关闭自动前馈。委托计算的重力会逐轴乘以硬件 profile 中的
`gravity_compensation_scale`，与单轴 commissioning 完全一致；它是真机辨识参数，
不是电机力矩换算使用的 `torque_scale`。在逐轴缩放之前，schema v2 必填的
`gravity_vector_base_m_s2` 会提供 URDF `base_link` 坐标系下的重力向量。真机
launch 会拒绝 v1 或缺少该字段的 profile；修正 joint_2 漏写的负号后，当前本地
`-Z` 值仍只是未标定的 commissioning 候选。运行期 `SetGravity` 只在当前独占会话内覆盖，release、shutdown
或新 acquire 都会恢复 profile 值。任何非空外部 `tau_ff` 都会绕过自动载荷模型和该
重力缩放（电机力矩换算仍然生效）；ROS 桥使用空数组委托路径。本地 GR80 的 0.41 kg
质量/质心来自 trial URDF，且保持 `inertial_calibrated: false`，因此 calibrated
profile 会无效并阻止真机 MoveIt 执行。真实 TCP、已标定工具惯量、夹爪变换和最终
碰撞几何仍待完成，`link_6` 只是临时法兰 tip。该 launch 的契约和 mock 回归已离线
验证，但尚未声称完成任何真机动作或 MoveIt 真机轨迹测试。

### Jazzy 退出兼容层

当前 Jazzy 二进制组合（MoveIt 2.12.4、rclcpp 28.1.21）存在已知的
`move_group` 退出期 callback group 析构故障。本项目的 mock 与 real 入口统一使用
`hex_arm_moveit_runtime` 的有序退出可执行文件和窄范围兼容 shim。其 CMake 版本检查
采用 fail-closed：升级 MoveIt 或 rclcpp 后必须重新审计并移除或更新兼容层，不能绕过
版本检查继续构建。

运行以下命令可执行无界面的规划与轨迹执行冒烟测试：

```bash
python3 src/hex_arm_moveit_config/test/test_moveit_mock.py
```
