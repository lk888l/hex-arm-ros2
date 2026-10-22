# 将机械臂接入整机

本包的两个入口均使用 `mock_components/GenericSystem`，不启动真机驱动。

```bash
# 两个 controller manager；验证命名空间、关节前缀和局部故障隔离
ros2 launch hex_arm_integration_examples isolated_arms.launch.py

# 一个整机 controller manager、一个 joint_state_broadcaster、两个轨迹控制器
ros2 launch hex_arm_integration_examples shared_manager.launch.py
```

独立模式的动作接口为 `/left/firefly_arm_controller/follow_joint_trajectory`
和 `/right/firefly_arm_controller/follow_joint_trajectory`。共享模式的接口为
`/left_arm_controller/follow_joint_trajectory` 和
`/right_arm_controller/follow_joint_trajectory`。关节名分别为
`left_joint_1` … `left_joint_6` 和 `right_joint_1` … `right_joint_6`。

## 引用机械臂模型

在整机包中声明 `hex_arm_description` 依赖，包含其已安装的宏：

```xml
<xacro:include filename="$(find hex_arm_description)/urdf/firefly_y6.macro.xacro"/>
<xacro:firefly_y6_arm prefix="left_" parent="chassis"
  xyz="0 0.3 0.05" rpy="0 0 0" backend="mock"
  hardware_name="LeftArmSystem" ros_namespace="left"
  tool_link="tcp" tcp_xyz="0 0 0.10"/>
```

`parent` 是整机已有 link 的完整名称；为空时不创建安装关节。
`prefix` 同时作用于关节、link 和固定连接名称。底层设备关节仍为
`joint_1` … `joint_6`，由 ros2_control 的 `device_joint_name` 参数显式映射。
`hardware_name` 在一个 controller manager 的资源中必须唯一。

`tool_link` 默认为空，此时保留 `link_6` 为末端。设置 `tool_link="tcp"`
会添加相对 `link_6` 的无几何 TCP frame；工具的质量、惯量、视觉与碰撞几何
由整机模型提供。参见 `urdf/dual_arm.urdf.xacro` 的固定工具示例。
真实负载还需要同步配置驱动的 payload 质量和质心；URDF 不会自动改变驱动参数。
整机 MoveIt 应使用整机 URDF/SRDF 并重新配置工具及两臂间的碰撞关系。

## 状态与故障范围

独立模式的 `joint_state_mux.py` 将两路带原始时间戳的关节状态转发到
`/joint_states`，供整机状态发布器使用。消费者按关节名累积各消息；不会为
失联设备补零或生成新的时间戳。输入中的重复关节归属会被拒绝。该公共状态流
不接入驱动的私有安全反馈。

独立模式使用 `shutdown_scope:=arm`：单臂 controller manager 退出时，另一臂
及整机状态发布器继续运行。共享模式中 controller manager 是整机拥有的共同
进程，其整体退出会使两臂控制都不可用；整机应自行定义协调停机策略。

## 复用已有 controller manager

`hex_arm_bringup/mock.launch.py` 支持 `start_controller_manager:=false`、
`start_robot_state_publisher:=false` 和 `controller_manager:=/robot/controller_manager`。
整机必须已在自己的描述中包含目标机械臂硬件，且应为各臂设置不同的
`controller_name`；本包 `shared_manager.launch.py` 展示如何合并控制器参数。
已有整机广播器时设置 `start_joint_state_broadcaster:=false`；若由各臂分别
加载广播器，则为其配置不同的 `state_broadcaster_name`。

`test/test_two_arm_mock.py` 实际发送两路轨迹、拒绝使用另一臂关节名的目标，
并验证停止左臂的独立 manager 后右臂仍可执行。模型测试检查整机树、TCP、
碰撞几何以及底层关节映射。
