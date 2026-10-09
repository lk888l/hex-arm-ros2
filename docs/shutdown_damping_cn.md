# 真机正常退出：回安全位、阻尼下落、失能

本功能用于**供电和 CAN 通信仍正常时的软件退出**，不是断电后的制动。
实现及离线测试不等于实机验收；本机新增的阻尼增益是待验证初值。
首次验证应保留人工承托/隔离空间，记录下落速度和最终落位，不能据此认定已经消除撞击。

## 操作入口

使用 `scripts/docker-dev.sh real-launch moveit ... enable_execution:=true`，
或独立生产 `compose.runtime.yaml`。只有本次启动完成自动 J2→J4→J3 并通过
`startup_ready` 保持检查、且 profile 配置了 `controller.shutdown_damping`，才启用柔和退出。

在拥有 real-launch 的终端按**第一次 Ctrl+C 后，机械臂可能主动运动**：

1. ROS、MoveIt 和 ros2_control 暂时保持运行；取消当前 MoveIt/FJT goal，并检查静止反馈。
2. 通过严格碰撞检查的 MoveIt 规划，回到安全启动位
   `[0,-1.350,1.430,-0.300,0,0]` rad，固定使用 0.5/0.5 速度、加速度缩放。
   已在目标附近则只检查保持，不重复规划。
3. 六轴位置误差均不超过 0.005 rad、速度不超过 0.02 rad/s，连续保持 0.5 s 后，
   请求 Rust 接管退出。Rust 独立复核状态、位置、速度及电机目标合法性。
4. Rust 按 `unload_sec` 平滑降低 Kp 和重力前馈至零，同时过渡到退出 Kd；
   目标速度为零。电机仍使能，利用速度反馈阻尼随重力下落。
5. 完全卸载后，在折叠入口 `[0,-1.570,1.570,0,0,0]` rad 附近
   （每轴误差 ≤0.02 rad、速度 ≤0.02 rad/s）连续静止 `settle_sec`，才确认柔和阶段完成；
   随后执行并确认全轴 Disable，最后退出 ROS。

安全启动位和折叠位均读取同一份
[`startup.yaml`](../src/hex_arm_controller/config/startup.yaml)，本功能不修改零偏或启动顺序。
**下落阶段是六轴阻尼，不是 J2→J3→J4 的主动折叠轨迹**；回安全位由 MoveIt 协调规划，
不重放开机固定轨迹，也不绕过碰撞检查。环境障碍物仍需正确加入规划场景。

回位失败、阻尼超时、反馈失效、越界、超速或电机故障都不会报告柔和退出成功，
而是转入失能流程。**这时仍可能下落较快**；不能通过放宽安全门限掩盖失败。
再次 Ctrl+C、SIGTERM/SIGHUP、启动未完成或 ROS 意外退出走立即停机路径，不继续回位。
这里的“立即”表示不再等待回位/阻尼，最终硬件失能确认仍需时间；不是硬件急停的替代。

裸 `ros2 launch ...`、`real-launch bringup/startup`、未配置阻尼的 profile 保留原有退出行为。
直接杀死进程、停止开发容器、切断电源不能保证执行上述柔和流程。
开发模式应先在 real-launch 终端完成退出，再停止容器；生产 Compose 使用 SIGINT，
退出宽限时间为 180 s，不要提前强制结束。

## 参数与验证

本机 `config/hardware/firefly_y6.meow.can2.local.yaml` 已加入：

```yaml
controller:
  # 原有 loop_hz 等字段继续保留
  shutdown_damping:
    kd_nm_s_rad: [15.0, 60.0, 90.0, 15.0, 15.0, 15.0]
    unload_sec: 3.0
    timeout_sec: 20.0
    settle_sec: 0.5
```

Kd 按 J1～J6 排列，单位为关节侧 Nm·s/rad；仅在退出阶段生效。
正常运行的 Kp/Kd、重力参数、运动窗口和超速门限不变。
例子模板保留注释形式，其他机械臂不会自动开启。
增益必须六轴有限正数且 ≤100；卸载 1～10 s、静止确认 0.5～2 s、总阻尼时间 ≤30 s，
且至少留出卸载、静止及额外 1 s 的观察时间。实际电机增益/扭矩映射还需通过驱动校验。

初值依据：本机模型在安全位估算 J2/J3 重力力矩约 2.22/4.36 Nm；
用 `|重力力矩|/Kd` 作忽略耦合、惯性和摩擦的粗略速度估算，约为 0.037/0.048 rad/s，
低于原有 0.1 rad/s 门限。这不是速度保证，高 Kd 的噪声/振动和扭矩饱和同样需要实测。

退出不是定时睡眠后无条件失能：未到折叠区，哪怕暂时不动，也会超时报错。
位置与低速判断只能作为软件判据，不能证明机械支撑可靠。
初次验收需检查是否超速、停在半空、到不了折叠区或落位后回弹；有这些现象需重新调参/评估方案，
不能把“最终失能已确认”当作“柔和下落已验证”。

`graceful exit: complete` 表示柔和阶段完成；失败会输出 `FAILED/interrupted` 并返回非零。
`soft-stop.json` 与本次启动报告位于最终 `driver-shutdown.json` 同目录下的独立子目录，
路径在终端输出。生产镜像日志卷会保留这些报告。
柔和报告包含参数、回位/阻尼开始时刻，以及收到的关节位置/速度样本，供首轮验收分析。
最后仍应等待开发 supervisor 的 `VERIFIED structured disabled_confirmed`。
该回执仅确认失能，和柔和阶段报告是两个独立结果。

修改后先在已停止控制的开发容器中重新编译并 source：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hex_arm_controller hex_arm_bridge hex_arm_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DHEX_ARM_BUILD_COMMISSIONING=OFF
source install/setup.bash
```

独立生产镜像还需重新 build；增量开发 install 不会更新已构建的生产镜像。
