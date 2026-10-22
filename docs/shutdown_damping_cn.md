# 真机正常退出：回安全位、阻尼下落、失能

本功能用于**供电和 CAN 通信仍正常时的软件退出**，不是断电后的制动。
实现及离线测试不等于实机验收；本机新增的阻尼增益是待验证初值。
首次验证应保留人工承托/隔离空间，记录下落速度和最终落位，不能据此认定已经消除撞击。

## 操作入口

使用 `scripts/docker-dev.sh real-launch moveit ... enable_execution:=true`，
或独立生产 `compose.runtime.yaml`。默认 Meow 启动完成实测位置保持、自动 J2→J4→J3
及最终 `startup_ready` 静止验收，且 profile 配置了 `controller.shutdown_damping`，才启用柔和退出。
显式或旧版原地保持的兼容报告仍需通过完整保持证据验证。

在拥有 real-launch 的终端按**第一次 Ctrl+C 后，机械臂可能主动运动**：

1. ROS、MoveIt 和 ros2_control 暂时保持运行。默认由实例 StopArm 服务发布 STOPPING，
   `execution_ready=false`；上层运动客户端应停止提交目标。
   顺序取消 MoveGroup、ExecuteTrajectory、FJT goal，并通过 GetResult 确认每个旧目标已终止，随后检查静止反馈。
2. 通过严格碰撞检查的 MoveIt 规划，回到安全启动位
   `[0,-1.350,1.430,-0.300,0,0]` rad，默认使用 0.25/0.25 速度、加速度缩放。
   已在目标附近则只检查保持，不重复规划。
3. 六轴位置误差均不超过 0.005 rad、速度不超过 0.02 rad/s，连续保持 0.5 s 后，
   请求 Rust 接管退出。Rust 独立复核状态、位置、速度及电机目标合法性。
4. Rust 按 `unload_sec` 平滑降低 Kp 和重力前馈至零，同时过渡到退出 Kd；
   目标速度为零。电机仍使能，利用速度反馈阻尼随重力下落。
5. 完全卸载后，在折叠入口 `[0,-1.570,1.570,0,0,0]` rad 附近
   （每轴误差 ≤0.02 rad、速度 ≤0.02 rad/s）连续静止 `settle_sec`，才确认柔和阶段完成；
   随后执行并确认全轴 Disable，最后退出 ROS。

若已经停在折叠终点，退出时确认静止并直接失能。
对原地保持的兼容报告，若仍位于其摆放窗口且只有已知折叠模型接触，
预检完整路径后按 J3→J4→J2 回折并失能，J1/J5/J6 保持当前位置。
普通工作位置继续执行返回固定 ready 位和阻尼退出。

安全启动位和折叠位均读取同一份
[`startup.yaml`](../src/hex_arm_controller/config/startup.yaml)。开机按 J2→J4→J3 到达 ready；
ROS 开机轨迹按本次动态限值计算时长，最终 ready 保持验收为 3 秒。
**下落阶段是六轴阻尼，不是 J2→J3→J4 的主动折叠轨迹**；回安全位由 MoveIt 协调规划，
环境障碍物仍需正确加入规划场景。

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
默认 StopArm 入口的 `soft-stop.json` 包含 `motion_completed`、`disabled_confirmed` 和
`motion_report`；详细轨迹记录在 `motion_report` 指向的 `operations-stop-*.json`。
已经禁能而跳过运动时记录 `motion_skipped`，不把它当作完成回折的证据。
最后仍应等待开发 supervisor 的 `VERIFIED structured disabled_confirmed`。
该回执仅确认失能，和柔和阶段报告是两个独立结果。

## 返回 READY 的诊断与交接

关机从取消后静止的控制器参考开始规划；实际反馈仍用于碰撞、静止和 0.015 rad 保持误差检查。
执行前再次验证实测位置、当前参考和计划首点，参考与首点偏差超过 0.0001 rad 即拒绝执行。
这保留柔顺关节的保持参考；参考不新鲜、仍在运动或保持误差过大时停止回位并走禁能流程。
STOPPING 是集成客户端的提交门控，不是原生 MoveIt/FJT 服务的权限隔离；直接调用这些底层
接口的其他客户端也必须遵守 `hex_arm/status.execution_ready`，否则仍可能抢占关机轨迹。

关机报告默认记录以下数据，失败时同样保留：

强制杀死 helper 或整个进程组可能使报告未写完；完整记录依赖正常返回或异常清理阶段完成。

| 字段 | 内容 |
| --- | --- |
| `last_plan.trajectory` | 所有计划点的位置、速度、加速度、起始时间 |
| `action_feedback` | FJT 期望/实际/误差的位置及速度、各点时间 |
| `controller_states` | JTC 参考、实际和误差，包含交接期间样本 |
| `driver_states` | 驱动模式、故障码及原始故障原因、指令/反馈年龄 |
| `commands` | C++ 实际发送指令的诊断采样 |
| `interpolation_states` | Rust 消费的指令序号、目标、插补后目标、请求/有效时长与采样时间 |
| `handoff` | 各旧目标的取消及终止结果、静止后的实测位置和参考 |
| `ready_completed` / `motion_completed` / `disable_result` | 返回 READY、整个回位/回折和 helper 禁能的独立结果 |

`last_plan` 还包含实际执行状态、原始错误字符串和各轴最大位置/速度误差。
`interpolation_states` 通过可选 protobuf 字段传输，旧驱动不会提供这一记录；应同时重编译
`hex_arm_protocol`、`hex_arm_msgs`、`hex_arm_controller`、`hex_arm_hardware` 和 `hex_arm_bringup`。
采样话题采用 best-effort QoS，不向实时控制循环施加可靠订阅的背压；这不是每个控制 tick 的无损记录。
Rust 时间是驱动进程的单调时钟，不能直接减去 Python 的 `t` 或 ROS 时间戳；用指令序号关联，
在各自时钟内计算时长。比较 `effective_duration_ns` 与 `requested_duration_ns` 可观察底层重定时。
`interpolation_summary` 汇总已采到的段数、发生重定时的段数和最大有效时长；
`commands` 的 ROS 时间表示诊断发布时间，Rust 的指令序号标识真正被消费的目标。

两个 stop helper 可分别用 `--shutdown-velocity-scaling`、`--shutdown-acceleration-scaling`
设置返回段比例，默认均为 0.25；campaign 的 `--velocity-scaling` 和 `--acceleration-scaling`
仍只影响 campaign 动作。先比较 0.25 与原 0.5 的误差曲线，再决定是否调整插补职责。
本次保留底层速度/加速度限制、0.11 rad 路径容差和 J3→J4→J2 顺序。
这些软件改动已增加离线回归测试，故障是否消除仍需实机报告验证。

修改后先在已停止控制的开发容器中重新编译并 source：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to hex_arm_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DHEX_ARM_BUILD_COMMISSIONING=OFF
source install/setup.bash
```

独立生产镜像还需重新 build；增量开发 install 不会更新已构建的生产镜像。
