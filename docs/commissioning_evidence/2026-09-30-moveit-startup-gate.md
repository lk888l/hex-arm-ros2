# MoveIt 真机启动门控与失败退出诊断

用户提供的 `/tmp/hex-arm-real-launch.can2.ijSamo/launch.log` 与同次
`hex-arm-exit-timqncrc/startup-ready.json` 证实：启动动作成功，失败发生在随后的静止验收。
所有时间均为 2026-09-30 UTC。

| 时间 | 事件 |
| --- | --- |
| 10:12:22.525 | `startup_j3` 成功，J2/J4/J3 顺序启动完成；终点最大误差 0.0034323 rad |
| 10:12:25.149 | RViz 提交规划请求，处于 10 秒 ready 静止验收期间 |
| 10:12:30.145 | RViz 提交 Plan & Execute，轨迹控制器接受并开始执行 |
| 10:12:32.888 | ready 验收检测到离位，启动脚本主动停止控制器并失能硬件 |
| 10:12:33.243 | Rust 驱动写入本进程 `disabled_confirmed` 回执；驱动正常退出 |

静止验收期间 J3 最大离位 0.206340 rad，反馈峰值速度 0.167779 rad/s。
这是真实的新轨迹运动，不是重力补偿保持误差。原始报告 `passed=false`、`deactivated=true`，
错误为 `ready hold exceeds the 0.015 rad ROS goal requirement`。
驱动回执 PID=15962、`error=null`。末尾 pal_statistics/RViz context 错误位于随后退出过程，
相关进程均正常退出。原监督入口将启动脚本的退出码 1 误标成了“shutdown crash”。

修复后，真实 MoveIt runtime 在启动期间保留严格碰撞与规划服务，延后创建
MoveGroup 和 ExecuteTrajectory 动作入口。启动客户端完成静止验收后通知本次 runtime，
确认两个动作入口就绪再完成启动。通知绑定每次 launch 的独立标识，旧启动的消息不能解锁新启动。
启动报告成功且匹配当前 hardware profile 后，launch 才打开 RViz。
CiA402 的启动自检可以在保持验收通过后使用 MoveIt，完成自检并失能的 trial 不打开 RViz。

Meow ready 验收继续要求位置误差 ≤0.015 rad，并检查静止峰值速度 ≤0.02 rad/s。
监督入口先独立核对驱动失能回执，再报告启动或其他 ROS 失败；失败仍返回非零退出码。
对本次原始失败日志重放新 verifier，得到失能成功、启动失败两条独立结果，退出码仍为 1。

验证结果：

- 相关三个包重新构建成功。
- 实际 C++ MoveIt runtime 离线集成：碰撞服务可用、两个动作入口未开放；错误通知被忽略；
  正确通知后两个入口开放并成功规划；锁定和解锁两种退出路径均正常。
- launch 检查覆盖 RViz 延后、本次 profile 与通知绑定、缺失/失败/过期报告拒绝、已失能 trial。
- ROS 测试汇总 308 项，0 错误、0 失败、0 跳过；监督 shell 检查及 4 项回执测试通过。

本次修改没有进行新的真机使能或运动：准备复测时，can2 电机身份只读查询没有收到回复。
此前大范围 MoveIt 真机运动结果仍见[部署验收](2026-09-30-can2-moveit-deployment.md)，
不能把此前验收冒充本次启动门控的实机复测。
原始故障日志、报告、回执及 verifier 重放结果保留在 `.tmp-moveit-startup-failure/`。
