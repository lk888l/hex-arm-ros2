# 2026-09-07 18:08 用户要求暂停：续接记录

这是 18:08 的历史中断快照。用户随后已要求从 can2 继续并批准 J3 Kp=120，
最新结果见 [can2 续接记录](2026-09-07-can2-kp120.md)。

## 停止状态

- 最后真机 supervisor `/tmp/hex-arm-real-launch.can0.evosBV/launch.log`
  已输出 `VERIFIED clean controller exit`。运动客户端先停 JTC/JSB 并确认
  FireflyY6System INACTIVE，再退出整个 owner。所有控制进程已退出。
- 18:08 独立只读 SDO 确认六轴 mode=0、error=0、heartbeat consumer=0。
  位置约 `[0.000025,-1.570004,3.139900,0.002310,0.004458,-0.000077]` rad。
  [原始停止读数](2026-09-07-paused-state.json)。
- 所有模拟 ROS launch、运动客户端、被动抓包均已结束。没有留下持力进程。

## 已落地的本轮修复

- Rust 插值旧算法将可行时长误当作单调区间；确定性 10 ms 匀速段被拉到
  2.989966440 秒。现按三次曲线速度/加速度边界求共同可行时长，每次保留
  连续极值限幅检查。新增 4 个回归测试，完整 Rust 269 通过、1 个实机测试跳过。
  `log/interpolation-repro.log`、`log/interpolation-full-rust-tests.log`。
- C++ 硬件插件已有析构/线程退出修复；服务消失时立即结束停止请求，停止和
  error recovery 不再等待服务重新发现。增加使能前 command publisher DDS
  匹配门槛，和新鲜反馈一起检查。最新硬件测试 9 项通过（mapping 2 + lifecycle 7）。
  `log/dds-start-tests.log`。ROS 全套测试仍需在最终修改后重跑一次。
- 真机 controller overlay 当前只启用 `[position, velocity]`；桥接使用独立
  stream executor，Rust 串行化目标流和使能/失能。完整 ROS 模拟链路已成功
  三步启动、10 秒保持和 MoveIt 19 点规划/执行 J2 +0.008 rad。
  `log/full-mock-moveit-commission-settled.json`。
- 运动脚本 `scripts/commission-startup-ros.py` 增加 ROS 指令记录、每步起始
  信息、失败步骤记录及可选 `--deactivate-after`。应等 launch 控制器激活后
  再运行；过早调用曾在 ROS 图建立期间因客户端状态暂旧而退出。

## 真机结果与剩余问题

- 先前直接 Rust/CAN 固定启动两次成功，见同目录 meow-startup 报告。
- 本轮 17:56 ROS 试验 J2 在约 4 秒处仍出现 0.020189 rad 误差，中止并失能：
  `log/meow-ros-verified-20260907.json`，审计后缀 `pXqF5G`。
- 18:00 被动 CAN 抓包试验 J2 完整成功，误差 0.001887 rad；第二步 J4 到
  -0.296538，但 J3 仅 3.134540，距 3.140 目标差 0.005460，超过原
  0.005 rad 门槛而中止。`log/meow-ros-wire-20260907.json`；审计 `evosBV`。
  原始被动 RPDO 抓包在宿主 `/tmp/meow-wire-targets-20260907.jsonl`，
  每轴约 100 Hz 抽样。该临时解析器把 offset 18 的占位字段误命名为
  total_permille，实际是 0x3000:02；不可将其解读为电机总输出限制。
- 抓包的成功 J2 步骤中 ROS→CAN 目标偏差很小，电机到位约 -1.351887。
  两次试验 ROS 指令/反馈均稳定约 100 Hz。
- 新轨迹默认从实测状态重新起步，PD 静态偏差会造成已连续指令回退。
  用失败试验的真实到达时刻离线回放，部分 500 Hz 相位的插值滞后达
  0.016109 rad；保持指令起点连续的合成对照约 0.002636 rad。
  离线程序在容器 `/tmp/replay-interpolation{,.rs}`，输入
  `/tmp/replay-before-abort.json`、`/tmp/replay-continuous.json`。
  对照只是离线证据，不是真机新设置验收。

## 暂未执行的候选修改与审批状态

自动审批拒绝了一次组合命令：把真机 JTC 设置
`interpolate_from_desired_state: true`，同时把本机 J3 Kp 从 100 调到 120。
拒绝理由是认为广泛调试授权不够具体，且配置/增益修改尚未验证。
整条命令未执行；不能绕过拒绝直接应用。

当前两份本地 Meow profile 的实际 Kp 仍为 `[80,80,100,80,80,80]`，
全部 Kd=15；重力比例 `[0,1,1.05,0.7,0,0]`；J3 PD 450‰、其他 500‰，
总输出均 650‰，速度/加速度均 0.1，watchdog/feedback 均 100 ms。
`controllers_real.yaml` 尚未加入 interpolate_from_desired_state。

已准备容器内候选文件，均未用于真机：

- `/tmp/hex-proposed-gain-review.yaml`：Kp120 候选，calibrated=false，
  不存在的 CAN 接口及全零 USB 身份；仅运行 validate-profile-only，通过。
  日志 `log/proposed-gain-validation.log`；这不证明真实闭环稳定性。
- `/tmp/hex-proposed-controller-review.yaml`：JTC 连续起点候选。
- `/tmp/full_protocol_continuous_mock{,_moveit}.launch.py`：只在模拟后端
  加载上述 JTC 候选的临时 launch，尚未运行。

下一次在用户要求继续后，先复核只读现场状态；完成候选离线/模拟验证及可
审阅差异，处理审批限制，再决定真机验证。保持现有 0.02/0.005 rad 容差、
100 ms watchdog 和严格碰撞矩阵。完整真实 ROS 三步 + 10 秒保持 + MoveIt
执行尚未验收通过，不能在 README 中写成已完成。

所有源码、文档修改尚未提交；README 原有用户改写必须保留。
CAN 接口灵活绑定及 MoveIt mock 理想初始姿态已实现，参见 README。
最终还需统一现场报告/README 中参数与结果、归档 ROS 验收证据并跑最终 ROS 测试。
