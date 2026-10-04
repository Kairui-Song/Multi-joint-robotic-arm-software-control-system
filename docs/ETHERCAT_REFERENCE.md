# ROS2 多关节机器人控制系统

在原有 CAN 驱动旁新增 `ros2_ws/src/arm_control`，目标平台为 **Ubuntu 22.04 + ROS2 Humble + ros2_control**。
支持任意数量的旋转关节，默认六轴模拟配置；真实总线后端使用 **IgH EtherCAT 1.6 / CiA402 CSP（模式 8）**。

链路：`FollowJointTrajectory → controller_manager / joint_trajectory_controller → ArmSystem(SystemInterface) → SimBus 或 IghBus → 多轴伺服 → position/velocity → joint_state_broadcaster → /joint_states`。

原来的 `can_rv.*`、`math_ops.*` 保留原样。它们是 CAN 协议组帧/解析库，缺少 ZLG SDK 和实际传输实现，不能直接转换成 EtherCAT 驱动，也没有被伪装接入新总线。新轨迹客户端补充了同步三次贝塞尔轨迹采样。

## 已提供的代码

- **ROS2 控制链路**：pluginlib 硬件插件、位置命令、位置/速度反馈、控制器生成、启动文件、N 关节统一配置。
- **Lifecycle**：初始化、配置、激活、停用、清理、关闭、错误回调；启动后硬件保持 inactive，显式激活才使能。
- **Diagnostics**：`/diagnostics` 发布活动状态、锁存故障、周期、最大绝对抖动、read/write 用时、超期数、限幅数和时序样本丢弃数，以及逐轴位置、速度、状态字、驱动错误码、实际模式。
- **Watchdog**：单调时钟检查控制循环间隔和 read→write 超时；IgH 配置 SM2 输出看门狗。进程完全挂死时依赖伺服自身的通信看门狗。
- **Joint Limit**：位置硬限幅、每周期速度限幅、NaN/Inf 拒绝、编码器溢出检查；激活前验证反馈位置在范围内。轨迹客户端预检目标位置和贝塞尔峰值速度。
- **Fault Recovery**：总线异常、驱动故障、模式/使能异常、非法反馈和控制超时锁存；恢复脚本先卸载控制器清除旧轨迹，再重新配置、使能、加载控制器。
- **rosbag2**：启动参数打开录包，记录关节反馈、诊断、逐周期时序、控制器状态及轨迹 topic。
- **Latency/Jitter Test**：控制线程通过预分配 SPSC 环形缓冲输出每周期时序，普通 ROS 线程批量发布；测试脚本输出 CSV/JSON、P50/P95/P99/最大值、缺失样本和通过/失败结果。
- **验证入口**：C++ 安全核心测试、硬件生命周期测试、Python 配置测试、ROS2 模拟集成测试和 GitHub Actions 配置。

这里只实现 position CSP 控制。没有虚构力矩反馈、驱动器加速度约束、碰撞检测、动力学模型或安全认证；URDF 为控制接口占位模型，连杆几何/惯量/effort 值不能用于 MoveIt 或动力学仿真。

## Linux 构建和模拟运行

先安装 ROS2 Humble 并配置软件源，然后在项目根目录执行：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-yaml \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher ros-humble-rosbag2
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
ros2 launch arm_control bringup.launch.py backend:=sim
```

另一个终端 source 相同环境。等两个控制器完成加载后，显式使能：

```bash
ros2 control list_controllers
ros2 run arm_control recover.py --enable
ros2 run arm_control send_trajectory.py \
  --config src/arm_control/config/robot.yaml \
  --positions 0.2 -0.2 0.1 0.0 0.1 -0.1 --duration 5
ros2 topic echo /joint_states
```

`--config` 路径以 `ros2_ws` 为工作目录；自定义关节数时，目标位置数量必须一致。六轴共享一条轨迹的时间参数，每轴 P0=P1=起点、P2=P3=终点，端点速度为零。客户端发送 position/velocity/acceleration，JTC 进行轨迹插值；硬件只接收 position。

控制器动作接口为 `/joint_trajectory_controller/follow_joint_trajectory`，也支持标准 `/joint_trajectory_controller/joint_trajectory` topic。直接调用标准接口同样受硬件位置/速度限幅保护，但可能由于超限而触发轨迹误差中止。

## 故障注入与恢复

```bash
# 仅模拟后端提供此服务
ros2 service call /arm_hardware/inject_fault std_srvs/srv/Trigger '{}'
ros2 topic echo /diagnostics
# 显式恢复并使能；不会重放旧轨迹
ros2 run arm_control recover.py --enable
```

故障码：0 无故障、1 总线、2 驱动/反馈、3 预留限位码、4 周期看门狗、5 非有限命令。恢复后重新提交新轨迹。

Watchdog 监控控制循环与总线通信，不以“目标位置长时间不变”判断上层掉线；轨迹结束后的正常保持不会超时。如果应用需要操作者/规划器在线租约，应在上层增加独立的使能租约协议。

正常停机先停控制器，再停硬件：

```bash
ros2 control set_controller_state joint_trajectory_controller inactive
ros2 control set_hardware_component_state ArmSystem inactive
```

`recover.py --enable` 同时用于首次激活和故障恢复。它检查每次服务响应；未知活动控制器会阻止恢复。真实驱动的故障复位默认禁用，现场确认故障原因已排除后，才将配置中的 `reset_faults_on_activate` 设为 true 并重启节点。每次显式激活最多发一次 fault-reset 脉冲，不会无限复位。

## EtherCAT 实机接入

此仓库没有伺服型号、ESI、PDO、编码器与减速比资料，**示例配置不能使能真实电机**。`backend:=igh` 不会回退成模拟；未编译 IgH、未填写从站身份或未声明 commissioned 都会报错。

安装并配置 IgH 1.6 主站、专用网卡、主站服务与设备权限后构建：

```bash
colcon build --symlink-install --cmake-args -DARM_WITH_IGH=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 launch arm_control bringup.launch.py backend:=igh config:=/absolute/path/robot_real.yaml
# 检查状态并完成现场使能条件后执行
ros2 run arm_control recover.py --enable
```

`robot_real.yaml` 从示例复制修改。必须按实际设备确认：

1. 每个从站 `alias/position/vendor_id/product_code`，实际轴数和机械正方向。
2. `counts_per_rad = 编码器每转计数 × 减速比 / (2π)`（仅当驱动目标位置单位确为编码器计数）。`zero_counts` 为关节零点的原始计数，`direction` 为 ±1。反馈速度 0x606C 必须为相同单位的 counts/s；不同单位需修改后端转换。
3. `lower/upper/max_velocity` 使用输出关节 rad、rad/s，不能套用示例机械限位。
4. 当前后端明确支持的 PDO：SM2 Rx = `6040:00 u16, 607A:00 i32, 6060:00 i8`；SM3 Tx = `6041:00 u16, 6064:00 i32, 606C:00 i32, 6061:00 i8, 603F:00 u16`。可配置 Rx/Tx PDO 索引；条目、宽度和 SM 编号固定。若 ESI 不支持这个映射，需修改 `src/igh_bus.cpp`；这不是适配所有伺服的通用 ESI 解析器。
5. DC 默认关闭。确认驱动的 AssignActivate 后填写 `dc_assign`，控制周期来自同一 `cycle_ns`；代码发送 application time 和时钟同步请求，但仍需现场测量 DC 偏差。
6. `watchdog_divider/watchdog_intervals` 按驱动/ESC 手册设置。示例按常见公式 `(divider+2)×40ns×intervals` 为 10ms，必须实际验证。通信停止后究竟自由停车、制动还是保持，由驱动参数决定；代码故障停机发送 disable voltage（0x0000），不能替代重力轴抱闸与 STO。
7. 经核对配置和停机行为后再填写 `commissioned: true`。配置标志只是一道软件门槛，不代表硬件已验收。

真实后端检查域 Working Counter 完整、链路和每个从站 online/OP；激活通过 0x6040/0x6041 状态机等待 Operation Enabled，并验证 0x6061==8。进入使能前先用实际位置对齐目标，避免零位跳变。总线丢帧立即锁存停机，没有隐式容错重试。

## 录包与时延测试

```bash
ros2 launch arm_control bringup.launch.py backend:=sim record:=true bag_path:=arm_bag_001
# 另一个终端激活，然后测量；命令不会发送运动指令
ros2 run arm_control recover.py --enable
ros2 run arm_control latency_probe.py --seconds 60 \
  --output results/latency --max-p99-jitter-us 500
ros2 bag info arm_bag_001
```

时序包 `/arm_hardware/timing` 使用 Float64MultiArray，每连续 5 个元素为：cycle、实际周期 µs、read µs、write µs、期望周期 µs。环形缓冲共 4096 槽，诊断线程每 100ms 排空；满时丢弃新样本并计数，不阻塞控制线程。每周期没有 ROS 消息分配/发布。

JSON 区分控制周期抖动、硬件接口调用耗时、JointState 接收间隔、JointState 消息时间戳年龄。read/write 耗时不含控制器 update，周期包含整体调度影响；消息年龄包含排队与 DDS 传输，跨机需要时钟同步。它们都不等于伺服命令到机械响应的物理延迟。进程内采样会影响时序，性能报告需要记录 CPU、内核、负载和录包是否启用。

测试在无数据、样本缺口、诊断错误、时钟倒退或 P99 超阈值时返回非零；阈值是项目配置，不能作为保证 1kHz 硬实时的证明。配置 1kHz 只是目标频率。生产验收应在实际 Linux/实时内核、CPU 亲和性和调度权限配置下测量；同时做断线、进程挂起、驱动故障、限位、恢复和长时间带载测试。

rosbag2 不会记录 Action 目标/结果的完整调用语义；当前记录控制器状态和轨迹 topic，Action 客户端通过该 topic 以外发送的目标不会自动变成轨迹 topic 消息。需要目标审计时另行记录提交内容。回放只在隔离的 ROS_DOMAIN_ID、未连接真实驱动的环境进行，避免录制的命令 topic 被实机控制器订阅。

## 测试与当前验证边界

```bash
# 不依赖 ROS2 的配置测试
python3 src/arm_control/test/test_config.py
# source install/setup.bash 后执行：启动模拟系统，走动作、故障和恢复链路
python3 src/arm_control/test/integration_sim.py
```

GitHub Actions 提供 Humble 构建、单元测试和模拟集成流程；仅在将项目放入 GitHub 仓库后运行。IgH 可选后端仍需安装对应库的 Linux 主机编译及实机验证。

本次开发所在 Windows 环境不具备 ROS2 Humble / IgH 和可运行的 Docker daemon；不能声称已完成 ROS2 链路运行或实机实时性验证。实机接通仍需用户提供伺服型号/ESI、PDO、关节参数及目标 Linux/ROS2 版本。

已在本机执行：7 项 Python 单元测试全部通过，7 个 Python 文件语法检查和 package/plugin XML 解析通过。C++ 编译测试、ROS2 模拟集成、rosbag2 实际录制、IgH 编译与实机测试尚未执行。CI 集成时的宽松抖动门槛只检查时序数据链路，不代表实时性能验收。

接口依据：[ros2_control Humble 硬件插件](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)、[JointTrajectoryController](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)、[IgH 1.6 API](https://docs.etherlab.org/ethercat/1.6/doxygen/group__ApplicationInterface.html)。
