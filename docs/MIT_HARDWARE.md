# 参考文件中的 7 轴 CAN 硬件接入

实现依据是 `参考文件/quanbu_tongshi.py`，不是 C620/RV/通用达妙协议。
链路：`send_trajectory.py → 三次贝塞尔 → FollowJointTrajectory → ArmSystem → MitBus(mit_shared_v1) → SocketCAN → 电机`，反馈原路进入 `/joint_states`。

MIT 使用独立 `MitBus`；`CanBus` 仅处理 RV packed 协议，RoboMaster 使用独立的主机位置 P＋速度 PI 控制。配置仍使用 `backend: can_mock/socketcan/zcan` 与 `can_protocol: mit_shared_v1`，工厂根据完整配置选择后端，原启动命令不变。C++ 调用者应使用 `make_bus(config)`；仅传后端名称的旧重载仍按 RV 默认协议选择普通 CAN 后端。

MIT 的 `kp/kd` 是发送给电机的协议增益，不是 RoboMaster 的 `position_kp/velocity_kp/velocity_ki`；配置校验拒绝两类参数混用。MIT 速度与力矩前馈继续为零。

## 已对齐的协议

- 7 个标准帧命令 ID：1–7。反馈标准帧 ID **0**，8 字节首字节为 master ID 17–23（0x11–0x17）。
- 位置 16 位、速度/Kp/Kd/力矩各 12 位。速度前馈和力矩前馈为零，与参考脚本相同。
- J1/J2：位置 ±12.566、速度 ±20、力矩 ±120；J3/J4：±12.5、±20、±28；J5–J7：±12.5、±50、±10。Kp/Kd 同参考脚本。
- 使能 `FF FF FF FF FF FF FF FC`，失能末字节 `FD`。绝不自动发送写零点指令。
- `prepare_activate()` 先清除旧反馈、发零增益指令，再使能；获取各轴新反馈后，从实际位置开始位置控制。未显式激活时只发送失能。
- 激活后逐轴反馈过期、CAN 错误、已知电机反馈格式错误、任一温度达到阈值，锁存故障并向所有轴发送失能。参考协议没有提供状态/错误位定义，不能臆造电机故障码。

## 构建和模拟验证

当前工程实际位于 `参考文件/ros2_ws`。在 Ubuntu 22.04 / ROS2 Humble 上，从项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
cd "参考文件/ros2_ws"
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
CFG="$PWD/src/arm_control/config/robot_mit.yaml"
ros2 launch arm_control bringup.launch.py config:="$CFG"
```

新终端 source 同一工作区后执行 `ros2 run arm_control recover.py --enable --activate-only`。默认 `can_mock`，可观察 7 轴 `/joint_states`，再使用 `send_trajectory.py --config "$CFG" --positions ... --duration ...`，必须给 **7 个弧度目标**。

## PEAK / SocketCAN 实机

参考脚本明确使用 Linux `can0`。本次接口面向 Linux SocketCAN；没有实现 Windows PCAN-Basic 后端。PEAK Linux 驱动参考包的 README 区分字符设备和 `make netdev` 构建；仅有 `/dev/pcan*` 而没有 `can0` 时不能使用 SocketCAN 后端。先在目标 Linux 主机确认适配器提供 CAN 网络接口。

```bash
ip -details link show can0
# 以下 1 Mbit/s 是配置示例，参考 Python 脚本没有给出波特率；必须与实际电机一致。
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

复制 `robot_mit.yaml` 为自己的实机配置，设置 `backend: socketcan` 和实际 `can_interface` / `can_bitrate`。核对零点、方向、减速比、实际关节限位、温度上限和控制增益后，按实测情况填写 `commissioned`、`protocol_confirmed`、`stop_confirmed`、`motor_watchdog_confirmed`，均为 true 才允许打开实机后端。没有确认电机通信超时保护时不要置 true：主机卡死或 USB 断线后，软件无法保证失能帧送达。

用上述 launch 命令加载该配置并显式激活。初次运行先观察反馈与实物一致，再给当前角度附近的小目标。失能会释放输出，不保持重力负载，应有机械支撑。不要同时运行参考动作脚本和 ROS 控制器控制同一总线。

`robot_mit.yaml` 中几何、关节限位和零点是待实测示例；`kinematics.calibrated` 保持 false。新增协议本身不证明 FK/IK 模型已标定，也不把参考脚本的动作范围当作机械限位。

## 验证边界

已运行 Windows 离线协议与模拟测试：参考编码黄金向量、ID 0/master ID 分流、激活帧顺序、反馈超时、畸形帧、过温、重复 ID 拒绝，以及七轴三次贝塞尔的编码/解码闭环；配置和轨迹 Python 测试通过。

当前未连接实机，未在 ROS2/Linux 中运行新增协议；PEAK 实机收发、实际波特率、零增益指令与使能的固件行为、USB 断线保护均须在目标电脑验收。不能将离线模拟结果视为已打通真实硬件。
