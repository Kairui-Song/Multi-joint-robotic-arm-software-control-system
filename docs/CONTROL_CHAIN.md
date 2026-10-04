# ROS2 控制链工程约定

兼容基线为 Ubuntu 22.04 / ROS2 Humble。保留 `arm_control` 包和现有启动命令。

## 层次与接口

- `move_pose.py` / `kinematics.py`：位姿与关节目标转换，不直接操作总线。
- `send_trajectory.py`：贝塞尔、解析峰值约束、整数纳秒时间戳，以及 FollowJointTrajectory 客户端。`trajectory_samples` 可在无 ROS 环境测试。
- `controller_manager` / `joint_trajectory_controller`：接口占用、控制周期和轨迹执行；硬件继续使用 position 命令接口及 position/velocity 状态接口。
- `ArmSystem`：生命周期、read/write、状态与故障。状态接口按名称校验，不要求 URDF 声明顺序；有效反馈到达之前及总线关闭后使用 NaN 表示未知状态。
- `Bus` 与协议后端：设备通信、协议转换及底层伺服。保留全部 CAN、RoboMaster、Loong、模拟和可选 EtherCAT 后端。

### RoboMaster 与 MIT 的实现边界

- `RoboMasterBus` 运行主机位置 P＋速度 PI，发送 C620 电流帧。
- `MitBus` 负责 `mit_shared_v1` 的使能、ID 0/master ID 反馈分流、位置/Kp/Kd 命令和失能；速度、力矩前馈保持零。
- `CanBus` 仅负责 `rv_packed_v1`；CAN 收发驱动继续复用。
- `make_bus(config)` 根据后端与协议路由，拒绝错误组合。原 YAML 和 launch 参数保持兼容；直接使用 C++ 工厂的 MIT 调用者需传完整配置。
- 激活阶段由 `prepare_activate()` 和 `activation_step()` 驱动，后者返回 Waiting/Ready/Fault。`ArmSystem` 检查反馈新鲜性、位置/速度有效性、激活期限，并在 Ready 后从实测位置初始化命令。MIT 等待新反馈时保持使能；RoboMaster 激活时清零积分和旧就绪标记，不重建编码器参考。
- YAML 与直接 HardwareInfo 输入均拒绝跨协议控制参数。启动日志及诊断提供 backend、protocol、control_mode、joint_count 和 configured_cycle_ns（日志中为 axes/cycle_ns）。

本轮后端分离验证：WSL 独立构建的 10/10 个 CTest 测试组通过，包含 MIT 激活等待/重新激活/超时、后端路由、配置参数隔离及 RoboMaster 协议闭环。`test_system.cpp` 已补充六轴 RoboMaster 和七轴 MIT 的 ROS 生命周期及重新激活断言；当前 WSL 未安装 ROS2，这些新增 ROS 测试尚未运行，实机亦未验证。

硬件配置失败会清理已申请的总线资源；停用发送失败会返回生命周期错误。清理与析构中的停止操作捕获发送异常并关闭总线。重新激活从实测位置初始化命令和速度，避免沿用旧目标。

## 配置和部署

`launch/model.py` 继续从一份 YAML 生成 URDF 与控制器配置。启动时检查周期范围、看门狗、激活超时和加速度限制；轨迹入口再次验证运动相关限制，避免直接调用脚本绕过校验。安装不包含 Python 缓存文件。

四个 CLI 工具支持标准 `--ros-args`；客户端使用相对接口名，可重映射，根命名空间下原有路径不变。例如连接已有的另一个轨迹控制器：

```bash
ros2 run arm_control send_trajectory.py --config "$CFG" \
  --positions 0.1 -0.1 0.1 -0.1 0.1 -0.1 --duration 3 \
  --ros-args -r joint_states:=/arm_a/joint_states \
  -r joint_trajectory_controller/follow_joint_trajectory:=/arm_a/trajectory/follow_joint_trajectory
```

此能力针对客户端；当前 bringup 与硬件监控仍使用原有根命名空间，不宣称已经实现多机械臂部署。恢复工具继续支持 `--manager` 指定控制器管理器。

Action 的终态与错误码均须表示成功，并核对最终反馈；执行超时取消已接受目标，无法确认取消时输出错误。目标接受超时仍属于未知状态，不能认为控制器未开始执行。取消异常也必须释放客户端。离线 Action 测试使用替身验证客户端分支，不代表 DDS 或控制器集成验收。

## 验证

离线构建和测试（无需 ROS，无真实电机连接）：

```bash
cmake -S ros2_ws/src/arm_control -B .test-build/standardization -DARM_STANDALONE=ON
cmake --build .test-build/standardization -j 2
ctest --test-dir .test-build/standardization --output-on-failure
```

CTest 包含安全、CAN、RoboMaster、配置、运动学、轨迹契约和六轴协议闭环。轨迹契约测试覆盖非法限值、时间戳、导数、目标拒绝、抢占终态、执行超时及取消资源清理。

本次 WSL 重新编译后，7/7 个 CTest 测试组通过（含 10 项配置、6 项运动学、7 项轨迹契约测试）。WSL 缺少 NumPy，已仅在 `.test-deps/linux` 安装测试依赖；本机复现需使用 `PYTHONPATH="$PWD/.test-deps/linux" ctest --test-dir .test-build/standardization --output-on-failure`。Python 脚本与 launch 语法检查通过。

ROS 目标环境使用 `scripts/validate_humble.sh`，或现有 `.github/workflows` CI。`test_system.cpp` 新增接口顺序无关、未知初始状态、重新激活丢弃旧目标和清理后状态失效的回归断言。

本次本机 WSL 为 Ubuntu 26.04，未安装 ROS2。因此本次未验证 `ArmSystem` 的 ROS 编译、生命周期运行和 ROS Action 集成；厂商 SDK 与实机行为也需目标环境验收。贝塞尔的解析限值不等同于实机动态响应或机械安全保证。

实现依据：[Humble 硬件组件契约](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)、[Humble 轨迹控制器](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)。
