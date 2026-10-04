---
name: ros2-control-standardization
description: 为本机械臂项目实施 ROS2 控制链标准化、系统化、规范化，基于 ros2_control 统一轨迹执行、硬件接口、CAN 通信、配置、生命周期与验收。用于明确调用本技能进行控制链重构、规范审查或工程治理。
disable-model-invocation: true
---

项目规范化：1、将ROS2控制链改成标准控制框架
2、将ROS2控制链标准化、系统化、规范化

## 执行原则

- 先读项目 AGENTS.md、README、构建与 CI 配置，再核对代码；不要把 README 的能力说明当作验证结果。
- 默认以项目现有 Ubuntu 22.04 / ROS2 Humble 为兼容基线；修改 API 前核对安装版本及对应官方文档，不直接套用 Rolling/Jazzy API。
- 本项目已包含 ros2_control，先找差距，再增量改造。用户要求实施时完成代码、配置、文档与必要验证，不停留在建议。
- 保留 CAN 通信、同步三次贝塞尔、FK/IK 及已支持的后端；不能为了目录整齐删除能力或改写协议。
- 所有“标准化完成”结论必须对应实际证据。无 ROS、厂商 SDK 或实机时明确未验证项。

## 目标控制链及职责

1. 任务层：关节目标或末端位姿；FK/IK 和贝塞尔生成合法的带时间轨迹。
2. 执行层：通过 `control_msgs/action/FollowJointTrajectory` 向 `joint_trajectory_controller` 提交轨迹，处理接受、拒绝、反馈、取消、抢占、超时和结果。
3. 管理层：`controller_manager` 管理控制器、接口资源和更新周期；避免额外循环与它争用同一硬件命令。
4. 硬件层：`ArmSystem` 实现 `hardware_interface::SystemInterface`，通过 `read/write` 与总线抽象交换状态和命令，以 pluginlib 导出。
5. 驱动层：总线适配封装协议、设备、线程及单位转换；RoboMaster 位置 P / 速度 PI 到电流的伺服职责与上层轨迹跟踪清晰分开，避免重复闭环。
6. 状态层：编码器反馈经硬件状态接口、`joint_state_broadcaster` 发布 `/joint_states`；URDF 与 `robot_state_publisher` 提供一致的 TF 模型。

默认保持已验证的位置命令接口，提供真实位置/速度反馈。新增 velocity/effort 接口须说明驱动能力、控制律及单位；电流不可直接冒充力矩。按目标发行版核对接口组合。

## 工作流程

### 1. 建立基线

- 阅读 `ros2_ws/src/arm_control` 中的 `package.xml`、`CMakeLists.txt`、`arm_plugins.xml`、`launch/bringup.launch.py`、`launch/model.py`、`include/arm_control/system.hpp` 和 `src/system.cpp`。
- 追踪 `scripts/send_trajectory.py`、`scripts/move_pose.py` 到总线驱动及反馈的完整路径；检查配置、恢复脚本、测试与 CI。
- 按“已实现且验证 / 已实现待验证 / 缺失”记录差距，标出文件与行为依据；先解决功能和接口问题，再调整目录。

### 2. 统一工程与配置

- 明确模型、bringup、硬件、轨迹、运维、测试的模块边界；只有依赖或复用需求成立才拆 ROS 包，不把拆包本身当作验收成果。
- 使用 ament/colcon、明确依赖和安装规则；从安装空间也能启动，避免依赖源码目录或开发机绝对路径。
- 关节名称、顺序、方向、减速比、零点、限位、控制周期和反馈超时保持单一配置来源；校验重复 ID、非法数值及跨文件不一致。
- ROS 接口统一 SI 单位（rad、rad/s、m、s），协议转换集中在驱动边界。统一命名空间、TF frame、时间源和 QoS 配置。
- 保持现有启动入口兼容；迁移参数或包名时同步更新 launch、安装、测试和用户命令。

### 3. 规范轨迹与运行行为

- 贝塞尔输出使用标准 JointTrajectory 表达，验证维度、关节映射、有限值、严格递增时间、起止边界及位置/速度/加速度约束。
- 核对采样点与控制器插值后的轨迹行为；不能仅检查采样点就声称连续轨迹满足全部限制。
- 启动轨迹使用新鲜反馈；执行成功依据 Action 结果及实际反馈，不以发送成功或固定等待代替。
- 明确 configure、activate、deactivate、cleanup、shutdown、error 的资源与输出语义；保留默认 inactive 和显式使能流程。
- 故障锁存并停止输出，恢复前校验反馈和参考位置，恢复后不重放旧命令；验证取消、抢占及停用时的行为。
- 检查 read/write 的阻塞、锁、分配、日志和厂商调用；明确线程所有权、同步和退出顺序。没有时序证据不宣称硬实时。
- 保留 diagnostics、周期/耗时统计和 rosbag 入口，记录指标定义，区分软件反馈年龄与真实 CAN/机械响应延迟。

### 4. 保留本项目硬件约束

- 默认使用 `robot_robomaster.yaml` / `rm_mock` 验证；C620 经典 CAN 与原 RV packed 协议、Loong 后端不可混用。
- 不改变通道/电机 ID、组电流独占、限流、温度、反馈超时、命令看门狗与编码器多圈参考约束来绕过故障。
- C620 单圈编码器不具备断电绝对位置能力；停用零电流不等于保持重力负载。保留已有调试确认与标定门槛。
- 模拟测试不连接实机、不自动设置 commissioned/calibrated、不自动使能真实电机。

### 5. 分层验证并交付

- 离线：沿用 `ARM_STANDALONE=ON` 的 CMake/CTest 入口，执行受影响的协议、配置、运动学、轨迹测试。
- ROS 目标环境：运行 colcon build/test/test-result，验证插件加载、控制器状态与接口占用、关节反馈、Action 成功/失败/取消/抢占及生命周期。
- 模拟集成：沿用 `test/integration_sim.py` 等现有入口，验证故障注入、恢复、反馈停滞与记录；新增测试针对行为风险，不复制实现逻辑。
- 厂商/实机：分别记录 SDK 编译、适配器通信、方向/零点、限位、掉线和停用验证。缺少条件则写明阻塞项与可复现命令，不伪造通过。
- 交付说明列出已改行为、涉及模块、兼容影响、实际执行命令与结果、未验证项；同步 README 与相关部署文档。

## 调用示例

- “使用 ros2-control-standardization，检查现有控制链并完成标准化重构。”：核查后实施、验证并更新文档。
- “使用 ros2-control-standardization，只审查框架差距。”：提供带代码依据的差距与验收条件，不修改运行代码。

## 官方参考

- [Humble 轨迹控制器与 Action 接口](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [Humble 硬件组件与插件实现](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)

执行时按项目实际安装版本重新核对参考；以上链接不是跨发行版兼容保证。
