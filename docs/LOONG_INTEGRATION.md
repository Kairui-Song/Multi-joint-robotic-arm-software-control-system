# Loong SDK 适配说明

`loong_driver_sdk` 是用户提供的 SDK。其原有 Apache-2.0 授权、版权头与 NOTICE 保留。本次修改发生在 `loong_driver_sdk.cpp`、`loong_driver_sdk.h`、`can.cpp`、`can.h`；必须重新编译，旧二进制没有新增接口。

## 接口连接

`ArmSystem → LoongBus → DriverSDK::instance() → setMotorTarget/getMotorActual → SDK CAN → motors`。

适配层在单独线程访问 SDK，ROS 控制循环通过同步快照交换目标和反馈。每个进程只允许一个适配器拥有 SDK。关节 `sdk_index` 是从零开始的 SDK 电机索引，对应 XML `alias - 1`。XML 必须仅启用这些 CAN 电机，不能同时启用 SDK 的 EtherCAT、RS485 或 IMU。真实标定、方向和单位由 SDK XML 管理，ROS `direction` 必须为 1。

当前适配限定经典 SocketCAN 的 Encos/Damiao 协议，拒绝 CAN HAL、CAN FD 和 RealMan 配置。提供的 Damiao XML 仅展示完整字段和映射，电机参数必须替换；RoboMaster C620 使用另一个 `rm_*` 后端。SDK 中已有其他协议并不意味着本适配层已验收它们。

## SDK 修改

- 增加 `getCANFeedbackAge(std::vector<double>&)`：采用单调时钟，CAN 通道收齐一批映射反馈并发布时更新时间；非 CAN 或尚未接收返回无限年龄，不能把缓存当作新反馈。
- `init(InitConfig)` 传播底层初始化错误。
- 增加 `CAN::stopThreads()`，析构前取消并 join 接收/发送线程，再释放驱动状态；HAL 子接收线程同步退出。
- 共享的 DriverParameters 按类型只释放一次，避免按多个电机 alias 重复 delete。
- epoll 直接携带通道序号，避免用任意系统 fd 索引固定长度数组；补充接收 ID、HAL 长度边界。

适配层解析 XML 后再进入 SDK；检查 CAN-only、必需节点、alias 连续性、活动电机与 ROS 映射一致、反馈 ID 唯一、限位/增益范围，降低 SDK 原始解析器对缺失字段直接解引用的风险。它不是对 SDK 全部输入与线程实现的全面审计。

## 生命周期与限制

SDK 具有进程级全局状态。第一次成功初始化后只能复用同一 XML；初始化失败或变更 XML 要重启进程。停用时发送 disabled 目标，SDK 底层线程保留供后续生命周期使用，进程结束再停止线程。外部电机通信看门狗仍必须配置并实测。

本次仅编译通过 LoongBus 适配源文件。完整 SDK 为 Linux 平台，携带的预编译依赖有 CPU 架构限制；没有在当前 Windows 环境运行 SDK 或真实 CAN。请按 SDK README_BUILD_WSL.md/CMakeLists.txt 构建目标架构依赖，再开启 ARM_WITH_LOONG。不能以此说明代替实机验收。
