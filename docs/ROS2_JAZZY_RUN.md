# Ubuntu 虚拟机运行记录

2026-10-03 已在 `skr@192.168.25.129` 的 ROS 2 Jazzy 下构建当前项目。
独立工作区为 `/home/skr/can_bezier_ws`，未替换原 Linglong 工作区。
使用 `robot_mit.yaml`、`can_mock` 七轴模拟后端，ROS_DOMAIN_ID 为 73。

## 验证结果

- CTest 11/11 通过。
- MIT 模拟联测通过：插件加载、生命周期、轨迹、故障注入、恢复、时序采样、rosbag2。
- 连续多路点贝塞尔示例执行完成，最终反馈位置与目标一致。
- 17:33 状态检查：模拟服务与两个控制器 active，七轴反馈可用，fault_code=0。
- RViz 已随模拟服务启动。

原始日志在 `测试结果/ros2_jazzy_20261003/`。
虚拟机运行期间存在调度超时与硬件停用，已显式恢复；不作为长期实时性或实机验收结论。

## 在 Ubuntu 终端操作

准备命令环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/can_bezier_ws/install/setup.bash
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1
```

检查当前运行状态：

```bash
systemctl --user status can-bezier-mit-sim --no-pager
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

再次执行模拟轨迹：

```bash
ros2 run arm_control send_trajectory.py \
  --config ~/can_bezier_ws/src/arm_control/config/robot_mit.yaml \
  --path ~/can_bezier_ws/src/arm_control/config/path_mit_example.json
```

控制停用时，在确认仍使用模拟后端后恢复：

```bash
ros2 run arm_control recover.py --enable
```

停止服务（同时关闭其 RViz）：

```bash
systemctl --user stop can-bezier-mit-sim
```

重启服务：

```bash
systemctl --user restart can-bezier-mit-sim
```

等待启动日志中两个 spawner 均退出，再执行 recover.py；不要与 spawner 同时加载控制器。
已将临时服务改为 `~/.config/systemd/user/can-bezier-mit-sim.service` 固定用户服务，停止后仍可 start/restart，没有配置开机自启。重启虚拟机并登录后使用 `systemctl --user start can-bezier-mit-sim`。

每个新终端均须先执行本页的 source/export。也可以使用已安装的包装脚本，自动加载工作区及 Domain 73，无需手动 source：

```bash
~/can_bezier_ws/arm_ros2.sh control list_controllers
~/can_bezier_ws/arm_ros2.sh run arm_control recover.py --enable
~/can_bezier_ws/arm_ros2.sh run arm_control send_trajectory.py --config ~/can_bezier_ws/src/arm_control/config/robot_mit.yaml --path ~/can_bezier_ws/src/arm_control/config/path_mit_example.json
```

若需前台运行，先停止服务，避免启动重复节点：

```bash
systemctl --user stop can-bezier-mit-sim
ARM_RVIZ=true bash ~/can_bezier_ws/run_mit_sim.sh
```

在另一个已准备环境的终端中等待控制器加载完成，再执行恢复和轨迹命令。
Windows 复制的 Python 脚本已在 Linux 上补充执行权限，以支持 symlink-install。
