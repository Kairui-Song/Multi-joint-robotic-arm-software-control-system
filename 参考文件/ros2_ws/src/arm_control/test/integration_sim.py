#!/usr/bin/env python3
"""Run after colcon build and sourcing install/setup.bash. Simulation only."""
import os
import argparse
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from controller_manager_msgs.srv import ListControllers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['sim', 'can_mock', 'rm_mock', 'mit_mock'], default='rm_mock')
    args = parser.parse_args()
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '73'))
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    config = Path(__file__).resolve().parents[1] / 'config' / {
        'sim': 'robot.yaml', 'can_mock': 'robot_can.yaml', 'rm_mock': 'robot_robomaster.yaml',
        'mit_mock': 'robot_mit.yaml'}[args.backend]
    backend = 'can_mock' if args.backend == 'mit_mock' else args.backend
    count = 7 if args.backend == 'mit_mock' else 6
    with tempfile.TemporaryFile(mode='w+') as log, tempfile.TemporaryDirectory(prefix='arm_integration_') as artifacts:
        bag = str(Path(artifacts) / 'recording')
        process = subprocess.Popen(['ros2', 'launch', 'arm_control', 'bringup.launch.py',
                                    f'backend:={backend}', f'config:={config}', 'record:=true', f'bag_path:={bag}'], env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init(); node = Node('arm_integration_test'); faults = []
        node.create_subscription(DiagnosticArray, '/diagnostics',
                                 lambda m: faults.extend(s for s in m.status if s.name == 'arm_control/hardware' and s.level == DiagnosticStatus.ERROR), 10)
        def run(*args):
            subprocess.run(args, env=env, check=True, timeout=90)
        try:
            client = node.create_client(ListControllers, '/controller_manager/list_controllers')
            deadline = time.monotonic() + 35
            ready = False
            while time.monotonic() < deadline and process.poll() is None:
                if not client.wait_for_service(timeout_sec=1): continue
                future = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2)
                if future.done() and {c.name: c.state for c in future.result().controller} == {
                        'joint_state_broadcaster': 'active', 'joint_trajectory_controller': 'inactive'}:
                    ready = True; break
                time.sleep(.2)
            if not ready: raise RuntimeError('Controllers did not load')
            run('ros2', 'run', 'arm_control', 'recover.py', '--enable')
            run('ros2', 'run', 'arm_control', 'send_trajectory.py', '--config', str(config),
                '--positions', *['.1' if i % 2 == 0 else '-.1' for i in range(count)], '--duration', '2')
            if args.backend not in ('sim', 'mit_mock'):
                run('ros2', 'run', 'arm_control', 'move_pose.py', '--config', str(config),
                    '--fk-joints', '.12', '-.12', '.12', '-.12', '.12', '-.12', '--duration', '3', '--execute')
            run('ros2', 'service', 'call', '/arm_hardware/inject_fault', 'std_srvs/srv/Trigger', '{}')
            deadline = time.monotonic() + 5
            while not faults and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.1)
            if not faults: raise RuntimeError('Injected fault was not diagnosed')
            run('ros2', 'run', 'arm_control', 'recover.py', '--enable')
            run('ros2', 'run', 'arm_control', 'send_trajectory.py', '--config', str(config),
                '--positions', *(['0'] * count), '--duration', '2')
            # Broad CI threshold checks telemetry plumbing, not a real-time performance claim.
            run('ros2', 'run', 'arm_control', 'latency_probe.py', '--seconds', '2',
                '--output', str(Path(artifacts) / 'timing'), '--max-p99-jitter-us', '20000')
        except Exception:
            log.seek(0); print(log.read()); raise
        finally:
            node.destroy_node(); rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
        info = subprocess.run(['ros2', 'bag', 'info', bag], env=env, check=True,
                              capture_output=True, text=True, timeout=15).stdout
        for topic in ('/joint_states', '/diagnostics', '/arm_hardware/timing'):
            if topic not in info:
                raise RuntimeError('Recorded topic missing: ' + topic)
        print('PASS: plugin load, lifecycle, trajectory, injected fault, recovery, telemetry and rosbag2')


if __name__ == '__main__':
    main()
