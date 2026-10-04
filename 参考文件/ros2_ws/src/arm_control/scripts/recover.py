#!/usr/bin/env python3
"""Explicit activation/recovery; unload old trajectories before enabling hardware."""
import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from controller_manager_msgs.srv import (
    ListControllers, SwitchController, UnloadController, LoadController,
    ConfigureController, SetHardwareComponentState)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manager', default='controller_manager')
    parser.add_argument('--hardware', default='ArmSystem')
    parser.add_argument('--activate-only', action='store_true',
                        help='Activate already configured hardware without closing its encoder receiver (required on initial real C620 startup)')
    parser.add_argument('--enable', action='store_true', required=True,
                        help='Explicitly authorize activation of the configured hardware')
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init(); node = Node('arm_recovery')
    names = ['joint_state_broadcaster', 'joint_trajectory_controller']
    recovery_started = False

    def call(service, suffix, request):
        client = node.create_client(service, args.manager + '/' + suffix)
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('Service unavailable: ' + suffix)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=70)
            if not future.done():
                raise RuntimeError('Timed out: ' + suffix + '; inspect hardware state before retry')
            response = future.result()
            if response is None or (hasattr(response, 'ok') and not response.ok):
                raise RuntimeError('Operation rejected: ' + suffix)
            return response
        finally:
            node.destroy_client(client)

    def state(label):
        request = SetHardwareComponentState.Request()
        request.name = args.hardware; request.target_state.label = label
        result = call(SetHardwareComponentState, 'set_hardware_component_state', request)
        if result.state.label != label:
            raise RuntimeError('Hardware did not reach ' + label)

    try:
        controllers = call(ListControllers, 'list_controllers', ListControllers.Request()).controller
        # Do not enable with an unknown controller still claiming the same hardware.
        others = [c.name for c in controllers if c.state == 'active' and c.name not in names]
        if others:
            raise RuntimeError('Deactivate other controllers before recovery: ' + ', '.join(others))
        # From here a mutating request can reach the server even if its reply times out.
        recovery_started = True
        active = [c.name for c in controllers if c.name in names and c.state == 'active']
        if active:
            request = SwitchController.Request(); request.deactivate_controllers = active
            request.strictness = 2; request.timeout.sec = 5
            call(SwitchController, 'switch_controller', request)
        for controller in controllers:
            if controller.name in names:
                request = UnloadController.Request(); request.name = controller.name
                call(UnloadController, 'unload_controller', request)
        if not args.activate_only:
            state('unconfigured'); state('inactive')
        state('active')
        for name in names:
            request = LoadController.Request(); request.name = name
            call(LoadController, 'load_controller', request)
            request = ConfigureController.Request(); request.name = name
            call(ConfigureController, 'configure_controller', request)
        request = SwitchController.Request(); request.activate_controllers = names
        request.strictness = 2; request.timeout.sec = 5
        call(SwitchController, 'switch_controller', request)
        print('Hardware and controllers active; old trajectory goals discarded. Submit a new goal.')
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        if not recovery_started:
            return 1
        try:
            request = SwitchController.Request(); request.deactivate_controllers = names
            request.strictness = 1; request.timeout.sec = 5
            call(SwitchController, 'switch_controller', request)
            state('inactive')
        except Exception as stop_error:
            print('Could not confirm inactive hardware: ' + str(stop_error), file=sys.stderr)
        return 1
    finally:
        node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
