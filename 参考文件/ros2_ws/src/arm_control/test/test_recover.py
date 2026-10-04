"""Recovery preflight must be read-only; failures after mutation must stop hardware."""
import importlib.util
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch


class RecoveryTest(unittest.TestCase):
    def run_recovery(self, *, others=False, list_failure=False, load_failure=False):
        calls = []
        node = Mock()
        services = ('ListControllers', 'SwitchController', 'UnloadController',
                    'LoadController', 'ConfigureController', 'SetHardwareComponentState')
        def request():
            return NS(target_state=NS(label=''), timeout=NS(sec=0))
        service_module = NS(**{name: NS(Request=request) for name in services})

        def create_client(service, name):
            suffix = name.rsplit('/', 1)[-1]
            client = Mock()
            client.wait_for_service.return_value = not (list_failure and suffix == 'list_controllers')
            def call(req):
                calls.append((suffix, req))
                response = NS(ok=True)
                if suffix == 'list_controllers':
                    response.controller = [NS(name='other', state='active')] if others else []
                elif suffix == 'set_hardware_component_state':
                    response.state = NS(label=req.target_state.label)
                elif suffix == 'load_controller' and load_failure:
                    response.ok = False
                future = Mock()
                future.done.return_value = True
                future.result.return_value = response
                return future
            client.call_async.side_effect = call
            return client
        node.create_client.side_effect = create_client
        modules = {'rclpy': Mock(), 'rclpy.node': NS(Node=Mock(return_value=node)),
                   'rclpy.utilities': NS(remove_ros_args=lambda: ['recover.py', '--enable']),
                   'controller_manager_msgs.srv': service_module}
        spec = importlib.util.spec_from_file_location(
            'recover_under_test', Path(__file__).resolve().parents[1] / 'scripts/recover.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
            result = module.main()
        node.destroy_node.assert_called_once()
        return result, calls

    def test_other_controller_rejection_does_not_mutate(self):
        result, calls = self.run_recovery(others=True)
        self.assertEqual(result, 1)
        self.assertEqual([name for name, _ in calls], ['list_controllers'])

    def test_failed_inventory_does_not_mutate(self):
        result, calls = self.run_recovery(list_failure=True)
        self.assertEqual(result, 1)
        self.assertEqual(calls, [])

    def test_failure_after_activation_still_stops(self):
        result, calls = self.run_recovery(load_failure=True)
        self.assertEqual(result, 1)
        self.assertEqual(calls[-2][0], 'switch_controller')
        self.assertEqual(calls[-1][0], 'set_hardware_component_state')
        self.assertEqual(calls[-1][1].target_state.label, 'inactive')

    def test_success_activates_without_rollback(self):
        result, calls = self.run_recovery()
        self.assertEqual(result, 0)
        self.assertEqual(calls[-1][0], 'switch_controller')
        self.assertEqual(calls[-1][1].activate_controllers,
                         ['joint_state_broadcaster', 'joint_trajectory_controller'])


if __name__ == '__main__':
    unittest.main()
