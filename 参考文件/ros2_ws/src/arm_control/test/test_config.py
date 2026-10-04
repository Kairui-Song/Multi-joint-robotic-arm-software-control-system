import copy
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


model = load('model', ROOT / 'launch/model.py')
probe = load('probe', ROOT / 'scripts/latency_probe.py')
trajectory = load('trajectory', ROOT / 'scripts/send_trajectory.py')


class ConfigurationTest(unittest.TestCase):
    def test_control_parameters_cannot_cross_protocols(self):
        cases = [('robot_mit.yaml', 'position_kp'), ('robot_mit.yaml', 'velocity_ki'),
                 ('robot_robomaster.yaml', 'kp'), ('robot_robomaster.yaml', 'master_id'),
                 ('robot_can.yaml', 'master_id')]
        for filename, key in cases:
            with self.subTest(filename=filename, key=key):
                cfg = yaml.safe_load((ROOT / 'config' / filename).read_text(encoding='utf-8'))
                cfg['joints'][0][key] = 1
                with self.assertRaisesRegex(ValueError, key): model.build(cfg)

    def test_control_gain_ranges(self):
        for filename, key, value in [('robot_mit.yaml', 'kp', 501), ('robot_mit.yaml', 'kd', -1),
                ('robot_mit.yaml', 'kp', float('nan')), ('robot_robomaster.yaml', 'position_kp', 0),
                ('robot_robomaster.yaml', 'velocity_ki', -1), ('robot_robomaster.yaml', 'velocity_kp', float('inf'))]:
            with self.subTest(filename=filename, key=key, value=value):
                cfg = yaml.safe_load((ROOT / 'config' / filename).read_text(encoding='utf-8'))
                cfg['joints'][0][key] = value
                with self.assertRaises(ValueError): model.build(cfg)

    def test_backend_protocol_mismatch(self):
        for filename, backend in [('robot_mit.yaml', 'rm_mock'), ('robot_robomaster.yaml', 'can_mock')]:
            cfg = yaml.safe_load((ROOT / 'config' / filename).read_text(encoding='utf-8'))
            with self.assertRaises(ValueError): model.build(cfg, backend)

    def test_mit_reference_configuration(self):
        cfg = yaml.safe_load((ROOT / 'config/robot_mit.yaml').read_text(encoding='utf-8'))
        urdf, controllers = model.build(cfg)
        self.assertEqual(len(ET.fromstring(urdf).findall('ros2_control/joint')), 7)
        self.assertEqual(controllers['controller_manager']['ros__parameters']['update_rate'], 100)
        with self.assertRaises(ValueError):
            model.build(cfg, 'socketcan')
        cfg['joints'][1]['master_id'] = 17
        with self.assertRaises(ValueError):
            model.build(cfg)

    def setUp(self):
        self.config = yaml.safe_load((ROOT / 'config/robot.yaml').read_text(encoding='utf-8'))

    def test_six_axis_consistency(self):
        urdf, controllers = model.build(self.config)
        root = ET.fromstring(urdf)
        names = [j['name'] for j in self.config['joints']]
        self.assertEqual([j.attrib['name'] for j in root.findall('ros2_control/joint')], names)
        self.assertEqual(controllers['joint_trajectory_controller']['ros__parameters']['joints'], names)
        self.assertEqual(controllers['controller_manager']['ros__parameters']['update_rate'], 1000)
        self.assertEqual(len(root.findall('joint')), 6)
        self.assertEqual(root.find('ros2_control/hardware/plugin').text, 'arm_control/ArmSystem')

    def test_arbitrary_joint_count(self):
        for count in (1, 3, 12):
            self.config['joints'] = [dict(self.config['joints'][0], name=f'joint{i}', position=i) for i in range(count)]
            urdf, controllers = model.build(self.config)
            self.assertEqual(len(ET.fromstring(urdf).findall('ros2_control/joint')), count)
            self.assertEqual(len(controllers['joint_trajectory_controller']['ros__parameters']['joints']), count)

    def test_real_hardware_gate(self):
        with self.assertRaises(ValueError): model.build(self.config, 'igh')
        self.config['hardware']['commissioned'] = True
        with self.assertRaises(ValueError): model.build(self.config, 'igh')
        for j in self.config['joints']:
            j['vendor_id'] = 1; j['product_code'] = 2
        model.build(self.config, 'igh')  # Model validation only, no bus opened.

    def test_invalid_inputs(self):
        for key, value in [('lower', 4), ('max_velocity', 0), ('counts_per_rad', -1),
                           ('direction', 0), ('upper', float('nan'))]:
            cfg = copy.deepcopy(self.config); cfg['joints'][0][key] = value
            with self.assertRaises(ValueError, msg=key): model.build(cfg)
        self.config['joints'][1]['name'] = self.config['joints'][0]['name']
        with self.assertRaises(ValueError): model.build(self.config)

    def test_statistics(self):
        self.assertEqual(probe.summary([]), {'samples': 0})
        self.assertEqual(probe.summary([1])['p99'], 1)
        result = probe.summary(list(range(1, 101)))
        self.assertEqual(result['p99'], 99)
        self.assertEqual(result['p95'], 95)
        self.assertEqual(result['mean'], 50.5)

    def test_timing_contract_matches_hardware(self):
        for key, value in [('cycle_ns', True), ('cycle_ns', 1000), ('cycle_ns', 1000000000),
                           ('watchdog_s', float('nan')), ('watchdog_s', .001),
                           ('activation_timeout_s', 0), ('activation_timeout_s', 61)]:
            cfg = copy.deepcopy(self.config); cfg['hardware'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                model.build(cfg)

    def test_can_models_and_geometry(self):
        for file in ('robot_can.yaml', 'robot_robomaster.yaml'):
            cfg = yaml.safe_load((ROOT / 'config' / file).read_text(encoding='utf-8'))
            urdf, controllers = model.build(cfg)
            root = ET.fromstring(urdf)
            self.assertEqual(controllers['controller_manager']['ros__parameters']['update_rate'], 200)
            for j in cfg['joints']:
                joint = root.find("joint[@name='%s']" % j['name'])
                self.assertEqual(list(map(float, joint.find('origin').attrib['xyz'].split())), j['origin_xyz'])
                self.assertEqual(list(map(float, joint.find('axis').attrib['xyz'].split())), j['axis'])
            self.assertIsNotNone(root.find("joint[@name='tool_fixed']"))

    def test_robomaster_bus_constraints(self):
        cfg = yaml.safe_load((ROOT / 'config/robot_robomaster.yaml').read_text(encoding='utf-8'))
        with self.assertRaises(ValueError): model.build(cfg, 'rm_zcan')
        bad = copy.deepcopy(cfg); bad['joints'][1]['motor_id'] = 1
        with self.assertRaises(ValueError): model.build(bad)
        bad = copy.deepcopy(cfg); bad['hardware']['can_channels'] = 1
        for i, joint in enumerate(bad['joints']):
            joint['can_channel'] = 0; joint['motor_id'] = i + 1
        with self.assertRaises(ValueError): model.build(bad)

    def test_bezier_endpoints_and_peak_velocity(self):
        start, goal, duration = [-1., 2.], [1., -2.], 4.
        first = trajectory.bezier(start, goal, 0., duration)
        last = trajectory.bezier(start, goal, 1., duration)
        self.assertEqual(first[0], start); self.assertEqual(last[0], goal)
        self.assertEqual(first[1], [0., 0.]); self.assertEqual(last[1], [0., 0.])
        for k in range(101):
            positions, velocities, _ = trajectory.bezier(start, goal, k / 100, duration)
            for a, b, q, v in zip(start, goal, positions, velocities):
                self.assertGreaterEqual(q, min(a, b)); self.assertLessEqual(q, max(a, b))
                self.assertLessEqual(abs(v), 1.5 * abs(b - a) / duration + 1e-12)

    def test_bezier_derivatives(self):
        u, h, duration = .37, 1e-5, 3.
        q1, v1, _ = trajectory.bezier([0.], [1.], u - h, duration)
        q2, v2, _ = trajectory.bezier([0.], [1.], u + h, duration)
        _, velocity, acceleration = trajectory.bezier([0.], [1.], u, duration)
        self.assertAlmostEqual((q2[0] - q1[0]) / (2 * h * duration), velocity[0], places=8)
        self.assertAlmostEqual((v2[0] - v1[0]) / (2 * h * duration), acceleration[0], places=8)


if __name__ == '__main__':
    unittest.main()
