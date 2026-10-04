"""Single configuration source for joint names, URDF and controllers."""
import math
import xml.etree.ElementTree as ET


def visual(link, shape, dimensions, xyz, rgba, rpy=(0, 0, 0)):
    item = ET.SubElement(link, 'visual')
    ET.SubElement(item, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))
    ET.SubElement(ET.SubElement(item, 'geometry'), shape, **{k: str(v) for k, v in dimensions.items()})
    material = ET.SubElement(item, 'material', name=link.attrib['name'] + '_material_' + str(len(link)))
    ET.SubElement(material, 'color', rgba=rgba)


def segment(link, endpoint, radius, color):
    # The next joint origin is expressed in this link's frame. Align a cylinder
    # with that vector without changing any kinematic joint transforms.
    x, y, z = map(float, endpoint)
    length = math.sqrt(x*x + y*y + z*z)
    if length > 1e-9:
        visual(link, 'cylinder', {'radius': radius, 'length': length},
               (x/2, y/2, z/2), color,
               (0, math.atan2(math.hypot(x, y), z), math.atan2(y, x)))


def validate_control_parameters(joint, protocol):
    rm_keys = {'position_kp', 'velocity_kp', 'velocity_ki', 'current_limit_a', 'startup_position', 'can_channel'}
    mit_keys = {'master_id', 'p_max', 'v_max', 't_max'}
    forbidden = ({'kp', 'kd', 'motor_zero_rad'} | mit_keys if protocol == 'robomaster_c620'
                 else rm_keys | (mit_keys if protocol == 'rv_packed_v1' else set()))
    mixed = sorted(forbidden.intersection(joint))
    if mixed:
        raise ValueError(f"Joint {joint['name']}: parameters {', '.join(mixed)} do not belong to {protocol}")
    required = (('position_kp', 'velocity_kp', 'velocity_ki', 'current_limit_a', 'startup_position',
                 'gear_ratio', 'max_temperature_c') if protocol == 'robomaster_c620'
                else ('kp', 'kd', 'gear_ratio', 'motor_zero_rad'))
    for key in required:
        if key not in joint or not math.isfinite(float(joint[key])):
            raise ValueError(f"Joint {joint['name']}: missing or non-finite {key} for {protocol}")
    if joint['gear_ratio'] <= 0:
        raise ValueError('gear_ratio must be positive')
    if protocol == 'robomaster_c620':
        if (joint['position_kp'] <= 0 or joint['velocity_kp'] <= 0 or joint['velocity_ki'] < 0
                or not 0 < joint['current_limit_a'] <= 20 or joint['max_temperature_c'] <= 0):
            raise ValueError('Invalid RoboMaster host P/PI gains, current or temperature limit')
    else:
        if not 0 < joint['kp'] <= 500 or not 0 <= joint['kd'] <= 5:
            raise ValueError('Invalid motor protocol kp/kd; these are not host P/PI gains')
        p_max = joint['p_max'] if protocol == 'mit_shared_v1' else 12.5
        v_max = joint['v_max'] if protocol == 'mit_shared_v1' else 18
        if joint['max_velocity'] * joint['gear_ratio'] > v_max:
            raise ValueError('Joint speed exceeds motor protocol range')
        if any(abs(joint['motor_zero_rad'] + joint['direction'] * joint['gear_ratio'] * q) > p_max
               for q in (joint['lower'], joint['upper'])):
            raise ValueError('Joint range exceeds motor protocol position range')


def build(config, backend=None):
    hw = dict(config['hardware'])
    if backend:
        hw['backend'] = backend
    if hw['backend'] not in ('sim', 'igh', 'can_mock', 'socketcan', 'zcan', 'loong', 'rm_mock', 'rm_socketcan', 'rm_zcan'):
        raise ValueError('Unsupported backend')
    joints = config['joints']
    if not joints or len({j['name'] for j in joints}) != len(joints):
        raise ValueError('Empty or duplicate joints')
    cycle = hw['cycle_ns']
    if type(cycle) is not int or not 100000 <= cycle <= 100000000 or 1000000000 % cycle:
        raise ValueError('cycle_ns must be 100000..100000000 and divide one second exactly')
    for key in ('watchdog_s', 'activation_timeout_s'):
        if not math.isfinite(hw[key]):
            raise ValueError('Non-finite hardware timing: ' + key)
    if hw['watchdog_s'] <= cycle * 1e-9 or not 0 < hw['activation_timeout_s'] <= 60:
        raise ValueError('Invalid watchdog or activation timeout')
    if hw['backend'] == 'igh' and (hw['commissioned'] is not True or
                                   any(not j['vendor_id'] or not j['product_code'] for j in joints)):
        raise ValueError('Real EtherCAT requires commissioned=true and actual slave identities')
    can = hw['backend'] in ('can_mock', 'socketcan', 'zcan')
    rm = hw['backend'] in ('rm_mock', 'rm_socketcan', 'rm_zcan')
    if rm:
        if hw['can_protocol'] != 'robomaster_c620' or hw['can_bitrate'] != 1000000 or hw['can_feedback_hz'] != 1000:
            raise ValueError('C620 profile requires classic CAN 1Mbit/s and 1kHz motor feedback')
        if hw['can_channels'] not in (1, 2): raise ValueError('Use one or two CAN channels')
        ids = [(j['can_channel'], j['motor_id']) for j in joints]
        if len(set(ids)) != len(ids) or any(ch < 0 or ch >= hw['can_channels'] or mid < 1 or mid > 8 for ch, mid in ids):
            raise ValueError('Invalid/duplicate C620 channel/ID')
        for ch in range(hw['can_channels']):
            selected = [j for j in joints if j['can_channel'] == ch]
            groups = len({(j['motor_id'] - 1) // 4 for j in selected})
            frames = len(selected) * hw['can_feedback_hz'] + groups * 1e9 / cycle
            if not selected or frames * 135 / hw['can_bitrate'] > .7:
                raise ValueError('C620 channel empty or estimated load >70%; distribute motors across channels')
        if hw['backend'] != 'rm_mock' and not all(hw.get(k) is True for k in (
                'commissioned', 'motor_watchdog_confirmed', 'startup_reference_confirmed', 'group_exclusive_confirmed')):
            raise ValueError('C620 physical hardware requires startup reference and bus/stop commissioning')
    if can:
        if hw['can_protocol'] not in ('rv_packed_v1', 'mit_shared_v1') or hw['can_bitrate'] <= 0:
            raise ValueError('Invalid CAN protocol/bitrate')
        if hw['can_protocol'] == 'mit_shared_v1':
            masters = [j['master_id'] for j in joints]
            if len(set(masters)) != len(masters) or any(type(i) is not int or not 1 <= i <= 255 for i in masters):
                raise ValueError('Invalid/duplicate MIT master ID')
            for j in joints:
                if any(not math.isfinite(j[k]) or j[k] <= 0 for k in ('p_max', 'v_max', 't_max', 'max_temperature_c')):
                    raise ValueError('Invalid MIT range or temperature limit')
        ids = [j['motor_id'] for j in joints]
        if len(set(ids)) != len(ids) or any(not isinstance(i, int) or i < 1 or i >= 2047 for i in ids):
            raise ValueError('Invalid or duplicate motor IDs')
        if len(joints) * 270 * 1e9 / cycle / hw['can_bitrate'] > .7:
            raise ValueError('Estimated CAN utilization exceeds 70%')
        if hw['backend'] != 'can_mock' and not all(hw.get(k) is True for k in (
                'commissioned', 'protocol_confirmed', 'stop_confirmed', 'motor_watchdog_confirmed')):
            raise ValueError('Real CAN requires protocol, stop and watchdog commissioning')
    if hw['backend'] == 'loong':
        if hw.get('commissioned') is not True or hw.get('motor_watchdog_confirmed') is not True:
            raise ValueError('Loong real hardware requires commissioning and drive watchdog confirmation')
        ids = [j['sdk_index'] for j in joints]
        if len(set(ids)) != len(ids) or any(not isinstance(i, int) or i < 0 for i in ids):
            raise ValueError('Invalid SDK index mapping')
    if can or rm:
        for joint in joints:
            validate_control_parameters(joint, hw['can_protocol'])
    root = ET.Element('robot', name='multi_joint_arm')
    base = ET.SubElement(root, 'link', name='base_link')
    visual(base, 'cylinder', {'radius': .09, 'length': .05}, (0, 0, .025), '0.15 0.18 0.23 1')
    segment(base, joints[0].get('origin_xyz', [0, 0, 0]), .04, '0.25 0.30 0.38 1')
    control = ET.SubElement(root, 'ros2_control', name='ArmSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'arm_control/ArmSystem'
    def param(parent, name, value):
        ET.SubElement(parent, 'param', name=name).text = str(value).lower() if isinstance(value, bool) else str(value)
    for key, value in hw.items():
        param(hardware, key, value)
    parent = 'base_link'
    for index, j in enumerate(joints):
        if rm and (j['gear_ratio'] <= 0 or not j['lower'] <= j['startup_position'] <= j['upper'] or
                   not 0 < j['current_limit_a'] <= 20):
            raise ValueError('Invalid C620 ratio, startup position or current limit')
        if not all(math.isfinite(float(j[k])) for k in ('lower', 'upper', 'max_velocity')):
            raise ValueError('Non-finite joint configuration')
        if 'max_acceleration' in j and (not math.isfinite(j['max_acceleration']) or j['max_acceleration'] <= 0):
            raise ValueError('Invalid joint acceleration limit')
        if j['lower'] >= j['upper'] or j['max_velocity'] <= 0 or j.get('counts_per_rad', 1) <= 0 or j['direction'] not in (-1, 1):
            raise ValueError('Invalid joint limits or encoder scale')
        link = j['name'] + '_link'
        link_element = ET.SubElement(root, 'link', name=link)
        visual(link_element, 'sphere', {'radius': .035}, (0, 0, 0), '0.15 0.18 0.23 1')
        endpoint = (joints[index + 1].get('origin_xyz', [0, 0, 0]) if index + 1 < len(joints)
                    else config.get('kinematics', {}).get('tool_xyz', [0, 0, 0]))
        segment(link_element, endpoint, .023,
                '0.12 0.48 0.85 1' if index % 2 == 0 else '0.72 0.79 0.88 1')
        joint = ET.SubElement(root, 'joint', name=j['name'], type='revolute')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=link)
        def triple(values):
            if len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
                raise ValueError('Invalid joint geometry')
            return ' '.join(str(v) for v in values)
        axis = j.get('axis', [0, 0, 1])
        if sum(v * v for v in axis) <= 0: raise ValueError('Zero joint axis')
        ET.SubElement(joint, 'axis', xyz=triple(axis))
        ET.SubElement(joint, 'origin', xyz=triple(j.get('origin_xyz', [0, 0, 0])), rpy=triple(j.get('origin_rpy', [0, 0, 0])))
        ET.SubElement(joint, 'limit', lower=str(j['lower']), upper=str(j['upper']), velocity=str(j['max_velocity']), effort='1')
        parent = link
        cj = ET.SubElement(control, 'joint', name=j['name'])
        for key, value in j.items():
            if key not in ('name', 'origin_xyz', 'origin_rpy', 'axis'):
                param(cj, key, value)
        cmd = ET.SubElement(cj, 'command_interface', name='position')
        param(cmd, 'min', j['lower']); param(cmd, 'max', j['upper'])
        ET.SubElement(cj, 'state_interface', name='position')
        ET.SubElement(cj, 'state_interface', name='velocity')
    if 'kinematics' in config:
        tool = config['kinematics']
        tool_link = ET.SubElement(root, 'link', name='tool_link')
        visual(tool_link, 'box', {'size': '0.035 0.045 0.045'}, (0, 0, 0), '1 0.5 0.08 1')
        joint = ET.SubElement(root, 'joint', name='tool_fixed', type='fixed')
        ET.SubElement(joint, 'parent', link=parent); ET.SubElement(joint, 'child', link='tool_link')
        ET.SubElement(joint, 'origin', xyz=triple(tool.get('tool_xyz', [0, 0, 0])), rpy=triple(tool.get('tool_rpy', [0, 0, 0])))
    controllers = {
        'controller_manager': {'ros__parameters': {
            'update_rate': 1000000000 // cycle,
            'hardware_components_initial_state': {'inactive': ['ArmSystem']},
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'joint_trajectory_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'}}},
        'joint_trajectory_controller': {'ros__parameters': {
            'joints': [j['name'] for j in joints], 'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'], 'state_publish_rate': 100.0,
            'action_monitor_rate': 20.0, 'allow_partial_joints_goal': False,
            'constraints': {'goal_time': 2.0, 'stopped_velocity_tolerance': 0.02,
                            **{j['name']: {'trajectory': 0.2, 'goal': 0.02} for j in joints}}}}}
    return ET.tostring(root, encoding='unicode'), controllers
