import can
import time
import math
import sys


# =========================
# 基础转换函数
# =========================
def float_to_uint(x, x_min, x_max, bits):
    x = max(min(x, x_max), x_min)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return float(x_int) * span / ((1 << bits) - 1) + x_min


def pack_mit_cmd(p_des, v_des, kp, kd, t_ff, p_max, v_max, t_max):
    p_uint = float_to_uint(p_des, -p_max, p_max, 16)
    v_uint = float_to_uint(v_des, -v_max, v_max, 12)
    kp_uint = float_to_uint(kp, 0.0, 500.0, 12)
    kd_uint = float_to_uint(kd, 0.0, 5.0, 12)
    t_uint = float_to_uint(t_ff, -t_max, t_max, 12)

    return [
        (p_uint >> 8) & 0xFF,
        p_uint & 0xFF,
        (v_uint >> 4) & 0xFF,
        ((v_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F),
        kp_uint & 0xFF,
        (kd_uint >> 4) & 0xFF,
        ((kd_uint & 0x0F) << 4) | ((t_uint >> 8) & 0x0F),
        t_uint & 0xFF,
    ]


def send_frame(bus, can_id, data):
    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=False
    )
    bus.send(msg)


# =========================
# 电机配置
# =========================
MOTORS = {
    1: {"slave_id": 0x01, "master_id": 0x11, "p_max": 12.566, "v_max": 20.0, "t_max": 120.0, "kp": 10.0, "kd": 1.0},
    2: {"slave_id": 0x02, "master_id": 0x12, "p_max": 12.566, "v_max": 20.0, "t_max": 120.0, "kp": 10.0, "kd": 1.0},
    3: {"slave_id": 0x03, "master_id": 0x13, "p_max": 12.5,   "v_max": 20.0, "t_max": 28.0,  "kp": 12.0, "kd": 1.0},
    4: {"slave_id": 0x04, "master_id": 0x14, "p_max": 12.5,   "v_max": 20.0, "t_max": 28.0,  "kp": 18.0, "kd": 1.2},
    5: {"slave_id": 0x05, "master_id": 0x15, "p_max": 12.5,   "v_max": 50.0, "t_max": 10.0,  "kp": 20.0, "kd": 1.0},
    6: {"slave_id": 0x06, "master_id": 0x16, "p_max": 12.5,   "v_max": 50.0, "t_max": 10.0,  "kp": 20.0, "kd": 1.0},
    7: {"slave_id": 0x07, "master_id": 0x17, "p_max": 12.5,   "v_max": 50.0, "t_max": 10.0,  "kp": 20.0, "kd": 1.0},
}


# =========================
# 动作参数
# =========================
ACTIVE_JOINTS = [1, 2, 3, 4, 5, 6, 7]

RATE_HZ = 100

# 第一次实机不放心，可以先改成 0.5。
# 确认轨迹正确后再用 1.0。
MOTION_SCALE = 1.0

# J1、J4 主动作
J1_FORWARD_RAD = math.radians(30.0) * MOTION_SCALE
J4_STAGE1_RAD = math.radians(40.0) * MOTION_SCALE
J4_FINAL_RAD = math.radians(90.0) * MOTION_SCALE

# J2：正向 20°，负向最多 60°
J2_POS_RAD = math.radians(20.0) * MOTION_SCALE
J2_NEG_RAD = math.radians(60.0) * MOTION_SCALE

# 其他测试动作：正负 60°
J3_TEST_RAD = math.radians(60.0) * MOTION_SCALE
J5_TEST_RAD = math.radians(60.0) * MOTION_SCALE
J6_TEST_RAD = math.radians(60.0) * MOTION_SCALE
J7_TEST_RAD = math.radians(60.0) * MOTION_SCALE

# 某个关节方向反了，只改这里
JOINT_SIGN = {
    1: 1.0,   # J1 往前方向，反了改 -1.0
    2: 1.0,
    3: 1.0,
    4: 1.0,   # J4 弯曲方向，反了改 -1.0
    5: 1.0,
    6: 1.0,
    7: 1.0,
}


# =========================
# 反馈解析
# =========================
def parse_feedback(data):
    master_id = data[0]

    joint_id = None
    cfg = None

    for jid, m in MOTORS.items():
        if m["master_id"] == master_id:
            joint_id = jid
            cfg = m
            break

    if joint_id is None:
        return None

    p_uint = (data[1] << 8) | data[2]
    v_uint = (data[3] << 4) | (data[4] >> 4)
    t_uint = ((data[4] & 0x0F) << 8) | data[5]

    p = uint_to_float(p_uint, -cfg["p_max"], cfg["p_max"], 16)
    v = uint_to_float(v_uint, -cfg["v_max"], cfg["v_max"], 12)
    t = uint_to_float(t_uint, -cfg["t_max"], cfg["t_max"], 12)

    return {
        "joint_id": joint_id,
        "master_id": master_id,
        "position": p,
        "velocity": v,
        "torque": t,
        "temp1": data[6],
        "temp2": data[7],
    }


def read_positions(bus, joint_ids, timeout=2.0):
    positions = {}
    start = time.time()

    while time.time() - start < timeout:
        msg = bus.recv(timeout=0.05)

        if msg is None:
            continue

        if msg.arbitration_id != 0x000:
            continue

        if len(msg.data) != 8:
            continue

        fb = parse_feedback(list(msg.data))
        if fb is None:
            continue

        jid = fb["joint_id"]

        if jid in joint_ids:
            positions[jid] = fb["position"]
            print(
                f"反馈 J{jid}: "
                f"pos={fb['position']:.4f} rad, "
                f"vel={fb['velocity']:.4f}, "
                f"tau={fb['torque']:.4f}"
            )

        if all(j in positions for j in joint_ids):
            return positions

    return positions


def read_initial_positions_robust(bus, joint_ids):
    positions = {}

    for attempt in range(5):
        print(f"读取当前位置 attempt {attempt + 1}/5")

        for jid in joint_ids:
            enable_motor(bus, jid)
            time.sleep(0.02)

        new_positions = read_positions(bus, joint_ids, timeout=1.0)
        positions.update(new_positions)

        missing = [j for j in joint_ids if j not in positions]

        if not missing:
            return positions

        print("仍缺少反馈:", missing)

    return positions


# =========================
# 控制函数
# =========================
def enable_motor(bus, joint_id):
    send_frame(bus, MOTORS[joint_id]["slave_id"], [0xFF] * 7 + [0xFC])


def disable_motor(bus, joint_id):
    send_frame(bus, MOTORS[joint_id]["slave_id"], [0xFF] * 7 + [0xFD])


def send_mit_joint(bus, joint_id, p_des):
    cfg = MOTORS[joint_id]

    # 不做相对初始位置限幅，只保留电机绝对 p_max 范围保护
    p_des = max(min(p_des, cfg["p_max"]), -cfg["p_max"])

    data = pack_mit_cmd(
        p_des=p_des,
        v_des=0.0,
        kp=cfg["kp"],
        kd=cfg["kd"],
        t_ff=0.0,
        p_max=cfg["p_max"],
        v_max=cfg["v_max"],
        t_max=cfg["t_max"],
    )

    send_frame(bus, cfg["slave_id"], data)


def make_pose(initial_positions, delta_map):
    """
    delta_map 表示相对初始姿态的角度偏移。
    这里不做相对限幅。
    """
    pose = {}

    for jid in ACTIVE_JOINTS:
        delta = delta_map.get(jid, 0.0)
        pose[jid] = initial_positions[jid] + delta

    return pose


def send_pose(bus, pose):
    for jid in ACTIVE_JOINTS:
        send_mit_joint(bus, jid, pose[jid])


def hold_delta_pose(bus, initial_positions, delta_map, duration=0.2, rate_hz=100):
    pose = make_pose(initial_positions, delta_map)

    period = 1.0 / rate_hz
    steps = int(duration * rate_hz)
    next_t = time.perf_counter()

    for _ in range(steps):
        send_pose(bus, pose)

        next_t += period
        sleep_time = next_t - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)


# =========================
# 平滑轨迹函数
# =========================
def smootherstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * x * (x * (x * 6.0 - 15.0) + 10.0)


def interp_points(alpha, points):
    if alpha <= points[0][0]:
        return points[0][1]

    for i in range(len(points) - 1):
        a0, v0 = points[i]
        a1, v1 = points[i + 1]

        if a0 <= alpha <= a1:
            if a1 <= a0:
                return v1

            u = (alpha - a0) / (a1 - a0)
            s = smootherstep(u)
            return v0 + (v1 - v0) * s

    return points[-1][1]


def cubic_hermite(x, x0, y0, m0, x1, y1, m1):
    """
    三次 Hermite 插值。
    用来让 J4 在 40° 连接点不停顿，保证速度连续。
    """
    if x1 <= x0:
        return y1

    u = (x - x0) / (x1 - x0)
    u = max(0.0, min(1.0, u))

    h00 = 2 * u**3 - 3 * u**2 + 1
    h10 = u**3 - 2 * u**2 + u
    h01 = -2 * u**3 + 3 * u**2
    h11 = u**3 - u**2

    return (
        h00 * y0
        + h10 * (x1 - x0) * m0
        + h01 * y1
        + h11 * (x1 - x0) * m1
    )


def move_profile(bus, initial_positions, duration, profile_func, rate_hz=100, name=""):
    if name:
        print(name)

    period = 1.0 / rate_hz
    steps = int(duration * rate_hz)
    next_t = time.perf_counter()

    for i in range(steps):
        alpha = (i + 1) / steps
        delta_map = profile_func(alpha)
        pose = make_pose(initial_positions, delta_map)
        send_pose(bus, pose)

        next_t += period
        sleep_time = next_t - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)


# =========================
# 动作轨迹
# =========================
def profile_stage12(alpha):
    """
    阶段1 + 阶段2 合并：
    1. J1 从 0 到 30°
    2. J4 从 0 到 40°，不停顿继续到 90°
    3. J2：正 60°，负最多 20°
    4. J3：正负 60°
    """

    # 原阶段1 20s，原阶段2 9s，合并成 29s
    split = 12.0 / 16.0

    # ---------- J1 ----------
    if alpha <= split:
        a1 = alpha / split
        s = smootherstep(a1)
        j1 = JOINT_SIGN[1] * J1_FORWARD_RAD * s
    else:
        b = (alpha - split) / (1.0 - split)

        # 阶段2期间 J1 在 30° 附近小幅连续运动，避免阶段切换时完全静止
        j1_base = JOINT_SIGN[1] * J1_FORWARD_RAD
        j1_wave = JOINT_SIGN[1] * math.radians(4.0) * MOTION_SCALE * math.sin(2.0 * math.pi * b)
        j1 = j1_base + j1_wave

    # ---------- J4 ----------
    # J4 从 0° 连续经过 40° 到 90°，40°处不停顿
    j4_mid_speed = JOINT_SIGN[4] * math.radians(120.0) * MOTION_SCALE

    if alpha <= split:
        j4 = cubic_hermite(
            alpha,
            0.0,
            0.0,
            0.0,
            split,
            JOINT_SIGN[4] * J4_STAGE1_RAD,
            j4_mid_speed,
        )
    else:
        j4 = cubic_hermite(
            alpha,
            split,
            JOINT_SIGN[4] * J4_STAGE1_RAD,
            j4_mid_speed,
            1.0,
            JOINT_SIGN[4] * J4_FINAL_RAD,
            0.0,
        )

    # ---------- J2 / J3 ----------
    # J2/J3 只在前半段动作
    if alpha <= split:
        a1 = alpha / split

        j2 = interp_points(a1, [
            (0.00, 0.0),
            (0.06, JOINT_SIGN[2] * J2_POS_RAD),
            (0.18, 0.0),
            (0.30, JOINT_SIGN[2] * -J2_NEG_RAD),
            (0.42, 0.0),
            (1.00, 0.0),
        ])

        j3 = interp_points(a1, [
            (0.00, 0.0),
            (0.34, 0.0),
            (0.46, JOINT_SIGN[3] * J3_TEST_RAD),
            (0.58, 0.0),
            (0.70, JOINT_SIGN[3] * -J3_TEST_RAD),
            (0.82, 0.0),
            (1.00, 0.0),
        ])
    else:
        j2 = 0.0
        j3 = 0.0

    return {
        1: j1,
        2: j2,
        3: j3,
        4: j4,
    }


def profile_wrist_sequence(alpha):
    """
    阶段3：
    J1 保持 30°
    J4 保持 90°
    J5、J6、J7 依次正负 60°
    """
    j1 = JOINT_SIGN[1] * J1_FORWARD_RAD
    j4 = JOINT_SIGN[4] * J4_FINAL_RAD

    j5 = interp_points(alpha, [
        (0.00, 0.0),
        (0.08, JOINT_SIGN[5] * J5_TEST_RAD),
        (0.18, 0.0),
        (0.28, JOINT_SIGN[5] * -J5_TEST_RAD),
        (0.38, 0.0),
        (1.00, 0.0),
    ])

    j6 = interp_points(alpha, [
        (0.00, 0.0),
        (0.26, 0.0),
        (0.36, JOINT_SIGN[6] * J6_TEST_RAD),
        (0.46, 0.0),
        (0.56, JOINT_SIGN[6] * -J6_TEST_RAD),
        (0.66, 0.0),
        (1.00, 0.0),
    ])

    j7 = interp_points(alpha, [
        (0.00, 0.0),
        (0.54, 0.0),
        (0.64, JOINT_SIGN[7] * J7_TEST_RAD),
        (0.74, 0.0),
        (0.84, JOINT_SIGN[7] * -J7_TEST_RAD),
        (0.94, 0.0),
        (1.00, 0.0),
    ])

    return {
        1: j1,
        4: j4,
        5: j5,
        6: j6,
        7: j7,
    }


def profile_return(alpha):
    """
    阶段4：
    从 J1=30°、J4=90° 平滑回初始。
    """
    s = smootherstep(alpha)

    j1 = JOINT_SIGN[1] * J1_FORWARD_RAD * (1.0 - s)
    j4 = JOINT_SIGN[4] * J4_FINAL_RAD * (1.0 - s)

    return {
        1: j1,
        4: j4,
    }


# =========================
# 单次完整动作
# =========================
def run_one_cycle(bus, initial_positions, cycle_index):
    print(f"\n========== 开始第 {cycle_index} 次动作循环 ==========")

    print("\n阶段1+2：J1/J4 连续动作，J4 不停顿地从 0° 到 90°，J2 正60°负20°，J3 正负60°")
    move_profile(
        bus,
        initial_positions,
        duration=16.0,
        profile_func=profile_stage12,
        rate_hz=RATE_HZ,
        name="阶段1+2 连续执行中...",
    )

    print("\n阶段3：J4 保持 90°，J5/J6/J7 依次正负 60°")
    move_profile(
        bus,
        initial_positions,
        duration=9.0,
        profile_func=profile_wrist_sequence,
        rate_hz=RATE_HZ,
        name="J5/J6/J7 依次动作中...",
    )

    print("\n阶段4：全部关节平滑回到初始姿态")
    move_profile(
        bus,
        initial_positions,
        duration=8.0,
        profile_func=profile_return,
        rate_hz=RATE_HZ,
        name="回初始姿态连续执行中...",
    )

    print(f"\n========== 第 {cycle_index} 次动作循环完成 ==========")


# =========================
# 主流程
# =========================
def main():
    loop_mode = "-pan" in sys.argv

    if loop_mode:
        print("检测到 -pan 参数，启动循环动作。")
        print("按 Ctrl+C 可中断，程序会自动失能全部关节。")
    else:
        print("未检测到 -pan 参数，启动单次动作。")
        print("本次动作执行完成后会自动停止并失能。")

    bus = can.interface.Bus(channel="can0", interface="socketcan")

    try:
        print("\n1. 使能全部关节:", ACTIVE_JOINTS)
        for jid in ACTIVE_JOINTS:
            enable_motor(bus, jid)
            time.sleep(0.05)

        time.sleep(0.5)

        print("\n2. 读取初始位置")
        initial_positions = read_initial_positions_robust(bus, ACTIVE_JOINTS)

        missing = [j for j in ACTIVE_JOINTS if j not in initial_positions]
        if missing:
            print("以下关节没有收到反馈，停止:", missing)
            return

        print("\n初始位置:")
        for jid in ACTIVE_JOINTS:
            print(f"  J{jid}: {initial_positions[jid]:+.4f} rad")

        print("\n目标动作：")
        print("  1. 阶段1+2 合并执行，J4 从 0° 连续到 90°，中途不在 40° 停顿")
        print("  2. J2 正 60°，负最多 20°")
        print("  3. J3 正负 60°")
        print("  4. J4 到 90° 后，J5/J6/J7 依次正负 60°")
        print("  5. 全部回初始姿态")
        print("  6. 取消相对限幅，仅保留电机 p_max 绝对范围保护")

        if loop_mode:
            print("  7. 当前为 -pan 模式，会循环执行")
        else:
            print("  7. 当前为单次模式，只执行一遍")

        print("\n3. 初始姿态保持 0.2 秒")
        hold_delta_pose(bus, initial_positions, {}, duration=0.2, rate_hz=RATE_HZ)

        cycle_index = 1

        if loop_mode:
            while True:
                run_one_cycle(bus, initial_positions, cycle_index)
                cycle_index += 1
        else:
            run_one_cycle(bus, initial_positions, cycle_index)
            print("\n单次动作完成，程序结束。")

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，中断动作")

    finally:
        print("\n失能全部关节")
        for jid in ACTIVE_JOINTS:
            disable_motor(bus, jid)
            time.sleep(0.05)

        bus.shutdown()


if __name__ == "__main__":
    main()
