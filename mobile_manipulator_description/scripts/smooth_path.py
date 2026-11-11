"""TOPP-RA blending for the mono plans (mono-arm experiment): runs of joint moves / straight lines between two jaw
actions become ONE time-optimal trajectory that passes through every waypoint without stopping. Straight lines (the
hand approaching or leaving a product) are timed the same way but stop at both ends: blending the lift out of the tote
into the fold cut the corner and the carried product hit the robot (offline check).

How: every step of a run is turned into dense joint-space waypoints (joint moves: straight in joint space; lines: IK
every few mm, so the TCP still follows the line), a cubic spline goes through all of them, and TOPP-RA finds the fastest
timing within each joint's speed limit (SPEED x the URDF velocity) and an acceleration limit (speed / RAMP). It slows
down by itself at the corners between steps. Steps with an explicit duration (the slow 80 mm shelf insertions) and the
jaw actions stay as they are.

    plan = expand(steps, start_q, vmax)  -> list of ('traj', label, Q (N x 6)) / original steps
    run(task, steps, ...)                  executes it in Isaac (mono_poses.run's arguments)

Needs `pip install toppra` (0.6.x).
"""
import numpy as np
import scara_ik as K

KEYS = ('lift', 'shoulder_pan', 'elbow_pan', 'wrist_pan', 'wrist_pitch', 'wrist_roll')
DQ, DZ, DX = 0.02, 0.004, 0.004          # waypoint spacing: rad per joint, m of lift, m of TCP along a line
SPEED = 0.88                             # fraction of the URDF speed limits (as Task.joint_move)
RAMP = 0.3                               # s: accel limit = speed limit / RAMP (as the 'trap' profile)
CTRL_HZ = 60


def _vec(q):
    return np.array([q[k] for k in KEYS], float)


def _dense(a, b):
    """Points from a to b (excluding a), straight in joint space, each joint <= DQ (lift <= DZ) per step."""
    d = np.abs(b - a); n = int(max(1, np.ceil(max(d[0] / DZ, np.max(d[1:]) / DQ))))
    return [a + (b - a) * (i / n) for i in range(1, n + 1)]


def _line_points(cur, tgt):
    x, y, z, yaw, pitch, e = tgt[:6]; a = K.fk('left', dict(zip(KEYS, cur)))
    L = float(np.linalg.norm(np.array([x, y, z]) - np.array(a[:3])))
    n = int(max(2, np.ceil(L / DX))); out = []
    for i in range(1, n + 1):
        u = i / n
        q = K.ik('left', a[0] + (x - a[0]) * u, a[1] + (y - a[1]) * u, a[2] + (z - a[2]) * u, yaw, pitch, elbow=e)
        if q is None:
            raise RuntimeError(f'no IK on the line at u={u:.2f}')
        out.append(_vec(q))
    return out


def time_path(W, vmax):
    """TOPP-RA through the waypoints W (N x 6). vmax: 6 speed limits (lift m/s, joints rad/s). Returns Q at CTRL_HZ."""
    import toppra as ta, toppra.constraint as tc, toppra.algorithm as algo
    W = np.asarray(W, float)
    keep = [0] + [i for i in range(1, len(W)) if np.max(np.abs(W[i] - W[i - 1])) > 1e-7]
    W = W[keep]
    if len(W) < 2:
        return W
    if len(W) == 2:                                   # a spline needs 3 points
        W = np.array([W[0], (W[0] + W[1]) / 2, W[1]])
    ss = np.linspace(0.0, 1.0, len(W))
    path = ta.SplineInterpolator(ss, W)
    v = SPEED * np.asarray(vmax, float); acc = v / RAMP
    inst = algo.TOPPRA([tc.JointVelocityConstraint(np.stack([-v, v], 1)), tc.JointAccelerationConstraint(np.stack([-acc, acc], 1))],
                       path, gridpoints=np.linspace(0, 1, min(2000, 4 * len(W))), parametrizer='ParametrizeConstAccel')
    tr = inst.compute_trajectory(0.0, 0.0)
    if tr is None:
        raise RuntimeError('TOPP-RA found no timing')
    t = np.arange(1, int(np.ceil(tr.duration * CTRL_HZ)) + 1) / CTRL_HZ
    return tr(np.minimum(t, tr.duration))


def expand(steps, start, vmax):
    """Turn a mono plan into blended trajectories. start: dict of KEYS. Returns (plan, end vector)."""
    cur = _vec(start); plan = []; run, labels = [cur], []

    def flush():
        nonlocal run, labels
        if len(run) > 1:
            plan.append(('traj', ' + '.join(labels), time_path(run, vmax)))
        run, labels = [cur], []
    for st in steps:
        kind, label, tgt = st
        if kind == 'jaw' or (kind == 'line' and len(tgt) > 6):      # jaw actions and the timed (slow) lines: as before
            flush(); plan.append(st)
            if kind == 'line':
                cur = _line_points(cur, tgt)[-1]; run = [cur]
            continue
        if kind == 'joint':
            b = cur.copy()
            for k, v in tgt.items(): b[KEYS.index(k)] = v
            pts = _dense(cur, b)
        else:                                    # straight lines (approach / leave a product): own run, stop at both ends
            flush(); pts = _line_points(cur, tgt); run += pts; labels.append(label); cur = pts[-1]; flush()
            continue
        run += pts; labels.append(label); cur = pts[-1]
    flush()
    return plan, cur


def vmax_of(task):
    return [task.vmax[task.idx['lift']]] + [task.vmax[task.idx[f'left_{k}']] for k in KEYS[1:]]


def play(task, Q):
    """Send a blended trajectory tick by tick, then settle."""
    for q in Q:
        task.set_arm('left', dict(zip(KEYS, q))); task.step_once()
    task.settle(timeout=3.0)


def run(task, steps, jaw_cmd, jaw_wait, on_jaw=None):
    """Execute a plan in Isaac (proto_task.Task): blended runs tick by tick, the rest as mono_poses.run does."""
    plan, _ = expand(steps, task.arm_now('left'), vmax_of(task))
    for kind, label, tgt in plan:
        if kind == 'traj':
            play(task, tgt)
        elif kind == 'jaw':
            task.jaw('left', jaw_cmd[tgt], 1.0 if tgt == 2 else jaw_wait)
            if on_jaw: on_jaw(label)
        else:
            a = K.fk('left', task.arm_now('left'))
            task.cart_move('left', np.array([a[0], a[1], a[2], tgt[3], tgt[4]]), np.array(tgt[:5]), tgt[5], tgt[6])
