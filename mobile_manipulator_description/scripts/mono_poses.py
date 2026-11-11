"""Poses for the one-arm experiment tasks (layout prototype_mono_arm):
  isaac_mono_task.py        stationary: table in front -> tote behind -> table
  isaac_mono_table_task.py  the prototype table-to-table task with one arm (2 products, drive between tables)
  isaac_mono_shelf_task.py  the prototype shelf task with one arm (1 product, drive between shelves)
Pure Python (scara_ik only), so the offline collision check and the Isaac task use exactly the same plan.

Base frame (base_link), robot parked. The shoulder ring is on the column axis (0.18, 0); its zero pose points forward.
Spots come from reach_test.py --layout prototype_mono_arm (hand down, any of 8 headings, either elbow).
"""
import math
import scara_ik as K

K.use('prototype_mono_arm')
D = math.radians
P = math.pi / 2
ITEM_H = 0.070                       # 45 x 45 x 70 mm product, grasped at its middle
TABLE_EDGE = 0.30                    # table front edge, m in front of the base centre (bumper at 0.259)
PICK = (0.375, 0.09)                 # on the table
PLACE = (0.375, -0.09)               # on the table, the second spot
TOTE = (-0.073, 0.06)                # in the tote behind the column (floor top 0.263 m)
TOTE_FLOOR = 0.263
CLEAR = 0.005                        # put the product down 5 mm above the surface
YAWS = [k * math.pi / 4 for k in range(8)]


def tcp_z(surface):
    return surface + ITEM_H / 2 + CLEAR


def best(x, y, z, prefer=None):
    """IK for a hand-down pose: the heading / elbow whose joints are closest to `prefer` (a joint dict), or the first one."""
    sols = []
    for yaw in YAWS:
        for e in (1, -1):
            q = K.ik('left', x, y, z, yaw, P, elbow=e)
            if q is not None:
                sols.append((yaw, e, q))
    if not sols:
        raise RuntimeError(f'no IK for ({x:.3f}, {y:.3f}, {z:.3f})')
    if prefer is None:
        return sols[0]
    keys = ('shoulder_pan', 'elbow_pan', 'wrist_pan')
    return min(sols, key=lambda s: sum(abs(s[2][k] - prefer[k]) for k in keys))


def plan(table_h=0.75, lift_max=0.77):
    """List of (kind, label, target): kind 'joint' = time-synchronised joint move to a joint dict, 'line' = straight TCP
    line to (x, y, z, yaw, pitch, elbow), 'jaw' = open (1) / close (0)."""
    up = min(0.12, lift_max + K.Z0 - K.TOOL - tcp_z(table_h) - 0.005)       # vertical approach the lift allows
    zt, zb = tcp_z(table_h), tcp_z(TOTE_FLOOR)
    hi = zt + up                                                              # carry height (over the table and the tote)
    steps = []
    yaw_a, e_a, q_a = best(PICK[0], PICK[1], hi)
    steps += [('joint', 'over the product', q_a), ('jaw', 'open', 1),
              ('line', 'down to the product', (PICK[0], PICK[1], zt, yaw_a, P, e_a)), ('jaw', 'grasp', 0),
              ('line', 'lift it', (PICK[0], PICK[1], hi, yaw_a, P, e_a))]
    yaw_t, e_t, q_t = best(TOTE[0], TOTE[1], hi, prefer=q_a)
    steps += [('joint', 'swing round to the tote', q_t),
              ('line', 'down into the tote', (TOTE[0], TOTE[1], zb, yaw_t, P, e_t)), ('jaw', 'release in the tote', 1),
              ('line', 'up out of the tote', (TOTE[0], TOTE[1], hi, yaw_t, P, e_t)),
              ('line', 'down to the product again', (TOTE[0], TOTE[1], zb, yaw_t, P, e_t)), ('jaw', 'grasp in the tote', 0),
              ('line', 'lift it out', (TOTE[0], TOTE[1], hi, yaw_t, P, e_t))]
    yaw_p, e_p, q_p = best(PLACE[0], PLACE[1], hi, prefer=q_t)
    steps += [('joint', 'swing back to the table', q_p),
              ('line', 'down to the table', (PLACE[0], PLACE[1], zt, yaw_p, P, e_p)), ('jaw', 'release on the table', 1),
              ('line', 'up', (PLACE[0], PLACE[1], hi, yaw_p, P, e_p))]
    return steps, dict(up=up, hi=hi, zt=zt, zb=zb)


# ------------------------------------------------------------------ the ported prototype tasks
def _q(a, b, c, p, lift=None):
    q = dict(shoulder_pan=D(a), elbow_pan=D(b), wrist_pan=D(c), wrist_pitch=D(p), wrist_roll=0.0)
    if lift is not None: q['lift'] = lift
    return q


STOW = _q(-80, -100, -30, 0, 0.20)    # travel: hand level, x -0.17..0.29 (camera mast) y -0.17..0.08, clear at any lift
TABLE = dict(edge=0.36, spots=[(0.40, 0.10), (0.40, -0.10)])        # as the prototype table task: edge 0.36 m out
TOTE_SPOTS = [(-0.073, 0.09), (-0.073, -0.09)]                      # two upright products, 0.18 m apart
TOTE_UP = 0.12
JAW_WAIT = 0.6                         # s per open / close (was 0.8; the prototype tasks use 0.6-0.8)
JAW_OPEN = 0.035                       # each jaw (70 mm for the 45 mm product): at 30 mm the wedge tips clip its edges


def table_steps(table_h=0.75, lift_max=0.77, order=(0, 1), to_tote=True, src=None, dst=None, fast=True):
    """Products table spot k -> tote spot k (to_tote) or back, in `order`, starting and ending in STOW. src / dst: the
    measured product positions / target spots (base frame, indexed like the spots) instead of the planned ones.
    fast: the lift moves during the swings (down into the tote while swinging to it, up while swinging back), and the
    hand only rises TOTE_UP out of the tote; the table side keeps 'lift first, then swing over it' (checked offline)."""
    up = min(0.12, lift_max + K.Z0 - K.TOOL - tcp_z(table_h) - 0.005)
    zt, zb = tcp_z(table_h), tcp_z(TOTE_FLOOR)
    hi = zt + up                                  # over the table
    ht = zb + TOTE_UP                             # over the tote (fast)
    steps, cur = [], STOW
    for k in order:
        a = (src or (TABLE['spots'] if to_tote else TOTE_SPOTS))[k]
        b = (dst or (TOTE_SPOTS if to_tote else TABLE['spots']))[k]
        za, zb_ = (zt, zb) if to_tote else (zb, zt)
        ha, hb = (hi, ht if fast else hi) if to_tote else (ht if fast else hi, hi)
        ya, ea, qa = best(a[0], a[1], ha, prefer=cur)
        if to_tote or not fast or k == order[0]:
            steps += [('joint', 'lift up', {'lift': qa['lift']}), ('joint', f'over product {k + 1}', {**qa})]
        else:                                     # leaving the table: swing away first (at table height), then lower
            _, _, qh = best(a[0], a[1], hi, prefer=cur)
            steps += [('joint', 'swing off the table', qh), ('joint', f'over product {k + 1}', {**qa})]
        steps += [('jaw', 'open', 1),
                  ('line', f'down to product {k + 1}', (a[0], a[1], za, ya, P, ea)), ('jaw', f'grasp {k + 1}', 0),
                  ('line', f'lift product {k + 1}', (a[0], a[1], ha, ya, P, ea))]
        yb, eb, qb = best(b[0], b[1], hb, prefer=qa)
        dest = 'tote' if to_tote else 'table'
        steps += [('joint', f'swing to the {dest}' + (', lift moving too' if fast else ''), qb),
                  ('line', f'down to the {dest}', (b[0], b[1], zb_, yb, P, eb)),
                  ('jaw', f'release {k + 1} {dest}', 1), ('line', 'up', (b[0], b[1], hb, yb, P, eb)), ('jaw', 'close', 0)]
        cur = qb
    steps += [('joint', 'swing to the stow pose', {**STOW, 'lift': cur['lift']}), ('joint', 'lower to travel height', {'lift': STOW['lift']})]
    return steps


SHELF = dict(reach=(0.64, 0.10), approach=0.08, grasp_dz=-0.003, release=0.04, open=0.04)   # product relative to the parked base
SLOW_IN = 2.5     # s for the 80 mm straight moves into / out of a shelf: at the default ~0.6 s the elbow / wrist servos lag
                  # ~8 mm sideways and an open fingertip (40 mm jaws) hits the 45 mm product's corner (Isaac contact report)
TOTE_FLAT = (-0.043, 0.0)              # the shelf product is laid flat here (hand down)
ZT_FLAT = 0.263 + 0.045 / 2 + 0.008    # TCP height that sets a lying 45 mm item 8 mm above the tote floor


def level_ik(x, y, z, prefer):
    """Hand level pointing +x (into the shelf): the elbow closest to `prefer`."""
    sols = [(e, K.ik('left', x, y, z, 0.0, 0.0, elbow=e)) for e in (1, -1)]
    sols = [(e, q) for e, q in sols if q is not None]
    if not sols: raise RuntimeError(f'no level IK at ({x:.3f}, {y:.3f}, {z:.3f})')
    keys = ('shoulder_pan', 'elbow_pan', 'wrist_pan')
    return min(sols, key=lambda s_: sum(abs(s_[1][k] - prefer[k]) for k in keys))


def shelf_pick_steps(item):
    """Product at `item` (x, y, z centre, base frame) on a shelf: level grasp straight in, lift 30 mm, out, lay it in the tote."""
    x, y, z = item; g = (x, y, z + SHELF['grasp_dz']); A = SHELF['approach']
    e, qpre = level_ik(x - A, y, g[2], STOW)
    steps = [('jaw', 'open wide', 3), ('joint', 'lift to the shelf', {'lift': qpre['lift']}), ('joint', 'in front of the product', qpre),
             ('line', 'straight in', (x, y, g[2], 0.0, 0.0, e, SLOW_IN)), ('jaw', 'grasp', 0),
             ('line', 'lift 30 mm', (x, y, g[2] + 0.03, 0.0, 0.0, e)), ('line', 'straight out', (x - A, y, g[2] + 0.03, 0.0, 0.0, e, SLOW_IN))]
    hi = ZT_FLAT + TOTE_UP
    yt, et, qt = best(TOTE_FLAT[0], TOTE_FLAT[1], hi, prefer=qpre)
    steps += [('joint', 'swing to the tote, hand down', qt), ('line', 'down into the tote', (TOTE_FLAT[0], TOTE_FLAT[1], ZT_FLAT, yt, P, et)),
              ('jaw', 'release in the tote', 1), ('line', 'up', (TOTE_FLAT[0], TOTE_FLAT[1], hi, yt, P, et)), ('jaw', 'close', 0),
              ('joint', 'swing to the stow pose', {**STOW, 'lift': qt['lift']}), ('joint', 'lower to travel height', {'lift': STOW['lift']})]
    return steps, yt, et


def shelf_place_steps(target, grab_yaw, grab_elbow=None, item_in_tote=None, item_z=None):
    """Pick the lying product in the tote (top down, along its axis) and place it on a shelf at `target` (centre)."""
    tx, ty = item_in_tote or TOTE_FLAT
    hi = ZT_FLAT + TOTE_UP
    # grasp across the lying product (its axis = grab_yaw, either end) with the joints closest to STOW: the swing from
    # STOW then stays away from the shelf in front (elbow +1 at 225 deg swept the shoulder through the front, 2026-11-10)
    keys = ('shoulder_pan', 'elbow_pan', 'wrist_pan')
    sols = [(yw, e, K.ik('left', tx, ty, hi, yw, P, elbow=e)) for yw in (grab_yaw, grab_yaw + math.pi) for e in (1, -1)]
    sols = [s_ for s_ in sols if s_[2] is not None]
    if not sols: raise RuntimeError('no IK over the tote item')
    grab_yaw, grab_elbow, q = min(sols, key=lambda s_: sum(abs(s_[2][k] - STOW[k]) for k in keys))
    zg = item_z if item_z is not None else ZT_FLAT
    steps = [('jaw', 'open', 1), ('joint', 'lift up', {'lift': q['lift']}), ('joint', 'over the product in the tote', q),
             ('line', 'down to it', (tx, ty, zg, grab_yaw, P, grab_elbow)), ('jaw', 'grasp', 0),
             ('line', 'lift it', (tx, ty, hi, grab_yaw, P, grab_elbow))]
    x, y, z = target; g = (x, y, z + SHELF['grasp_dz']); A = SHELF['approach']
    e, qpre = level_ik(x - A, y, g[2] + 0.02, q)
    # via the fold (hand level, away from the shelf): a direct swing from the tote sweeps the carried product through
    # the shelf's lower board (2026-11-10 offline check)
    steps += [('joint', 'fold, hand level', {**STOW, 'lift': q['lift']}), ('joint', 'raise to the shelf', {'lift': max(q['lift'], qpre['lift'])}),
              ('joint', 'in front of the shelf', qpre),
              ('line', 'straight in, 20 mm high', (x, y, g[2] + 0.02, 0.0, 0.0, e, SLOW_IN)), ('line', 'down onto the board', (x, y, g[2], 0.0, 0.0, e)),
              ('jaw', 'release wide', 2), ('line', 'straight out', (x - A, y, g[2], 0.0, 0.0, e, SLOW_IN)), ('jaw', 'close', 0),
              ('joint', 'swing to the stow pose', {**STOW, 'lift': qpre['lift']}), ('joint', 'lower to travel height', {'lift': STOW['lift']})]
    return steps


def run(task, steps, jaw_open=JAW_OPEN, on_jaw=None, debug=None, smooth=False):
    """Execute a step list on a proto_task.Task (Isaac). on_jaw(label) is called after every jaw action;
    debug(label) after every step (MONO_DEBUG=1 prints jaw opening + TCP)."""
    import numpy as np, os
    if smooth:                       # TOPP-RA: blended, time-optimal runs between the jaw actions (smooth_path.py)
        import smooth_path as SP
        return SP.run(task, steps, {1: jaw_open, 0: 0.0, 2: SHELF['release'], 3: SHELF['open']}, JAW_WAIT, on_jaw)
    if debug is None and os.environ.get('MONO_DEBUG'):
        def debug(label):
            q = task.robot.get_joint_positions(); t = task.to_base(task.tcp['left'].get_world_pose()[0])
            gap = (q[task.idx['left_finger_right']] + q[task.idx['left_finger_left']]) * 1000
            cmd = (task.q_goal[task.idx['left_finger_right']] + task.q_goal[task.idx['left_finger_left']]) * 1000
            print(f'      [debug] {label:28s} jaw gap {gap:5.1f} mm (cmd {cmd:4.0f})  TCP ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:.3f})', flush=True)
    for kind, label, tgt in steps:
        if kind == 'jaw':
            task.jaw('left', {1: jaw_open, 0: 0.0, 2: SHELF['release'], 3: SHELF['open']}[tgt], 1.0 if tgt == 2 else JAW_WAIT)
            if on_jaw: on_jaw(label)
        elif kind == 'joint':
            task.joint_move('left', tgt, lift='lift' in tgt)
        else:
            a = K.fk('left', task.arm_now('left'))
            task.cart_move('left', np.array([a[0], a[1], a[2], tgt[3], tgt[4]]), np.array(tgt[:5]), tgt[5], tgt[6] if len(tgt) > 6 else None)
        if debug: debug(label)
