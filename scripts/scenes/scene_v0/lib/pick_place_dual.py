"""Both arms of the DUAL robot pick two products at the same time into the tote (scene_v0, pair stations of stations_dual.py).

    import pick_place_dual as PD
    PD.init(task, items, watch, st)          # after scene_robot.load(..., layout='prototype')
    ok, text = PD.run(pair_station)          # drive there, both arms pick, both drop into the tote, fold

Sequence (one shared lift, so every vertical move is shared; arms unfold / fold one at a time, as the dual table task:
unfolding together crosses the forearms):
    drive (both folded) -> unfold -> lift to the grip height -> both hands to READY (12 cm in front of the products,
    hand level) -> both straight in -> both jaws close -> lift 2 cm -> both straight out -> elbow-open pose -> lift down
    -> both hands down over their tote spots (proto_poses.TOTE2; the side-gripped products end up lying) -> release ->
    up -> fold.
Not collision-planned yet (the mono pick uses collide.py's planner; it has no two-arm model): moves are the tested dual
table task's pattern plus straight lines checked by the IK only.
"""
import math, os, sys, time
import numpy as np

import scene_robot as R
import grasp_plan_mono as G               # product sizes and grasp geometry (not its IK)
import proto_poses as PPo
import scara_ik as K
import pick_place_mono as PM              # restore(), START / MOVABLE bookkeeping, product_for(), NAMES

P = math.pi / 2
READY_GAP = 0.12
TOTE_DZ = 0.02                            # release this far above the lying product's resting height
UP = 0.02
HAND_DROP = 0.215                         # the TCP drops this much when the hand pitches from level to down (scara_ik)
LAYOUT = 'prototype'
TOTE_WALL = 0.376
TOTE_FLOOR = 0.263
SHALLOW_MAX = 0.020                       # a re-grasp may hold a product at most this much shallower than at release                        # tote floor top, base frame                         # tote wall top, base frame (URDF: tote at z 0.256, walls 0.12 high)
SIDES = ('left', 'right')
# ASYNC (2026-11-25, user: "as fast as mono ... asynchronous, not waiting for the other arm"): with the mounts at +-90 both
# arms unfold / fold together (checked offline, clear at lift 0.2 / 0.45 / 0.8), the lift moves while they do, and between
# shared vertical moves each arm runs its own chain of moves on its own clock (proto_task.Task.parallel). MM_DUAL_SYNC=1 =
# the old lock-step sequence (one arm at a time for the fold, every move waits for both arms).
ASYNC = os.environ.get('MM_DUAL_SYNC') != '1'
EMPTY_UP = 0.05                           # ASYNC: empty hands rise only this much over a release / re-grasp height (TCP),
EMPTY_CLEAR = TOTE_WALL + 0.06            # and at least this high (fingertips 12 mm below the TCP, ~5 cm over the walls)
TRAVEL_MAX = 0.50                         # ASYNC: the fold keeps the lift where it is, at most this high for driving (STOW
                                          # is clear at any lift; a lower centre of mass only for the high stops); the next
                                          # pick then starts near its grip height instead of from 0.20 (2026-11-25)
LIFT_SAFE = 0.20                          # ELBOW_OPEN touches the deck below this lift (urdf_selfcol); STOW is clear at any
task = items = watch = st = None
ROBOT = {'at': 'home'}
TRACE0 = {}
TRACE_NAMES = {}
CTX = {}                                  # the last pick, while both products are in the tote (for place_pair)                               # DUAL_DEBUG: product positions at the start of the pick


def init(task_, items_, watch_, st_mono, st_dual):
    """st_mono: the mono stations (product bookkeeping in pick_place_mono); st_dual: the pair stations."""
    global task, items, watch, st
    task, items, watch, st = task_, items_, dict(watch_), st_dual
    PM.init(task_, items_, watch_, st_mono, headless=True, out=None, workers=1)   # restore() / product_for() only
    K.use('prototype')                    # after PM.init: its (mono) planner switches the shared IK to the mono layout
    task.profile = 'trap'; task.settle_mode = 'still'
    task.settle_cap = PM.SETTLE_CAP; task.jaw_mode = 'stop'


def _both(fn):
    return {s: fn(s) for s in SIDES}


def J(q):
    return ('joint', q)


def unfold(lift=None, wide=0.0):
    """Out of the travel fold to ELBOW_OPEN, and the lift to `lift`. ASYNC: both arms at once with the lift moving too
    (above `wide` first if the hands swing out over a low obstacle; never below LIFT_SAFE while the arms open)."""
    if not ASYNC:
        if lift is not None: task.lift_to(lift)
        task.joint_move2(PPo.ELBOW_OPEN['left'], PPo.STOW['right'])
        task.joint_move2(PPo.ELBOW_OPEN['left'], PPo.ELBOW_OPEN['right'])
        return
    if wide > _lift_now():
        task.lift_to(min(lift, wide) if lift is not None else wide)
    tgt = None if lift is None else max(lift, LIFT_SAFE)
    task.parallel({s: [J(PPo.ELBOW_OPEN[s])] for s in SIDES}, lift=tgt)
    if lift is not None and abs(lift - _lift_now()) > 1e-4:
        task.lift_to(lift)


def _lift_now():
    return float(task.q_goal[task.idx['lift']])


def fold(wide_lift=0.0):
    """Back into the travel fold from over the tote: up to the height where the hands may swing out (hands still inside
    the outline), elbow-open, fold one arm at a time, then down (the fold is clear at any lift)."""
    if wide_lift > _lift_now():
        task.lift_to(wide_lift)
    if ASYNC:
        _fold_async(wide_lift)
        trace('folded', TRACE_NAMES)
        return
    task.joint_move2(PPo.ELBOW_OPEN['left'], PPo.ELBOW_OPEN['right'])
    trace('fold: elbow open', TRACE_NAMES)
    task.joint_move2(PPo.ELBOW_OPEN['left'], PPo.STOW['right'])
    task.joint_move2(PPo.STOW['left'], PPo.STOW['right'])
    task.lift_to(PPo.STOW['left']['lift'])
    trace('folded', TRACE_NAMES)


def _fold_async(wide_lift):
    """ELBOW_OPEN then STOW, each arm on its own clock; the lift goes down to the fold height meanwhile (once both hands are
    at ELBOW_OPEN when they swing out over a low obstacle, i.e. wide_lift > 0; never below LIFT_SAFE before they are in)."""
    down = min(max(_lift_now(), LIFT_SAFE), TRAVEL_MAX)
    if wide_lift > 0:
        task.parallel({s: [J(PPo.ELBOW_OPEN[s])] for s in SIDES})
        task.parallel({s: [J(PPo.STOW[s])] for s in SIDES}, lift=max(down, LIFT_SAFE))
    else:
        task.parallel({s: [J(PPo.ELBOW_OPEN[s]), J(PPo.STOW[s])] for s in SIDES}, lift=max(down, LIFT_SAFE))
    if abs(down - _lift_now()) > 1e-4:
        task.lift_to(down)


def trace(label, names):
    """DUAL_DEBUG=1: both products (base frame), each hand's TCP and jaw gap after a step."""
    if not os.environ.get('DUAL_DEBUG'):
        return
    p = R.positions(watch); q = task.robot.get_joint_positions()
    txt = []
    for s in SIDES:
        b = task.to_base(p[names[s]]); t = task.to_base(task.tcp[s].get_world_pose()[0])
        gap = (q[task.idx[f'{s}_finger_right']] + q[task.idx[f'{s}_finger_left']]) * 1000
        err = {k: math.degrees(q[task.idx[f'{s}_{k}']] - task.q_goal[task.idx[f'{s}_{k}']])
               for k in ('shoulder_pan', 'elbow_pan', 'wrist_pan', 'wrist_pitch', 'wrist_roll')}
        worst = max(err, key=lambda k: abs(err[k]))
        txt.append(f'{s[0].upper()} product ({b[0]:+.3f}, {b[1]:+.3f}, {b[2]:.3f}) tcp ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:.3f}) gap {gap:4.1f} '
                   f'err {worst} {err[worst]:+.1f} deg')
    moved = {n: d for n, d in R.knocked(watch, TRACE0).items() if n not in names.values()} if TRACE0 else {}
    print(f'      [trace] {label:22s} ' + ' | '.join(txt) + (f'  MOVED {[(n, round(d * 1000)) for n, d in moved.items()]}' if moved else ''), flush=True)


def _hand_path(side, q0, q1, n=16):
    """TCP (x, y) along the straight joint-space line q0 -> q1 (the path joint_move2 takes)."""
    pts = []
    for u in np.linspace(0, 1, n):
        q = {k: q0[k] + (q1[k] - q0[k]) * u for k in q0 if k in q1}
        x, y, _, _ = K.fk(side, q)
        pts.append((x, y))
    return pts


def back_out(side, pose, elbow, clear_x):
    """Joint pose of a waypoint behind `pose` (hand level, same height) with the hand at x <= clear_x (the held product
    out of the shelf), reached by a joint move whose hand path never goes forward (x) or inward (|y|)."""
    q0 = K.ik(side, *pose, elbow=elbow)
    sgn = 1 if side == 'left' else -1
    if pose[0] <= clear_x:
        return {**q0}
    for dx in np.arange(pose[0] - clear_x, 0.30, 0.02):
        for dy in np.arange(0.0, 0.25, 0.02):
            w = (pose[0] - dx, pose[1] + sgn * dy, pose[2], 0.0, 0.0)
            q1 = K.ik(side, *w, elbow=elbow)
            if not q1:
                continue
            path = _hand_path(side, q0, q1)
            if all(x <= pose[0] + 0.005 and sgn * y >= sgn * pose[1] - 0.005 for x, y in path):
                return {**q1, 'lift': q0['lift']}
    raise RuntimeError(f'{side}: no back-out waypoint behind {np.round(pose, 3)}')


def pick_pair(pid):
    S = st[pid]
    names = {s: PM.product_for(S[s]['station'])[0] for s in SIDES}
    for s in SIDES:
        watch[names[s]] = items[names[s]]['bodies'][0]
    print(f"\nDual pick: left {names['left']}, right {names['right']}, side grasps at z {S['grip_z']:.3f}", flush=True)
    if ROBOT['at'] != pid:
        R.leg(task, st, ROBOT['at'], pid); ROBOT['at'] = pid
    p0 = R.positions(watch)
    TRACE0.clear(); TRACE0.update(p0); TRACE_NAMES.clear(); TRACE_NAMES.update(names)
    c = {s: task.to_base(p0[names[s]])[:2] for s in SIDES}
    dims = {s: G.dims(items[names[s]]['pkg']) for s in SIDES}
    gz = S['grip_z']
    poses = {s: G.side_poses(c[s], 0.0, dims[s][1], gz, UP) for s in SIDES}           # (approach, grasp, lifted)
    # elbow: the station's, or the other one if the product now sits where only that one reaches (as planned, with the
    # products where they are now)
    el = {}
    for s in SIDES:
        for e in (S[s]['elbow'], -S[s]['elbow']):
            if all(K.ik(s, *q, elbow=e) for q in poses[s]):
                el[s] = e; break
        else:
            raise RuntimeError(f'{s}: product at ({c[s][0]:+.3f}, {c[s][1]:+.3f}) is out of reach from this station')

    def ready_for(s):
        """Back off from the approach start as far as READY_GAP while the arm still reaches (close to the robot the
        hand can't stay level at this sideways offset)."""
        a = poses[s][0]; best = a
        for back in np.arange(0.01, READY_GAP + 1e-6, 0.01):
            p = (a[0] - back, a[1], gz, 0.0, 0.0)
            if not (K.ik(s, *p, elbow=el[s]) and K.ik(s, p[0], p[1], gz + UP, 0.0, 0.0, elbow=el[s])):
                break
            best = p
        return best
    ready = {s: ready_for(s) for s in SIDES}
    out = {s: (ready[s][0], ready[s][1], gz + UP, 0.0, 0.0) for s in SIDES}
    task.report('ready poses', ', '.join(f'{s} {poses[s][0][0] - ready[s][0]:.2f} m in front of the approach' for s in SIDES))
    jopen = {s: min(0.045, dims[s][0] / 2 + 0.015) for s in SIDES}
    task.report('products seen at', ', '.join(f'{s} ({c[s][0]:+.3f}, {c[s][1]:+.3f})' for s in SIDES))

    # The lift moves only while both hands are inside the robot's outline (folded, over the tote) or straight in front of
    # the shelf; the wide elbow-open moves happen at or above wide_lift: beside a low obstacle (the table at the rack's
    # end) the hands swing out above it (stations_dual.side_min_z). Lifting with the hands out hit the table top.
    q_ready = {s: task.ik(s, ready[s], el[s]) for s in SIDES}
    tcp_off = gz - q_ready['left']['lift']                                # TCP height over the lift, hand level
    wide_lift = max(0.0, S.get('side_min_z', 0.0) - tcp_off)
    unfold(lift=max(q_ready['left']['lift'], wide_lift), wide=wide_lift)  # folded: clear at any lift
    trace('unfolded', names)
    task.joint_move2({**q_ready['left'], 'lift': _lift_now()}, {**q_ready['right'], 'lift': _lift_now()})   # both hands in front
    task.lift_to(q_ready['left']['lift'])
    trace('in front of products', names)
    if ASYNC:                             # each arm: (to the approach start), jaws fully open, straight in, close on it
        task.parallel({s: ([('cart', ready[s], poses[s][0], el[s])] if ready[s] is not poses[s][0] else []) +
                          [('jaw', jopen[s], 1.2), ('cart', poses[s][0], poses[s][1], el[s]), ('jaw', 0.0, 1.0)]
                       for s in SIDES})
    else:
        if any(ready[s] is not poses[s][0] for s in SIDES):
            task.cart_move2(ready['left'], poses['left'][0], el['left'], ready['right'], poses['right'][0], el['right'])
        task.set_jaw('right', jopen['right'])
        task.jaw('left', jopen['left'], 1.2); task.jaw('right', jopen['right'], 1.2)   # until both are open (0.8 s for 90 mm)
        task.cart_move2(poses['left'][0], poses['left'][1], el['left'], poses['right'][0], poses['right'][1], el['right'])
        task.jaw2(0.0, 1.0)
    trace('closed', names)
    z0 = {s: R.positions(watch)[names[s]][2] for s in SIDES}
    task.cart_move2(poses['left'][1], poses['left'][2], el['left'], poses['right'][1], poses['right'][2], el['right'])
    if not ASYNC:
        task.cart_move2(poses['left'][2], out['left'], el['left'], poses['right'][2], out['right'], el['right'])
    trace('out of the shelf', names)
    pz = R.positions(watch)
    for s in SIDES:
        q = task.robot.get_joint_positions()
        gap = (q[task.idx[f'{s}_finger_right']] + q[task.idx[f'{s}_finger_left']]) * 1000
        rise = (pz[names[s]][2] - z0[s]) * 1000 + UP * 1000
        task.report(f'{s}: pick', f'jaw gap {gap:.1f} mm (product {dims[s][0]*1000:.0f}), lifted {rise:.0f} mm',
                    ok=dims[s][0] * 500 < gap < dims[s][0] * 1000 + 8 and rise > 10)

    # to the tote: first each hand BACK out of the shelf (a joint move to a reachable waypoint whose hand path never
    # moves forward or inward), then elbow-open, down, hands down over the two tote spots. Swinging straight to
    # elbow-open slid the hands sideways along the board first, and a held box pushed the next product 7 cm.
    # the board front is about 40 mm in front of the products' near faces; the held product reaches deep - 30 mm ahead of
    # the TCP: the hand is out when that tip is 30 mm in front of the board
    front = min(c[s][0] - dims[s][1] / 2 - 0.04 for s in SIDES)
    wq = {s: back_out(s, out[s], el[s], front - (dims[s][1] - 0.03) - 0.03) for s in SIDES}
    chained = ASYNC and wide_lift <= 0
    if ASYNC and not chained:
        task.parallel({s: [('cart', poses[s][2], out[s], el[s]), J(wq[s])] for s in SIDES})
    elif chained:                         # out of the shelf, back out, elbow open, hands down: each arm on its own
        task.parallel({s: [('cart', poses[s][2], out[s], el[s]), J(wq[s]), J(PPo.ELBOW_OPEN[s]),
                           J({**PPo.ELBOW_OPEN[s], 'wrist_pitch': P})] for s in SIDES})
    else:
        task.joint_move2(wq['left'], wq['right'])
    trace('backed out', names)
    # holding the products, the hands swing out beside the robot and then pitch down there (the box turns from upright to
    # lying, its far end sweeping up to its full length below the TCP; pitching down also drops the TCP by HAND_DROP):
    # beside a low obstacle that all happens above it. Pitching during the swing in over the tote dragged the crate.
    if wide_lift > 0:
        reach = max(max(dims[s][1], dims[s][2]) for s in SIDES)
        wide_held = min(K.LIFT_MAX[LAYOUT], wide_lift + reach + HAND_DROP)
        if wide_held > _lift_now():
            task.lift_to(wide_held)
    if not chained:
        task.joint_move2(PPo.ELBOW_OPEN['left'], PPo.ELBOW_OPEN['right'])
        trace('elbow open', names)
        task.joint_move2({**PPo.ELBOW_OPEN['left'], 'wrist_pitch': P}, {**PPo.ELBOW_OPEN['right'], 'wrist_pitch': P})
    trace('hands down', names)
    T = PPo.TOTE2
    zt = {s: 0.263 + (dims[s][1] - 0.03) + TOTE_DZ for s in SIDES}           # lying: the approach depth is now vertical
    z = max(zt.values())
    # swing in over the tote with the held boxes clear of its walls (top z TOTE_WALL) in ANY orientation: the hand pitches
    # from level to down during the swing, so a box can hang its full length below the TCP (at z + 0.08 they caught
    # on the side walls and pinned both arms outside the tote, shoulders 25-30 deg short)
    zs = max(z + 0.08, TOTE_WALL + 0.03 + max(max(dims[s][1], dims[s][2]) for s in SIDES))
    hi = {s: (T[s]['x'], T[s]['y'], zs, T[s]['yaw'], P) for s in SIDES}
    lo = {s: (T[s]['x'], T[s]['y'], z, T[s]['yaw'], P) for s in SIDES}
    q_hi = {s: task.ik(s, hi[s], T[s]['elbow']) for s in SIDES}
    lift_hi = max(q_hi['left']['lift'], _lift_now())                      # over the tote first (still high), then down
    task.joint_move2({**q_hi['left'], 'lift': lift_hi}, {**q_hi['right'], 'lift': lift_hi})
    trace('over the tote', names)
    task.lift_to(q_hi['left']['lift'])
    trace('lift down', names)
    task.cart_move2(hi['left'], lo['left'], T['left']['elbow'], hi['right'], lo['right'], T['right']['elbow'])
    trace('lowered', names)
    held = {s: PM.pose_b(items[names[s]]['bodies'][0]) for s in SIDES}       # product pose in the hand, just before release
    if ASYNC:
        task.parallel({s: [('jaw', 0.04, 0.8)] for s in SIDES})
    else:
        task.jaw2(0.04, 0.8)
    trace('released', names)
    task.wait(0.3)
    if ASYNC:                             # empty hands: up just clear of the product and the walls (not back to the carry
        ze = min(max(z + EMPTY_UP, EMPTY_CLEAR), hi['left'][2])          # height the boxes needed going in)
        up_ = {s: (lo[s][0], lo[s][1], ze, lo[s][3], lo[s][4]) for s in SIDES}
        task.cart_move2(lo['left'], up_['left'], T['left']['elbow'], lo['right'], up_['right'], T['right']['elbow'])
    else:
        task.cart_move2(lo['left'], hi['left'], T['left']['elbow'], lo['right'], hi['right'], T['right']['elbow'])
    task.jaw2(0.0, 0.5)
    x0, x1, y0, y1 = PPo.TOTE_INNER
    for s in SIDES:
        cb = task.to_base(R.positions(watch)[names[s]])
        task.report(f'{s}: in the tote', f'product at ({cb[0]:+.3f}, {cb[1]:+.3f}, {cb[2]:.3f}) in the base frame',
                    ok=x0 < cb[0] < x1 and y0 < cb[1] < y1 and cb[2] < 0.40)
    fold_lift = min(K.LIFT_MAX[LAYOUT], wide_lift + HAND_DROP) if wide_lift > 0 else 0.0
    fold(fold_lift)                                                       # the hands leave the tote pitched down
    CTX.clear()
    CTX.update(pid=pid, names=names, held=held, lo=lo, hi=hi, T=T, poses=poses, ready=ready, out=out, el=el, jopen=jopen,
               wq=wq, wide_lift=wide_lift, fold_lift=fold_lift, lift_hi=lift_hi, gz=gz, dims=dims)
    moved = {n: d for n, d in R.knocked(watch, p0).items() if n not in names.values()}
    task.report('nothing else moved', ', '.join(f'{n} {d*1000:.0f} mm' for n, d in moved.items()) or 'none over 10 mm', ok=not moved)


def run(pid):
    n0 = len(task.results)
    CTX.clear()
    back = PM.restore()
    if back:
        print(f'      {back} product(s) put back where they started', flush=True)
    t0 = task.t
    try:
        pick_pair(pid)
    except RuntimeError as err:
        task.report('aborted', str(err), ok=False)
    res = [r for r in task.results[n0:] if not r[0].startswith('  via')]
    ok = bool(res) and all(o for _, o, _ in res)
    bad = [f'{a}: {t}' for a, o, t in res if not o]
    return ok, ('OK' if ok else bad[0] if bad else 'no result') + f' ({task.t - t0:.0f} s)'


def _yaw(Rm):
    return math.atan2(Rm[1, 0], Rm[0, 0])


def place_pair():
    """Both arms take their products out of the tote and put them back on their spots on the shelf, at the same time:
    the pick's path backwards. Each re-grasp follows its product's shift since release (position and turn about the
    vertical, as pick_place_mono's place back); set down 5 mm over the grasp height."""
    C = dict(CTX)
    pid, names, T = C['pid'], C['names'], C['T']
    S = st[pid]
    if ROBOT['at'] != pid:
        R.leg(task, st, ROBOT['at'], pid); ROBOT['at'] = pid
    p0 = R.positions(watch)
    TRACE0.clear(); TRACE0.update(p0); TRACE_NAMES.clear(); TRACE_NAMES.update(names)
    lo2 = {}
    for s in SIDES:
        (p_rel, R_rel), (p_now, R_now) = C['held'][s], PM.pose_b(items[names[s]]['bodies'][0])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, (R_now @ R_rel.T)[2, 2]))))
        if tilt > 15:
            raise RuntimeError(f'{s}: the product tipped over in the tote ({tilt:.0f} deg): put it back by hand ("Put products back")')
        dyaw = _yaw(R_now @ R_rel.T)
        lo = C['lo'][s]
        lo2[s] = (lo[0] + p_now[0] - p_rel[0], lo[1] + p_now[1] - p_rel[1], lo[2] + p_now[2] - p_rel[2],
                  lo[3] + dyaw, lo[4])
        task.report(f'{s}: in the tote', f'slid {np.hypot(p_now[0] - p_rel[0], p_now[1] - p_rel[1]) * 1000:.0f} mm, turned '
                    f'{math.degrees(dyaw):+.0f} deg, dropped {(p_rel[2] - p_now[2]) * 1000:.0f} mm since release')
    # one lift: both hands at one height. The products dropped different distances (different thicknesses), so grip at
    # the lower one's height (the other's jaws just go deeper round it), never with the fingertips (12 mm below the
    # TCP) closer than 5 mm to the tote floor
    # Gripped lower than its own resting height, a product sits deeper in the hand and must go further into the shelf;
    # gripped higher, shallower (at most SHALLOW_MAX: the jaws keep >= 2 cm of it). Lowest shared height first (the best
    # grip on the lower product), raised until both set-down poses are reachable.
    po, el = C['poses'], C['el']
    ideal = {s: lo2[s][2] for s in SIDES}
    zc = deeper = None
    for z in np.arange(max(TOTE_FLOOR + 0.017, min(ideal.values())), max(ideal.values()) + 1e-6, 0.002):
        d = {s: ideal[s] - z for s in SIDES}
        if min(d.values()) < -SHALLOW_MAX:
            break
        if all(K.ik(s, po[s][2][0] + d[s], po[s][2][1], po[s][2][2], 0.0, 0.0, elbow=el[s]) and
               K.ik(s, po[s][1][0] + d[s], po[s][1][1], po[s][1][2] + 0.005, 0.0, 0.0, elbow=el[s]) for s in SIDES):
            zc, deeper = float(z), d
            break
    if zc is None:
        raise RuntimeError('no shared re-grasp height lets both arms set their products down (they rest '
                           f'{abs(ideal["left"] - ideal["right"]) * 1000:.0f} mm apart in height)')
    lo2 = {s: (lo2[s][0], lo2[s][1], zc, lo2[s][3], lo2[s][4]) for s in SIDES}
    hi2 = {s: (lo2[s][0], lo2[s][1], C['hi'][s][2], lo2[s][3], P) for s in SIDES}
    el_t = {s: T[s]['elbow'] for s in SIDES}
    q_hi2 = {s: task.ik(s, hi2[s], el_t[s]) for s in SIDES}
    if ASYNC:                             # empty hands go in just over the walls, not at the carry height
        # over BOTH products' tops: each rests at its own height (the shared grip zc is the lower one); a held product's
        # top sits ~2 cm above the TCP and the fingertips 12 mm below it (at zc + 5 cm the right hand swept the cracker
        # box, which rested 20 mm higher, out of its place)
        ze = min(max(max(ideal.values()) + 0.02 + 0.012 + EMPTY_UP, EMPTY_CLEAR), hi2['left'][2])
        in2 = {s: (lo2[s][0], lo2[s][1], ze, lo2[s][3], P) for s in SIDES}
        q_in2 = {s: task.ik(s, in2[s], el_t[s]) for s in SIDES}
    else:
        in2, q_in2 = hi2, q_hi2

    # out of the fold and over the tote (the pick's way back, forwards): up, elbow-open, hands down beside the robot, in
    unfold(lift=max(C['fold_lift'], q_in2['left']['lift']), wide=C['wide_lift'])
    task.set_jaw('left', 0.04); task.set_jaw('right', 0.04)
    if ASYNC:
        task.parallel({s: [J({**PPo.ELBOW_OPEN[s], 'wrist_pitch': P}), J(q_in2[s])] for s in SIDES})
    else:
        task.joint_move2({**PPo.ELBOW_OPEN['left'], 'wrist_pitch': P}, {**PPo.ELBOW_OPEN['right'], 'wrist_pitch': P})
        task.joint_move2(q_hi2['left'], q_hi2['right'])
    task.lift_to(q_in2['left']['lift'])
    trace('over the tote', names)
    task.cart_move2(in2['left'], lo2['left'], el_t['left'], in2['right'], lo2['right'], el_t['right'])
    if ASYNC:
        task.parallel({s: [('jaw', 0.0, 1.0)] for s in SIDES})
    else:
        task.jaw2(0.0, 1.0)
    z0 = {s: R.positions(watch)[names[s]][2] for s in SIDES}
    task.cart_move2(lo2['left'], hi2['left'], el_t['left'], lo2['right'], hi2['right'], el_t['right'])
    pz = R.positions(watch); q = task.robot.get_joint_positions()
    for s in SIDES:
        gap = (q[task.idx[f'{s}_finger_right']] + q[task.idx[f'{s}_finger_left']]) * 1000
        rise = (pz[names[s]][2] - z0[s]) * 1000
        task.report(f'{s}: picked from the tote', f'jaw gap {gap:.1f} mm, lifted {rise:.0f} mm',
                    ok=C['dims'][s][0] * 500 < gap < C['dims'][s][0] * 1000 + 8 and rise > 40)
    trace('re-grasped', names)

    # back to the shelf: up, out beside the robot (hands down), hands level, back-out waypoint, down, in, set down
    task.lift_to(C['lift_hi'])
    if ASYNC:
        task.parallel({s: [J({**PPo.ELBOW_OPEN[s], 'wrist_pitch': P}), J(PPo.ELBOW_OPEN[s])] +
                          ([J(C['wq'][s])] if C['wide_lift'] <= 0 else []) for s in SIDES})
        if C['wide_lift'] > 0:
            task.parallel({s: [J(C['wq'][s])] for s in SIDES})
    else:
        task.joint_move2({**PPo.ELBOW_OPEN['left'], 'wrist_pitch': P}, {**PPo.ELBOW_OPEN['right'], 'wrist_pitch': P})
        task.joint_move2(PPo.ELBOW_OPEN['left'], PPo.ELBOW_OPEN['right'])
        trace('beside the robot', names)
        task.joint_move2(C['wq']['left'], C['wq']['right'])
    task.lift_to(C['wq']['left']['lift'])
    q_out = {s: task.ik(s, C['out'][s], C['el'][s]) for s in SIDES}
    task.joint_move2(q_out['left'], q_out['right'])
    trace('in front of the shelf', names)
    # the lying-in-the-tote depth axis is the shelf's in / out axis: a product held deeper goes in that much further
    up = {s: (po[s][2][0] + deeper[s], po[s][2][1], po[s][2][2], 0.0, 0.0) for s in SIDES}
    down = {s: (po[s][1][0] + deeper[s], po[s][1][1], po[s][1][2] + 0.005, 0.0, 0.0) for s in SIDES}
    task.report('re-grasp offsets', ', '.join(f'{s} {deeper[s] * 1000:+.0f} mm (+ = deeper in the hand)' for s in SIDES))
    if ASYNC:
        task.parallel({s: [('cart', C['out'][s], up[s], el[s])] for s in SIDES})
    else:
        task.cart_move2(C['out']['left'], up['left'], el['left'], C['out']['right'], up['right'], el['right'])
    task.cart_move2(up['left'], down['left'], el['left'], up['right'], down['right'], el['right'])
    if ASYNC:                             # each arm: open on the shelf; the 5 mm set-down margin back (shared lift); then
        task.parallel({s: [('jaw', C['jopen'][s], 1.0)] for s in SIDES})      # each arm straight out, (ready), close
        d0 = {s: (down[s][0], down[s][1], po[s][0][2], 0.0, 0.0) for s in SIDES}
        task.cart_move2(down['left'], d0['left'], el['left'], down['right'], d0['right'], el['right'])
        task.parallel({s: [('cart', d0[s], po[s][0], el[s])] +
                          ([('cart', po[s][0], C['ready'][s], el[s])] if C['ready'][s] is not po[s][0] else []) +
                          [('jaw', 0.0, 0.5)] for s in SIDES})
        task.wait(0.3)
    else:
        task.set_jaw('right', C['jopen']['right'])
        task.jaw('left', C['jopen']['left'], 1.0); task.jaw('right', C['jopen']['right'], 1.0)   # until both jaws are open
        trace('set down', names)
        task.cart_move2(down['left'], po['left'][0], el['left'], down['right'], po['right'][0], el['right'])
        if any(C['ready'][s] is not po[s][0] for s in SIDES):
            task.cart_move2(po['left'][0], C['ready']['left'], el['left'], po['right'][0], C['ready']['right'], el['right'])
        task.jaw2(0.0, 0.5)
        task.wait(0.5)
    for s in SIDES:
        n = names[s]
        pf, Rf = PM.pose_b(items[n]['bodies'][0])
        p_start = task.to_base(PM.START[n][0][0]); R_start = PM.pose_b(None, PM.START[n][0])[1]
        dxy = np.linalg.norm(pf[:2] - p_start[:2]) * 1000
        tl = math.degrees(math.acos(max(-1.0, min(1.0, (Rf @ R_start.T)[2, 2]))))
        dyw = math.degrees(_yaw(Rf @ R_start.T))
        task.report(f'{s}: back on its spot', f'{dxy:.0f} mm from where it stood, turned {dyw:+.0f} deg, tilted {tl:.1f} deg',
                    ok=dxy < 20 and tl < 5)
    if C['wide_lift'] > _lift_now():
        task.lift_to(C['wide_lift'])                                       # hands in front of the shelf: up, then fold
    if ASYNC:
        _fold_async(C['wide_lift'])
    else:
        task.joint_move2(PPo.ELBOW_OPEN['left'], PPo.ELBOW_OPEN['right'])
        task.joint_move2(PPo.ELBOW_OPEN['left'], PPo.STOW['right'])
        task.joint_move2(PPo.STOW['left'], PPo.STOW['right'])
        task.lift_to(PPo.STOW['left']['lift'])
    trace('folded', names)
    moved = {n: d for n, d in R.knocked(watch, p0).items() if n not in names.values()}
    task.report('nothing else moved', ', '.join(f'{n} {d*1000:.0f} mm' for n, d in moved.items()) or 'none over 10 mm', ok=not moved)


def run_place():
    """put both products of the last pick back; returns (ok, text)"""
    if not CTX:
        return False, 'nothing to place back (pick a pair first)'
    n0 = len(task.results); t0 = task.t
    try:
        place_pair()
    except RuntimeError as err:
        task.report('aborted', str(err), ok=False)
    CTX.clear()
    res = [r for r in task.results[n0:] if not r[0].startswith('  via')]
    ok = bool(res) and all(o for _, o, _ in res)
    bad = [f'{a}: {t}' for a, o, t in res if not o]
    return ok, ('OK' if ok else bad[0] if bad else 'no result') + f' ({task.t - t0:.0f} s)'


def reset_robot():
    task.set_pose({**R.stow_joints('prototype'), **R._home_joints(task)})
    R._place_home(task)
    ROBOT['at'] = 'home'
    task.wait(0.5)
