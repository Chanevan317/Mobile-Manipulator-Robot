"""Robot stations for the DUAL-arm robot in scene_v0: one base pose per PAIR of task products, the left arm picking one,
the right arm the other, at the same time (user 2026-11-20: "always showcase dual working in parallel").

    python scripts/scenes/scene_v0/stations_dual.py        # no simulator needed; writes scenes/scene_v0/stations_dual.json

Why pairs on one board: both arms ride on ONE carriage, so the two grasps must be at the same height (one lift), and each
arm reaches only its own half in front (scara_ik 'prototype': 0.10 .. 0.35 m from the centre line, 0.55 .. 0.65 m ahead
with the hand level), so the two products must be 0.2 .. 0.7 m apart. Side grasps at a COMMON grip height (the lower of
the two products' usual ones). The mono stations (stations_mono.py) give the task products' positions; this file uses
their footprint check, approach points and route (the dual base is the same 0.45 x 0.45 since 2026-11-25; it was 0.50 with this 0.45 check).
Checked: kinematics (both arms, approach / grasp / lift poses, joint limits) and the footprint. NOT checked yet: the arms
against the furniture, the products and each other (the dual pick script's job).
"""
import itertools, json, math, os, sys
import numpy as np
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
sys.path.insert(0, here)
import stations_mono as SM
import assets as SA
import scene as SC
import grasp_plan_mono as G                 # dims, side_poses, grip_heights (geometry only; the IK is done here per arm)
import scara_ik as K

LAYOUT = 'prototype'
OUT = os.path.join(RP.scene_dir('scene_v0'), 'stations_dual.json')
GAP = (0.12, 0.75)                          # m between the two products' centres (0.20 until the mounts turned to +-90, 2026-11-25)
DZ = 0.02                                   # lift after grasping (as the mono stations)
HUB = SM.HUB
SIDE_ENV = (-0.075, 0.225, 0.25, 0.47)        # base frame (x0, x1, |y| from, |y| to): where a hand goes in the elbow-open pose
SIDE_CLEAR = 0.06                           # wide moves keep the TCP this far above a low obstacle there (the hand's bottom)                                # the aisle middle (scene_robot.leg labels it)


def _ok(side, poses):
    for e in (1, -1):
        if all(K.ik(side, *p, elbow=e) for p in poses):
            return e
    return None


MARGIN_XY = 0.01                            # the poses must also solve with the product this far off in x / y


def _ok_margin(side, p, deep, gz):
    """Elbow that reaches the side grasp of a product at p AND with it MARGIN_XY off in every direction (a product
    put back a few mm off, or jittered, must not fall off the edge of the reach)."""
    for e in (1, -1):
        if all(all(K.ik(side, *q, elbow=e) for q in G.side_poses((p[0] + dx, p[1] + dy), 0.0, deep, gz, DZ))
               for dx in (0.0, MARGIN_XY, -MARGIN_XY) for dy in (0.0, MARGIN_XY, -MARGIN_XY)):
            return e
    return None


def _to_base(p, bx, by, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = p[0] - bx, p[1] - by
    return (c * dx + s * dy, -s * dx + c * dy)


def _basket_hits(box):
    """Baskets / crates (on the table: not in the floor obstacle map) whose footprint overlaps box."""
    out = []
    for name, (pkg, (cx, cy), yaw, zb) in SC.FURNITURE.items():
        if SA.FURNITURE[pkg]['role'] != 'basket':
            continue
        sx, sy, _ = (v / 1000 for v in SA.FURNITURE[pkg]['size'])
        if abs(abs(yaw) - 90.0) < 1:
            sx, sy = sy, sx
        if box[0] < cx + sx / 2 and cx - sx / 2 < box[1] and box[2] < cy + sy / 2 and cy - sy / 2 < box[3]:
            out.append(name)
    return out


def side_min_z(bx, by, hd):
    """Lowest TCP height at which the hands may swing out beside the robot (the elbow-open pose) at this base pose:
    0 when nothing is there, the obstacle's top + SIDE_CLEAR for a low one (the table), None for a tall one (a rack, a
    shelf unit, a wall: the station is not usable)."""
    c, s = math.cos(math.radians(hd)), math.sin(math.radians(hd))
    x0, x1, y0, y1 = SIDE_ENV
    z = 0.0
    for sg in (1, -1):
        pts = [(bx + c * u - s * v, by + s * u + c * v) for u in (x0, x1) for v in (sg * y0, sg * y1)]
        box = (min(p[0] for p in pts), max(p[0] for p in pts), min(p[1] for p in pts), max(p[1] for p in pts))
        for name in SM.hits(box) + _basket_hits(box):
            if name not in SC.FURNITURE:
                return None                                       # a wall
            pkg, _, _, zb = SC.FURNITURE[name]
            top = zb + SA.FURNITURE[pkg]['size'][2] / 1000
            if top > 1.1:
                return None                                       # a tall unit beside the robot
            z = max(z, round(top + SIDE_CLEAR, 3))
    return z


def pair_station(A, B):
    """A, B: mono station dicts of two task products on one board. Returns the pair station or None."""
    hd = A['yaw']; yaw = math.radians(hd)
    c, s = math.cos(yaw), math.sin(yaw)
    mid = ((A['target'][0] + B['target'][0]) / 2, (A['target'][1] + B['target'][1]) / 2)
    dA, dB = G.dims(A['product']), G.dims(B['product'])
    gz = min(G.grip_heights(A['surface'], dA[2])[0], G.grip_heights(B['surface'], dB[2])[0])
    for d in np.arange(0.45, 0.80, 0.01):
        for b in (0.0, 0.02, -0.02, 0.05, -0.05):
            bx, by = mid[0] - c * d + s * b, mid[1] - s * d - c * b
            pa, pb = _to_base(A['target'], bx, by, yaw), _to_base(B['target'], bx, by, yaw)
            (L, pl, dl), (R, pr, dr) = sorted([(A, pa, dA), (B, pb, dB)], key=lambda t: -t[1][1])   # left = +y in the base frame
            if pl[1] <= 0 or pr[1] >= 0:
                continue
            eL = _ok_margin('left', pl, dl[1], gz)
            eR = eL and _ok_margin('right', pr, dr[1], gz)
            if not (eL and eR):
                continue
            if SM.hits(SM.footprint(bx, by, hd)):
                continue
            side_z = side_min_z(bx, by, hd)
            if side_z is None:
                continue
            return dict(x=round(bx, 3), y=round(by, 3), yaw=hd, grasp='side', grip_z=round(gz, 3), reach=round(float(d), 3),
                        side_min_z=side_z,
                        board=A['board'], surface=A['surface'], section=A.get('section'),
                        left=dict(station=L['_name'], product=L['product'], target=L['target'], elbow=eL),
                        right=dict(station=R['_name'], product=R['product'], target=R['target'], elbow=eR))
    return None


def compute():
    K.use(LAYOUT)
    mono = SM.load()
    st = {'home': dict(mono['home'])}
    task = [dict(s_, _name=n) for n, s_ in mono.items() if s_.get('task') and 'x' in s_]
    boards = {}
    for s_ in task:
        boards.setdefault(s_['board'], []).append(s_)
    tried = []
    for board, ss in sorted(boards.items()):
        for A, B in itertools.combinations(ss, 2):
            gap = math.dist(A['target'], B['target'])
            if not GAP[0] <= gap <= GAP[1] or 'side' not in (A.get('grasp'), B.get('grasp')) or A.get('grasp') != B.get('grasp'):
                tried.append((A['_name'], B['_name'], f'gap {gap:.2f} m / grasps {A.get("grasp")}, {B.get("grasp")}'))
                continue
            r = pair_station(A, B)
            if r is None:
                tried.append((A['_name'], B['_name'], 'no base pose: both arms + footprint'))
                continue
            name = f"pair_{A['_name']}__{B['_name'].split('_', 2)[-1]}"
            r['approach'] = SM.approach_for(r)
            st[name] = r
    return st, tried


def load():
    return json.load(open(OUT))['stations']


def route(st, a, b):
    return SM.route(st, a, b)


if __name__ == '__main__':
    st, tried = compute()
    json.dump(dict(note='scene_v0 DUAL-arm pair stations (stations_dual.py); world frame; yaw deg; approach = turning point',
                   stations=st), open(OUT, 'w'), indent=1)
    for n, s_ in st.items():
        if n == 'home':
            continue
        print(f"{n:62s} ({s_['x']:6.3f}, {s_['y']:6.3f}, {s_['yaw']:4.0f})  grip z {s_['grip_z']:.3f}  side z {s_['side_min_z']:.2f}  "
              f"L {s_['left']['product']}  R {s_['right']['product']}  approach {s_['approach']}")
    print(f'\n{len(st) - 1} pair station(s) -> {OUT}')
    for a, b, why in tried:
        print(f'  not paired: {a} + {b}: {why}')
