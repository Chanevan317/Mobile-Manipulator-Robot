"""Reach test: which spots on a table or a shelf board can each arm reach? (no Isaac Sim needed)

For each table height the hand points straight down (top-down pick, TCP 20 mm above the table, any of 8 headings,
either elbow). For each shelf board the hand is level and points into the shelf (+x), TCP 32 mm above the board,
as for a 70 mm product. A spot counts as reachable when the closed-form IK has a solution inside the joint limits
and the arm, with its jaws open 60 mm, touches nothing: not the robot (base, column, carriage, tote, its own base),
not the other arm (parked straight out to its side, hand level), and not the table / shelf boards.

The table front edge is --edge m in front of the base centre (the bumper is at 0.259 m). Shelves use --shelf_edge
(default 0.55 m): with the hand level the wrist sits 0.255 m behind the TCP and must stay >= 0.155 m from the shoulder,
so a straight-in grasp cannot reach close to the robot; the shelf task stops with the shelf edge about 0.64 m out.
Collision shapes are the URDF collision meshes, so the result matches Isaac Sim's geometry.

Run from the package folder with a Python that has numpy + manifold3d (env_isaaclab):
    python scripts/reach_test.py                                   # prototype, tables 0.45/0.60/0.75/0.90, shelves 0.30/0.70/1.10
    python scripts/reach_test.py --tables 0.75 --shelves "" --step 0.02
    python scripts/reach_test.py --layout prototype_mono_arm         # the one-arm experiment (shoulder ring around the column)
The tote behind the column is tested too (hand down, TCP 20 mm over its floor); --no_tote skips it.
Writes reach_test_<layout>.csv and reach_test_<layout>.png next to the package (or --out PREFIX).
"""
import argparse, math, os, sys, csv, time, collections
import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
pkg = os.path.normpath(os.path.join(here, '..'))
ap = argparse.ArgumentParser()
ap.add_argument('--layout', default='prototype', choices=('prototype', 'prototype_mono_arm'))
ap.add_argument('--tables', default='0.45,0.60,0.75,0.90', help='table heights (m), comma separated; "" for none')
ap.add_argument('--shelves', default='0.30,0.70,1.10', help='shelf board heights (m); "" for none')
ap.add_argument('--edge', type=float, default=0.30, help='table front edge, m in front of the base centre')
ap.add_argument('--shelf_edge', type=float, default=0.55, help='shelf front edge, m in front of the base centre')
ap.add_argument('--depth', type=float, default=0.55, help='how far past the edge to test (m)')
ap.add_argument('--width', type=float, default=0.45, help='test y from -width to +width (m)')
ap.add_argument('--step', type=float, default=0.03, help='grid step (m)')
ap.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) - 2))
ap.add_argument('--no_tote', action='store_true', help='skip the tote')
ap.add_argument('--out', default=None, help='output prefix (default: <package>/reach_test_<layout>)')
args = ap.parse_args()

sys.path.insert(0, os.path.join(pkg, 'generator')); sys.path.insert(0, here)
from urdf_fk import Robot, org                    # noqa: E402
from lib import xf, union, box                    # noqa: E402
import scara_ik as K                              # noqa: E402

K.use(args.layout)
SIDES = list(K.SHOULDER)                          # ['left', 'right'], or ['left'] for the mono-arm experiment
TOTE = dict(x=(-0.238, 0.098), y=0.233, floor=0.263)   # tote interior / floor top in base_link (urdf_build lift_prototype)
D = math.radians
ARM = ['upper_arm', 'forearm', 'wrist', 'tilt', 'hand', 'finger_right', 'finger_left']
PARK = dict(shoulder_pan=D(45), elbow_pan=0.0, wrist_pan=0.0, wrist_pitch=0.0, wrist_roll=0.0)
TABLE_DZ, SHELF_DZ, OPEN = 0.020, 0.032, 0.030
BOARD_T, SHELF_GAP, SHELF_DEPTH = 0.02, 0.40, 0.30


def init():
    global R, LS, STATIC_NAMES
    R = Robot(pkg, args.layout)
    LS = {}
    for nm, l in R.links.items():
        parts = [xf(R.geom(v.find('geometry')), org(v)) for v in l.findall('collision')]
        if parts: LS[nm] = union(parts)
    par = {j.find('child').get('link'): j.find('parent').get('link') for j in R.J}
    def on_arm(n):                                # links carried by an arm (the mono-arm camera mast rides on the ring)
        while n in par:
            n = par[n]
            if n.startswith(('left_upper_arm', 'right_upper_arm')): return True
        return False
    global EXTRA
    EXTRA = [n for n in LS if not n.startswith(('left_', 'right_')) and on_arm(n)]
    STATIC_NAMES = [n for n in LS if not n.startswith(('left_', 'right_')) and 'roller' not in n and not n.startswith('wheel') and n not in EXTRA]


def solids(q, names):
    W = R.fk(q)
    return union([xf(LS[n], W[n]) for n in names if n in LS])


def edge_of(kind):
    return {'table': args.edge, 'shelf': args.shelf_edge, 'tote': TOTE['x'][0]}[kind]


def scene(kind, h):
    e = edge_of(kind)
    if kind == 'tote':
        return box(5.0, 5.1, 0, 0.1, 0, 0.1)           # the tote is part of the robot (static) already
    if kind == 'table':
        return box(e, e + 1.0, -0.8, 0.8, 0.0, h)
    return union([box(e, e + SHELF_DEPTH, -0.8, 0.8, h - BOARD_T, h), box(e, e + SHELF_DEPTH, -0.8, 0.8, h + SHELF_GAP - BOARD_T, h + SHELF_GAP),
                  box(e + SHELF_DEPTH, e + SHELF_DEPTH + 0.01, -0.8, 0.8, 0.0, h + SHELF_GAP + 0.1)])


def hit(a, b):
    return (a ^ b).volume() * 1e9 > 1.0


CACHE = {}


def obstacles(side, lift, kind, h):
    key = (side, round(lift, 4), kind, h)
    if key not in CACHE:
        other = 'right' if side == 'left' else 'left'
        q = {'lift': lift, **{f'{other}_{k}': (v if other == 'right' else (-v if k in ('shoulder_pan', 'elbow_pan', 'wrist_pan') else v))
                               for k, v in PARK.items()}}
        others = [f'{other}_base'] + [f'{other}_{k}' for k in ARM] if other in SIDES else []
        env = union([solids(q, STATIC_NAMES), solids(q, others), scene(kind, h)])
        CACHE[key] = env
    return CACHE[key]


def reach_one(job):
    side, kind, h, x, y = job
    if kind in ('table', 'tote'):
        z, pitch, yaws = h + TABLE_DZ, D(90), [k * math.pi / 4 for k in range(8)]
    else:
        z, pitch, yaws = h + SHELF_DZ, 0.0, [0.0]
    for yaw in yaws:
        for e in (1, -1):
            q = K.ik(side, x, y, z, yaw, pitch, elbow=e)
            if q is None: continue
            qq = {'lift': q['lift'], f'{side}_finger_right': OPEN, f'{side}_finger_left': OPEN,
                  **{f'{side}_{k}': v for k, v in q.items() if k != 'lift'}}
            a = solids(qq, [f'{side}_{k}' for k in ARM] + EXTRA)
            if hit(a, obstacles(side, q['lift'], kind, h)): continue
            if hit(solids(qq, [f'{side}_{k}' for k in ARM[1:]]), solids(qq, [f'{side}_base'])): continue
            return True
    return False


if __name__ == '__main__':
    from multiprocessing import Pool
    tables = [float(v) for v in args.tables.split(',') if v.strip()]
    shelves = [float(v) for v in args.shelves.split(',') if v.strip()]
    XS = {k: np.arange(edge_of(k) + args.step / 2, edge_of(k) + (args.depth if k == 'table' else SHELF_DEPTH), args.step) for k in ('table', 'shelf')}
    ys = np.arange(-args.width, args.width + 1e-9, args.step)
    XS['tote'] = np.arange(TOTE['x'][0] + args.step / 2, TOTE['x'][1], args.step)
    YS = {'table': ys, 'shelf': ys, 'tote': ys[np.abs(ys) <= TOTE['y']]}
    cases = [('table', h) for h in tables] + [('shelf', h) for h in shelves] + ([] if args.no_tote else [('tote', TOTE['floor'])])
    jobs = [(s, k, h, float(x), float(y)) for k, h in cases for s in SIDES for x in XS[k] for y in YS[k]]
    t0 = time.time()
    print(f'reach test, layout {args.layout}: {len(cases)} cases, {len(jobs)} checks on {args.jobs} processes', flush=True)
    with Pool(args.jobs, initializer=init) as p:
        res = p.map(reach_one, jobs, chunksize=4)
    ok = collections.defaultdict(bool, {j[:5]: r for j, r in zip(jobs, res)})
    prefix = args.out or os.path.join(pkg, f'reach_test_{args.layout}')
    with open(prefix + '.csv', 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['surface', 'height_m', 'x_m', 'y_m', 'left', 'right'])
        for k, h in cases:
            for x in XS[k]:
                for y in YS[k]:
                    w.writerow([k, h, round(x, 3), round(y, 3), int(ok[('left', k, h, float(x), float(y))]), int(ok[('right', k, h, float(x), float(y))])])
    cell = args.step ** 2
    print(f'\nedges: table {args.edge:.2f} m, shelf {args.shelf_edge:.2f} m in front of the base centre')
    print(f'{"surface":8s} {"height":>6s} | {"area either":>11s} {"both":>7s} | {"deepest past edge":>17s} {"width at 0.10 m":>16s} {"width at 0.20 m":>16s}')
    for k, h in cases:
        xs = XS[k]; yk = YS[k]
        e = [[ok[('left', k, h, float(x), float(y))] or ok[('right', k, h, float(x), float(y))] for y in yk] for x in xs]
        b = [[ok[('left', k, h, float(x), float(y))] and ok[('right', k, h, float(x), float(y))] for y in yk] for x in xs]
        E, B = np.array(e), np.array(b)
        deep = max((x - edge_of(k) for x, row in zip(xs, E) if row.any()), default=0.0) + args.step / 2
        def width_at(d):
            i = int(np.argmin(np.abs(xs - (edge_of(k) + d)))); return E[i].sum() * args.step if abs(xs[i] - edge_of(k) - d) < args.step else 0.0
        if k == 'tote':
            print(f'{k:8s} {h:6.2f} | {E.sum()*cell:9.3f} m2 {B.sum()*cell:5.3f} m2 | {100*E.mean():.0f} % of the tote floor reachable')
            continue
        print(f'{k:8s} {h:6.2f} | {E.sum()*cell:9.3f} m2 {B.sum()*cell:5.3f} m2 | {deep:15.2f} m {width_at(0.10):14.2f} m {width_at(0.20):14.2f} m')
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        n = len(cases); fig, axs = plt.subplots(1, n, figsize=(3.6 * n, 4.2), squeeze=False)
        for ax, (k, h) in zip(axs[0], cases):
            for x in XS[k]:
                for y in YS[k]:
                    a, bb = ok[('left', k, h, float(x), float(y))], ok[('right', k, h, float(x), float(y))]
                    ax.plot(y, x, 's', ms=3.5, color='green' if a and bb else ('#4c9be8' if a else ('#e8a14c' if bb else '#dddddd')))
            ax.axhline(edge_of(k), color='k', lw=0.8); ax.set_title(f'{k} {h:.2f} m', fontsize=9)
            ax.set_xlim(args.width + 0.02, -args.width - 0.02); ax.set_aspect('equal'); ax.tick_params(labelsize=7)
            ax.set_xlabel('y (m), left arm side on the left'); ax.set_ylabel('x forward (m)')
        fig.suptitle(f'reach test ({args.layout}): green both arms, blue left only, orange right only, grey none', fontsize=9)
        plt.tight_layout(); plt.savefig(prefix + '.png', dpi=80)
    except ImportError:
        pass
    print(f'\nwrote {prefix}.csv' + (f' and {prefix}.png' if os.path.exists(prefix + '.png') else '') + f'  ({time.time()-t0:.0f} s)')
