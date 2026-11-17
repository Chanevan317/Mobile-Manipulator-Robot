"""Fixed robot stations for scene_v0: named base poses (x, y, heading) the robot is DRIVEN to for arm training.

Not real navigation: drive_stations.py moves the base between these marked coordinates on a fixed route. Run this file
to (re)compute them after a layout change; it writes stations.json (kept in git) and prints the table.

    python scenes/scene_v0/stations.py           # no simulator needed

Stations (world frame = the robot's start pose, metres / degrees):
  home                      the start pose (0, 0, 0)
  <board>_<product>         one per product run on a board the mono arm reaches (rack_b0, rack2_b0, shelf_b1 / b2,
                            shelf2_b1 / b2): the base stops facing the board, with the run's centre product reachable
                            (grasp_plan: side grasp, or from above for short ones)
  table_<product>           one per table task product (the 4 short ones at the table edge)
Every station is checked against the furniture and walls with the robot's footprint (base 0.50 x 0.50 + the scene
camera reaching 0.34 m forward, + MARGIN), and gets an APPROACH point nearby with room to turn on the spot (TURN_R
clear all round): legs go station -> its approach -> (straight across, turning on the way) -> the next approach ->
straight in. When the straight line between two approach points is not clear for a turning robot, the leg goes via
HUB (the middle of the aisle).
Checked: kinematics (scara_ik, joint limits) and footprints (2-D boxes). NOT checked: the arm against the furniture.
"""
import json, math, os, sys
import numpy as np
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
import assets as SA
import scene as SC
import grasp_plan as G

FOOT = dict(back=0.25, front=0.34, side=0.25)   # robot frame: base 0.50 x 0.50, scene camera to x 0.341
MARGIN = 0.02
TURN_R = math.hypot(FOOT['front'], FOOT['side']) + 0.03     # clearance needed to turn on the spot
HUB = (-1.20, 0.0)
OUT = os.path.join(here, 'stations.json')


def obstacles():
    """Furniture footprints + walls as axis-aligned boxes (x0, x1, y0, y1)."""
    boxes = {}
    for name, (pkg, (cx, cy), yaw, zb) in SC.FURNITURE.items():
        if SA.FURNITURE[pkg]['role'] == 'basket':
            continue                                            # on the table
        sx, sy, _ = (v / 1000 for v in SA.FURNITURE[pkg]['size'])
        if abs(abs(yaw) - 90.0) < 1:
            sx, sy = sy, sx
        boxes[name] = (cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2)
    R, big = SC.ROOM, 10.0
    boxes['wall_table'] = (R['x1'], R['x1'] + big, -big, big)
    boxes['wall_racks'] = (-big, big, R['y1'], R['y1'] + big)
    boxes['wall_shelves'] = (-big, big, R['y0'] - big, R['y0'])
    boxes['open_side'] = (R['x0'] - big, R['x0'], -big, big)    # stay inside the room
    return boxes


OBST = obstacles()


def footprint(x, y, yaw_deg, margin=MARGIN):
    """World box of the robot outline at a heading that is a multiple of 90 deg."""
    c, s = round(math.cos(math.radians(yaw_deg))), round(math.sin(math.radians(yaw_deg)))
    pts = [(x + c * u - s * v, y + s * u + c * v) for u in (-FOOT['back'], FOOT['front']) for v in (-FOOT['side'], FOOT['side'])]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def hits(box):
    return [n for n, (a0, a1, b0, b1) in OBST.items() if box[0] < a1 and a0 < box[1] and box[2] < b1 and b0 < box[3]]


def clearance(x, y):
    """Distance from a point to the nearest obstacle."""
    return min(math.hypot(max(a0 - x, 0, x - a1), max(b0 - y, 0, y - b1)) for a0, a1, b0, b1 in OBST.values())


def line_clear(p, q, yaw_deg=None, step=0.02, skip_start=False):
    """Straight move p -> q: with a fixed heading (yaw_deg) the footprint must stay clear; with yaw_deg None the robot may
    turn on the way, so every point needs TURN_R all round."""
    n = max(1, int(math.dist(p, q) / step))
    for k in range(1 if skip_start else 0, n + 1):         # skip_start: p is a station, already accepted
        x, y = p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n
        if (yaw_deg is None and clearance(x, y) < TURN_R) or (yaw_deg is not None and hits(footprint(x, y, yaw_deg))):
            return False
    return True


def base_for(pkg, centre, surface, heading_deg, front_dist, dz):
    """First base pose (x, y, yaw) from which the arm picks the product at `centre` (world) and the robot's outline is
    clear. front_dist: furniture front edge -> product centre along the heading."""
    narrow, deep, h = G.dims(pkg); gz, fz = G.grip_heights(surface, h)
    c, s = math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg))
    for d in np.arange(0.40, 0.95, 0.01):
        if d - front_dist < G.FRONT_MIN:
            continue
        for b in (0.0, 0.02, -0.02, 0.05, -0.05, 0.10, -0.10, 0.15, -0.15, 0.20, -0.20):
            if h <= G.TOP_H:
                ok = any(G._ok(G.top_poses((d, b), yaw, fz, dz)) for yaw in (0.0, math.pi))
                kind = 'top'
            else:
                ok = G._ok(G.side_poses((d, b), 0.0, deep, gz, dz)); kind = 'side'
            if not ok:
                continue
            bx, by = centre[0] - c * d + s * b, centre[1] - s * d - c * b
            if not hits(footprint(bx, by, heading_deg)):
                return dict(x=round(bx, 3), y=round(by, 3), yaw=heading_deg, grasp=kind, reach=round(float(d), 3), offset=b)
    return None


def approach_for(st):
    """Nearest point behind / beside the station with TURN_R clear all round, reachable straight from it."""
    c, s = math.cos(math.radians(st['yaw'])), math.sin(math.radians(st['yaw']))
    best = None
    for t in np.arange(0.0, 0.85, 0.05):
        for l in (0.0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4, -0.4, 0.5, -0.5):
            p = (st['x'] - c * t - s * l, st['y'] - s * t + c * l)
            if clearance(*p) >= TURN_R and line_clear((st['x'], st['y']), p, st['yaw'], skip_start=True):
                cost = t + abs(l)
                if best is None or cost < best[0]:
                    best = (cost, p)
    return None if best is None else [round(best[1][0], 3), round(best[1][1], 3)]


def compute():
    st = {'home': dict(x=0.0, y=0.0, yaw=0.0, product=None, board='start', note='start pose')}
    surf = {sf[0]: sf for sf in SC._surfaces()}
    for key, zt, (u0, u1), front, hd, clr, dyn in SC._surfaces():
        if not dyn or key == 'table_back':
            continue
        for a, b, pkg in SC._segments(key, u0, u1):
            u = (a + b) / 2
            if SC.TASK_BOARD.get(pkg) == key:
                u = SC.REST[pkg][1]                               # the task product itself
            deep = G.dims(pkg)[1]
            inset = (0.04 if SC.TASK_BOARD.get(pkg) == key else 0.03) + deep / 2
            centre = (u, front + inset) if hd == 90.0 else (u, front - inset)
            r = base_for(pkg, centre, zt, hd, inset, 0.02)
            name = f'{key}_{pkg.replace("food_", "").replace("processed_", "").replace("beverage_", "").replace("condiment_", "")}'
            st[name] = dict(r or {}, product=pkg, board=key, surface=zt, section=SC.SECTION_NAMES[key.split('_')[0]],
                            target=[round(centre[0], 3), round(centre[1], 3)], task=SC.TASK_BOARD.get(pkg) == key)
            if r is None:
                st[name]['note'] = 'NO STATION: not reachable with a clear footprint'
    for pkg, (z, x, y, hd) in SC.REST.items():
        if hd != 0.0:
            continue
        r = base_for(pkg, (x, y), z, 0.0, x - SC.TABLE_EDGE, 0.10)
        st[f'table_{pkg.replace("food_", "").replace("processed_", "").replace("beverage_", "")}'] = dict(
            r or {}, product=pkg, board='table', surface=z, section='table', target=[x, y], task=True)
    for name, s_ in st.items():
        if 'x' in s_:
            s_['approach'] = approach_for(s_)
    return st


def route(st, a, b):
    """Waypoints (x, y, yaw) from station a to station b."""
    A, B = st[a], st[b]
    pa, pb = tuple(A['approach']), tuple(B['approach'])
    pts = [(pa[0], pa[1], A['yaw'])]
    if not line_clear(pa, pb):
        pts.append((HUB[0], HUB[1], B['yaw']))
    pts += [(pb[0], pb[1], B['yaw']), (B['x'], B['y'], B['yaw'])]
    return pts


def load():
    return json.load(open(OUT))['stations']


if __name__ == '__main__':
    st = compute()
    bad = [n for n, s_ in st.items() if 'x' not in s_ or s_.get('approach') is None]
    json.dump(dict(note='scene_v0 robot stations (stations.py); world frame = robot start; yaw deg; approach = turning point',
                   footprint=FOOT, margin=MARGIN, turn_r=round(TURN_R, 3), hub=HUB, stations=st), open(OUT, 'w'), indent=1)
    print(f'{"station":28s} {"x":>7s} {"y":>7s} {"yaw":>5s}  grasp  reach  approach           section / product')
    for n, s_ in st.items():
        if 'x' not in s_:
            print(f'{n:28s} {s_.get("note")}'); continue
        print(f'{n:28s} {s_["x"]:7.3f} {s_["y"]:7.3f} {s_["yaw"]:5.0f}  {s_.get("grasp", "-"):5s}  {s_.get("reach", 0):5.2f}  '
              f'{str(s_.get("approach")):18s} {s_.get("section", "")} / {(s_.get("product") or "").replace("food_", "")}{"  (task product)" if s_.get("task") else ""}')
    print(f'hub {HUB} clearance {clearance(*HUB):.2f} (need {TURN_R:.2f}); {len(st)} stations, problems: {bad or "none"} -> {OUT}')
