"""Mono holder (user 2026-11-23: replaces the mono ring for the one-arm prototype): ONE printed PETG part on the real
C-Beam XLarge gantry plate that carries the arm's own base (01 base_top + 02 base_bottom, as the dual holder does) and
the scene camera (scene_cam.py: a short mast + the pitch head) on a bar ABOVE the arm. The tote stays on the robot's BASE, behind the arm, as in the
original design (BASE_TOTE; urdf_build puts it there): the lift lowers the hand into it.

The lift column stands slightly to the RIGHT of the robot's centre (base y -ARM_DX / 1000) and is turned so its
gantry plate faces the centre; the arm's shoulder axis is then on the robot's centreline (y 0), ROBOT_X forward of the
base centre. Its zero pose
points LEFT (away from the column), so the shoulder's +-115 deg covers the front, the left and the back, and its blind
sector faces the column. The camera sits right above the shoulder, looking forward (user 2026-11-24: "camera above the
arm, a little behind if needed"), so nothing sticks out in front of the carriage and the shoulder can come forward.

Units mm, CARRIAGE frame (as dual_holder.py / mono_ring.py): origin on the column axis at the carriage origin, z up,
gantry plate behind the beam at x -27.5..-21.5, beam x -20..20, |y| <= 40. In the robot (base_link) this frame is
turned -90 deg: carriage +y = robot FRONT, carriage -x = robot LEFT (toward the centre).

    pad        12 mm under the arm base (its footprint), z 13.5..25.5; 4 x M5 x 20 from below into the base's M5 x 8 inserts
    legs, lip, rear bridge   as the dual holder: bolted through the gantry plate's own 5.2 mm holes (y +-50,
               z -12.5 / -72.5), nylocs on its front face; the lip rests on the plate's top edge
    post       on the rear (-y) leg, behind the beam and its wheels (x -44..-10, y -75..-46), up to the bar; it stands where
               the shoulder's +-115 deg never takes the arm (between the column and the back)
    bar        over the arm (z BAR, the arm's highest point is z ~135): from the post along -x, then forward over the
               shoulder; a 2 mm pocket + 4 x M3 x 8 inserts for the mast's flange, the mast turned 90 deg (camera
               looking along carriage +y = forward), its tilt axis straight above the shoulder

    python3 mono_holder.py [out_dir]     checks + export (default ../../CAD/mono_holder)
"""
import math, os, sys, csv, collections
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import M, CS, box, cyl, union, xf, T, Rx, Ry, Rz, print_mesh
from hw import screw_cut, insert_cut, Reg
import dual_holder as DH

ARM_DX = 122.0                    # column axis -> shoulder axis (carriage -x); the arm base pillar ends 3 mm before the
                                  # top end mount's footprint (x >= -30), so the lift keeps its full 0.871 m
BASE = 0.45                       # m: square mecanum base (user 2026-11-24: "resize 450 x 450 ... if needed"; was 0.50):
                                  # the hand reaches ~as much of the deck (0.098 vs 0.103 m2) and wastes less (52 vs 45 %)
ROBOT_X = round(BASE / 2 - 0.07, 3)   # m: the column and the shoulder sit this far FORWARD of the base centre (urdf_build
                                  # SINGLE_CX): 0.07 behind the deck's front edge, as 0.18 was on the 0.50 base (0.12 until
                                  # 2026-11-24); the carriage's front (plate, legs: 75 mm ahead) ends at the bumper
AZ = DH.AZ                        # 65.5: arm frame origin (servo mid-plane), as the dual holder
BASE_Z = DH.BASE_Z                # 25.5: base underside
PAD = DH.PAD                      # (13.5, 25.5)
ARM_YAW = 180.0                   # arm frame: zero pose along carriage -x (robot left), pillar toward the column
                                  # (robot heading of the zero pose = ARM_YAW - 90)
LEG, LIP, REAR = DH.LEG, DH.LIP, DH.REAR
BAR = (145.0, 167.0)              # bar z: 10 mm over the arm's highest point (z 134.7, any pose)
POST = dict(x=(-44.0, -10.0), y=(-75.0, -46.0), z=(21.0, BAR[1]))   # from 1 mm over the gantry plate / wheel tops (z 20)
BAR1 = dict(x=(-148.0, -10.0), y=(-75.0, -46.0))           # from the post toward the shoulder line, behind it
ARM_DY = 0.0                      # shoulder axis this far in FRONT of the column axis (carriage +y), mm
import scene_cam as SC
CAM_Z = BAR[1] - 2.0              # mast mount face (the pocket floor) = scene_cam.MOUNT['prototype_mono_arm']['z']


def _derive():
    """Everything placed relative to the shoulder (call after changing ARM_DX / ARM_DY). scene_cam builds the mast in its
    own frame (flange centre at x MX, camera along +x, tilt axis 23 mm in front): turned +90 deg, flange centre at
    (-ARM_DX, MAST_Y), so the tilt axis is right over the shoulder."""
    global BAR2, MAST_Y, MAST_T, POCKET, CAM_INSERTS
    MAST_Y = ARM_DY - (SC.AX - SC.MX)
    MAST_T = T(-ARM_DX, MAST_Y, 0.0) @ Rz(90.0) @ T(-SC.MX, 0.0, 0.0)
    POCKET = dict(x=(-ARM_DX - 22.2, -ARM_DX + 22.2), y=(MAST_Y - 20.2, MAST_Y + 20.2))   # the flange is 44 (carriage x) x 40 (y)
    CAM_INSERTS = [tuple((MAST_T @ np.array([SC.MX + sx * SC.FL['bx'], sy * SC.FL['by'], 0.0, 1.0]))[:2]) for sx in (-1, 1) for sy in (-1, 1)]
    BAR2 = dict(x=(-148.0, -96.0), y=(-75.0, MAST_Y + 25.0))   # forward over the shoulder, under the mast flange


_derive()
# the tote on the robot's BASE (robot frame, m), L-SHAPED over the WHOLE deck (user 2026-11-24: "let us have L shaped
# and we use the whole base, ... slightly slanted area so that near edges slightly falls to accessible edge"): part A
# across the deck behind the arm (back edge .. 15 mm before the holder's rear legs), part B along the free left side to
# the front edge; the column / holder corner (front right) is the column's. The floor is FLAT where the hand pointing
# down reaches inside the tote (TOTE_FLAT: traced from reach_test's tote map, smoothed) and rises at TOTE_SLOPE from
# there to the walls (the back strip, the corners, the right), so a product that slides there (braking, a drop off
# target) comes back down to where the arm gets it. A round flat floor (0.27..0.33 m from the shoulder) reached less.
TOTE_FLAT = [  # robot frame, m: two convex parts (each its own funnel; together the reachable patch)
    [(-0.048, -0.177), (0.005, -0.152), (0.017, -0.100), (0.004, -0.043), (0.008, 0.014), (0.027, 0.071), (-0.095, 0.137),
     (-0.126, 0.084), (-0.138, 0.031), (-0.140, -0.032), (-0.125, -0.089), (-0.092, -0.136)],          # behind the shoulder
    [(0.027, 0.071), (0.073, 0.099), (0.127, 0.116), (0.179, 0.148), (0.185, 0.187), (0.127, 0.202), (0.063, 0.201),
     (0.004, 0.198), (-0.050, 0.180), (-0.095, 0.137)],                                                 # left of the arm
]
TOTE_FLAT_GROW = 0.05             # m: the flat floor reaches this far past TOTE_FLAT (room for the hand + open jaws
                                  # beside the slope; slopes right at the reach edge made the edge cells unreachable)
TOTE_DEPTH = 0.12                 # m: deck top .. rim
TOTE_SLOPE = 30.0                 # deg: cardboard / plastic on PP or PETG slide from ~20-25 deg


def base_tote():
    e = round(BASE / 2 - 0.007, 3)                    # the walls' inner faces 7 mm inside the deck edge
    A = (-e, round(ROBOT_X - 0.09, 3), -e, e)
    B = (A[1], e, 0.06, e)
    return dict(rects=[A, B], x=(A[0], B[1]), y=(A[2], A[3]), z=(0.256, round(0.256 + TOTE_DEPTH, 3)), wall=0.007)


def _flat_cs():
    return [CS([[(x * 1000, y * 1000) for x, y in P]]).hull().offset(TOTE_FLAT_GROW * 1000, circular_segments=64) for P in TOTE_FLAT]


def tote_floor_z(t, x, y):
    """Floor top (m) at (x, y): flat over TOTE_FLAT, rising at TOTE_SLOPE with the distance from it."""
    import numpy as np
    d = 1e9
    for P in TOTE_FLAT:
        P = np.array(P); n = len(P); inside = True; dd = 1e9
        for i in range(n):
            a, b = P[i], P[(i + 1) % n]; e_ = b - a; v = np.array((x, y)) - a
            cr = e_[0] * v[1] - e_[1] * v[0]
            if cr < 0: inside = False
            u = min(1.0, max(0.0, float(v @ e_ / (e_ @ e_)))); dd = min(dd, float(np.linalg.norm(v - u * e_)))
        d = min(d, 0.0 if inside else max(0.0, dd - TOTE_FLAT_GROW))
    return t['z'][0] + t['wall'] + d * math.tan(math.radians(TOTE_SLOPE))


def tote_solid(t):
    """The tote (mm, robot frame): walls round the L (rects = INNER faces), floor + the slopes: the cavity is a funnel
    over each convex part of TOTE_FLAT (its outline at the floor, grown by H / tan(slope) at the rim), clipped to the L."""
    w, (z0, z1) = t['wall'] * 1000, (v * 1000 for v in t['z'])
    inner = union([box(r[0] * 1000, r[1] * 1000, r[2] * 1000, r[3] * 1000, z0 + w, z1 + 1) for r in t['rects']])
    outer = union([box(r[0] * 1000 - w, r[1] * 1000 + w, r[2] * 1000 - w, r[3] * 1000 + w, z0, z1) for r in t['rects']])
    H = z1 + 1 - (z0 + w); grow = H / math.tan(math.radians(TOTE_SLOPE))
    funnels = [M.hull(c.extrude(0.01).translate((0, 0, z0 + w)) + c.offset(grow, circular_segments=64).extrude(0.01).translate((0, 0, z0 + w + H - 0.01)))
               for c in _flat_cs()]
    return outer - (inner ^ union(funnels))


def in_tote(t, x, y, margin=0.0):
    """(x, y) inside the L tote's walls (robot frame), margin further in."""
    w = margin
    return any(r[0] + w < x < r[1] - w and r[2] + w < y < r[3] - w for r in t['rects']) or \
        (t['rects'][0][1] - w <= x <= t['rects'][0][1] + w and t['rects'][1][2] + w < y < t['rects'][1][3] - w)


BASE_TOTE = base_tote()
LIFT_MIN = 0.0
BASE_M5 = DH.BASE_M5
GANTRY_HOLES = DH.GANTRY_HOLES
GANTRY, WHEELS, COLUMN, TOP_MOUNT, FOOT = DH.GANTRY, DH.WHEELS, DH.COLUMN, DH.TOP_MOUNT, DH.FOOT

PARTS, CONTEXT = {}, {}


def arm_frame():
    return T(-ARM_DX, ARM_DY, AZ) @ Rz(ARM_YAW)


def pad(r):
    sh = (CS.circle(40.0, 96) + CS([[(-89.0, -24.5), (-46.0, -24.5), (-46.0, 24.5), (-89.0, 24.5)]])).hull()
    p = xf(sh.extrude(PAD[1] - PAD[0]).translate((0, 0, PAD[0] - AZ)), arm_frame())
    cut = []
    for (x, y) in BASE_M5:
        q = arm_frame() @ np.array([x, y, PAD[0] - AZ, 1.0])
        cut.append(screw_cut(tuple(q[:3]), (0, 0, 1), 'M5', PAD[1] - PAD[0], cb=0.0))
        r.add('SHCS', 'M5', tuple(q[:3]), (0, 0, 1), 20, 'arm base (01 + 02) to the pad, from below into its M5 x 8 inserts')
    return p, cut


def holder():
    r = Reg()
    p, cut = pad(r)
    parts = [p, box(*REAR['x'], -REAR['y'], REAR['y'], *REAR['z']),
             box(*POST['x'], *POST['y'], *POST['z']), box(*BAR1['x'], *BAR1['y'], *BAR), box(*BAR2['x'], *BAR2['y'], *BAR)]
    for sy in (-1, 1):
        y0, y1 = sorted((sy * LEG['y'][0], sy * LEG['y'][1])); parts.append(box(*LEG['x'], y0, y1, *LEG['z']))
        y0, y1 = sorted((sy * LIP['y'][0], sy * LIP['y'][1])); parts.append(box(*LIP['x'], y0, y1, *LIP['z']))
    s = union(parts)
    cut.append(box(*POCKET['x'], *POCKET['y'], CAM_Z, BAR[1] + 1.0))                             # mast flange pocket
    for (x, y) in CAM_INSERTS:
        cut.append(insert_cut((x, y, CAM_Z), (0, 0, -1), 'M3', length=8))
        r.add('insert', 'M3', (x, y, CAM_Z), (0, 0, -1), 8, 'scene camera mast flange (pocket floor)')
    for (y, z) in GANTRY_HOLES:
        p_ = (LEG['x'][0], y, z)
        cut.append(screw_cut(p_, (1, 0, 0), 'M5', (LEG['x'][1] - LEG['x'][0]) + 6.0 + 4.0))
        r.add('SHCS+nyloc', 'M5', p_, (1, 0, 0), 25, 'legs to the gantry plate (its 5.2 mm holes at y +-50)')
    s = s - union(cut)
    PARTS['holder'] = dict(solid=s, color=(0.20, 0.55, 0.62), reg=r, material='PETG', print_rot=Rx(90.0),
                           note='one piece: pad (arm base), legs + lip + rear bridge (gantry plate), post + bar over the arm '
                                '(scene camera mast); prints on its back (the rear leg / post face), supports under the bar')


def base_tote_C(lift=0.0):
    """The base tote in the carriage frame at this lift (robot x = carriage y + ROBOT_X, y = -carriage x - ARM_DX,
    z = carriage z + 420 + lift)."""
    return xf(tote_solid(BASE_TOTE), T(-ARM_DX, -ROBOT_X * 1000, -420.0 - lift * 1000) @ Rz(90.0))


def mast_parts():
    """The scene camera mast + head + servo + camera at pitch 0..80 (carriage frame), for the checks and the preview."""
    SC.build_all('prototype_mono_arm')
    zh = SC.CONTEXT['zh']
    fixed = union([xf(p['solid'], MAST_T) for k, p in SC.PARTS.items() if k != 'camera_cradle'] + [xf(SC.CONTEXT['servo'], MAST_T)])
    turning = union([SC.CONTEXT['cradle_C'], SC.CONTEXT['cam_body_C'], SC.CONTEXT['cam_plug_C']])
    sweep = union([xf(SC.pitched(turning, d, zh), MAST_T) for d in range(int(SC.PITCH[0]), int(SC.PITCH[1]) + 1, 10)])
    return fixed, sweep


def build_all():
    global BASE_TOTE
    _derive()
    PARTS.clear(); CONTEXT.clear()
    BASE_TOTE = base_tote()
    holder()
    CONTEXT['base'] = xf(DH.base_parts(), arm_frame())
    CONTEXT['mast'], CONTEXT['cam_sweep'] = mast_parts()


def arm_moving(q1, q2, q3, pitch, opening=60.0):
    """The arm's moving parts (upper arm .. fingers + their servos / bearings) in the carriage frame."""
    import build, arm
    F = build.frames((q1, q2, q3), pitch, 0.0, opening)
    build.build_parts(F)
    G = arm_frame()
    out = [xf(p['solid'], G @ p['frame']) for nm, p in arm.PARTS.items() if not nm.startswith('base')]
    out += [xf(b['solid'], G) for b in build.bought(F) if not ('shoulder' in b['name'] or '_J1_' in b['name'])]
    return union(out)


def check(sweep=True):
    res, info = [], []
    h, tt = PARTS['holder']['solid'], base_tote_C(0.0)
    kit = (('gantry plate', GANTRY), ('V-wheels', WHEELS), ('beam (1 mm clearance)', box(-21, 21, -41, 41, -300, 1200)),
           ('top end mount footprint (above the plate top)', TOP_MOUNT), ('column foot at lift 0', FOOT))
    for nm, e in kit:
        for what, s in (('holder', h), ('camera mast', CONTEXT['mast']), ('camera (pitch sweep)', CONTEXT['cam_sweep'])):
            v = (s ^ e).volume()
            if v > 0.5: res.append(f'{what} x {nm}: {v:.1f} mm3')
    for a, b, sa, sb in (('holder', 'arm base', h, CONTEXT['base']), ('base tote at lift 0', 'arm base', tt, CONTEXT['base']),
                         ('holder', 'base tote at lift 0', h, tt), ('camera mast', 'arm base', CONTEXT['mast'], CONTEXT['base']),
                         ('camera mast', 'holder', CONTEXT['mast'], h), ('camera mast', 'tote', CONTEXT['mast'], tt)):
        v = (sa ^ sb).volume()
        if v > 1.0 and not (a == 'camera mast' and b == 'holder' and v < 50.0):   # the flange sits in its pocket
            res.append(f'{a} x {b}: {v:.1f} mm3')
    for k in ('holder',):
        n = len([c for c in PARTS[k]['solid'].decompose() if c.volume() > 1.0])
        if n != 1: res.append(f'{k} is {n} pieces')
    # the mast flange seats on the pocket floor
    seat = (h ^ box(-ARM_DX - 22, -ARM_DX + 22, MAST_Y - 20, MAST_Y + 20, CAM_Z - 0.5, CAM_Z)).volume()
    info.append(f'mast flange seat: {seat / 0.5:.0f} mm2 of the 1760 mm2 flange on the pocket floor')
    lo = h.bounding_box()[2] + 420
    info.append(f"holder bottom at lift 0: {lo:.0f} mm over the floor; base L tote {BASE_TOTE['rects']}, "
                f"rim {BASE_TOTE['z'][1] * 1000:.0f} mm (clear of it at every lift: checked above at lift 0)")
    hits = collections.Counter(); n = 0
    if sweep:
        fixed = union([h, CONTEXT['mast'], CONTEXT['cam_sweep']])
        for q1 in range(-115, 116, 23):
            for q2 in range(-125, 126, 50):
                for q3 in (-122, 0, 122):
                    for pitch in (0, 90):
                        n += 1
                        if (arm_moving(q1, q2, q3, pitch) ^ fixed).volume() > 0.5:
                            hits[(q1, q2, q3, pitch)] += 1
        info.append(f'arm sweep: {n} poses; touching the holder / camera: {len(hits)}')
        if hits: info.append(f'  first {sorted(hits)[:12]}')
    vol = sum(p['solid'].volume() for p in PARTS.values()) / 1000
    info.append(f"holder {vol:.0f} cm3 PETG (~{vol * 0.80 * 1.27:.0f} g at the usual fill estimate)")
    return res, info, hits


# ------------------------------------------------------------------ export
def _write(path, solid):
    from stlio import write_stl
    V, F = print_mesh(solid); write_stl(path, V, F)


def export(out):
    for d in ('print_ready_stl', 'assembly_stl', 'assembly_stl/context'):
        os.makedirs(os.path.join(out, d), exist_ok=True)
        for f in os.listdir(os.path.join(out, d)):
            if f.endswith('.stl'): os.remove(os.path.join(out, d, f))
    rows, fast = [], collections.Counter()
    for i, (nm, p) in enumerate(PARTS.items(), 1):
        num = f'H{i:02d}_{nm}'
        _write(os.path.join(out, 'assembly_stl', num + '.stl'), p['solid'])
        s = xf(p['solid'], p['print_rot']); bb = s.bounding_box()
        s = s.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2])); bb = s.bounding_box()
        _write(os.path.join(out, 'print_ready_stl', num + '.stl'), s)
        rows.append(dict(file=num + '.stl', part=nm, material=p['material'],
                         size_mm=f'{bb[3]-bb[0]:.1f} x {bb[4]-bb[1]:.1f} x {bb[5]-bb[2]:.1f}',
                         volume_cm3=round(p['solid'].volume() / 1000, 1), note=p['note']))
        for it in p['reg'].items:
            fast[(it['kind'], it['size'], it['length'], it['note'])] += 1
    ctx = os.path.join(out, 'assembly_stl', 'context')
    _write(os.path.join(ctx, 'arm_base_01_02.stl'), CONTEXT['base'])
    _write(os.path.join(ctx, 'lift_kit_gantry_wheels_beam.stl'), union([GANTRY, WHEELS, COLUMN ^ box(-30, 30, -50, 50, -110, 140)]))
    _write(os.path.join(ctx, 'scene_camera_mast.stl'), CONTEXT['mast'])
    os.makedirs(os.path.join(out, 'tote'), exist_ok=True)              # the base's tote (made by the base team, not printed)
    _write(os.path.join(out, 'tote', 'tote_L_sloped.stl'), tote_solid(BASE_TOTE))
    with open(os.path.join(out, 'tote', 'tote.csv'), 'w', newline='') as f:
        w = csv.writer(f); t = BASE_TOTE
        w.writerow(['what', 'x0_mm', 'x1_mm', 'y0_mm', 'y1_mm', 'note'])
        for nm, r in zip(('A (behind the arm, full width)', 'B (left side, to the front edge)'), t['rects']):
            w.writerow([nm] + [round(v * 1000) for v in r] + ['inner wall faces, robot frame (x forward, y left, base centre)'])
        w.writerow(['floor', '', '', '', '', f"deck top {t['z'][0]*1000:.0f} mm + {t['wall']*1000:.0f} mm floor; flat within "
                    f"the hand's reach (TOTE_FLAT), rising {TOTE_SLOPE:.0f} deg to the walls; rim {t['z'][1]*1000:.0f} mm"])
    with open(os.path.join(out, 'printed_parts.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    with open(os.path.join(out, 'fasteners.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['kind', 'size', 'length_mm', 'qty', 'where'])
        for (k, s, L, n), q in sorted(fast.items(), key=lambda t: (str(t[0][1]), t[0][0], str(t[0][2]))):
            w.writerow([k, s, L, q, n])
    render(os.path.join(out, 'preview_mono_holder.png'))
    print(f'exported {len(rows)} parts to {out}')


def render(path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from render_robot import subdivide
    BG = (0.906, 0.922, 0.941)

    def draw(ax, items, el, az, title, zoom=1.0):
        T_, C_ = [], []
        for s, c in items:
            m = s.to_mesh(); t = np.asarray(m.vert_properties)[:, :3][np.asarray(m.tri_verts)]
            if len(t): t = subdivide(t, 14.0); T_.append(t); C_.append(np.tile(np.r_[c[:3]], (len(t), 1)))
        T_ = np.vstack(T_); C_ = np.vstack(C_)
        N = np.cross(T_[:, 1] - T_[:, 0], T_[:, 2] - T_[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
        Ld = np.array([math.cos(math.radians(el + 25)) * math.cos(math.radians(az + 30)),
                       math.cos(math.radians(el + 25)) * math.sin(math.radians(az + 30)), math.sin(math.radians(el + 25))])
        sh = 0.35 + 0.65 * np.abs(N @ Ld)
        ax.add_collection3d(Poly3DCollection(T_, facecolors=np.c_[C_ * sh[:, None], np.ones(len(sh))], edgecolor='none'))
        lo, hi = T_.reshape(-1, 3).min(0), T_.reshape(-1, 3).max(0)
        c = (lo + hi) / 2; r = (hi - lo).max() / 2 / zoom
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off(); ax.set_title(title, fontsize=13); ax.set_facecolor(BG)

    # shown in the ROBOT's orientation: carriage frame turned -90 deg (carriage +y = robot front = +x in the picture)
    R = Rz(-90.0)
    lift = [(xf(GANTRY, R), (0.80, 0.80, 0.82)), (xf(WHEELS, R), (0.35, 0.35, 0.38)),
            (xf(COLUMN ^ box(-30, 30, -50, 50, -110, 140), R), (0.30, 0.33, 0.37))]
    ours = [(xf(PARTS['holder']['solid'], R), PARTS['holder']['color'])]
    base = [(xf(CONTEXT['base'], R), (0.93, 0.55, 0.20))]
    cam = [(xf(CONTEXT['mast'], R), (0.78, 0.80, 0.83))]
    armp = [(xf(arm_moving(-60.0, 70.0, -10.0, 0.0), R), (0.94, 0.95, 0.96))]
    fig = plt.figure(figsize=(21, 13), facecolor=BG)
    draw(fig.add_subplot(2, 2, 1, projection='3d'), ours + base + cam + lift + armp, 24, -50,
         'mono holder on the C-Beam carriage: arm base, scene camera mast (front = +x)', 1.15)
    draw(fig.add_subplot(2, 2, 2, projection='3d'), ours + base + cam + lift, 28, 130, 'from behind: legs on the gantry plate, carriage facing the centre', 1.15)
    draw(fig.add_subplot(2, 2, 3, projection='3d'), ours + lift, 75, -90, 'from above: pad, post + bar over the arm (mast pocket, 4 inserts)', 1.1)
    draw(fig.add_subplot(2, 2, 4, projection='3d'), [(xf(PARTS['holder']['solid'], PARTS['holder']['print_rot']), PARTS['holder']['color'])], 30, -50,
         'H01 holder as printed (on its back: rear leg + post face on the bed)', 1.0)
    plt.subplots_adjust(left=0, right=1, bottom=0, top=0.96, wspace=0, hspace=0.06)
    plt.savefig(path, dpi=90, facecolor=BG); plt.close(fig)


if __name__ == '__main__':
    import mono_holder as MH
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = args[0] if args else os.path.join('..', '..', 'CAD', 'mono_holder')
    MH.build_all()
    if '--no_check' not in sys.argv:
        res, info, _ = MH.check(sweep='--no_sweep' not in sys.argv)
        for line in info: print(line)
        print('problems:', res if res else 'none')
    if '--no_export' not in sys.argv: MH.export(out)
