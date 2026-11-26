"""Dual-arm holder (user 2026-11-13: printed PETG, ONE piece around the beam): carries both arm bases (01 base_top +
02 base_bottom, the prototype's own parts) on the real C-Beam XLarge gantry plate. Replaces the two estimated 8 mm
aluminium brackets of the URDF.

Units mm, CARRIAGE frame: origin on the column axis, x forward (working face), z = carriage z (gantry plate top z 20).
The arm bases stay where they were: shoulder axes at (0, +-150), mount yaw +-90 deg (+-135 until 2026-11-25), base underside z 25.5.

    pads        12 mm under each arm base (its footprint + 4 mm), z 13.5..25.5; 4 x M5 x 20 from below into the
                base's own M5 x 8 inserts
    front bridge  in front of the beam (x 21..60, 1 mm clear of its front face), joins both pads; carries the scene
                scene camera mast's 4 M3 inserts (scene_cam.py)
    cheeks      beside the gantry plate and its wheels (|y| 63..75), x -40..60, z -40..25.5
    legs        behind the plate (x -40..-28), bolted through the plate's own 5.2 mm holes at y +-50 / z -12.5, -72.5
                (nylocs on the plate's front face, as the mono ring's bracket); a lip rests on the plate's top edge
    rear bridge behind the plate (x -40..-31), closes the frame
Nothing stands over the beam above the plate top within the top end mount's footprint (x -30..20, |y| <= 40), so the
lift keeps its full 0.871 m. Too long for the A1 (380 mm): printed as two halves (y >= 0 / y <= 0) joined by lap
joints in both bridges (6 x M5 + nylocs).

    python3 dual_holder.py [out_dir]     checks + export (default ../../CAD/dual_holder)
"""
import math, os, sys, csv, collections
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import M, CS, box, cyl, union, xf, T, Rx, Ry, Rz, print_mesh
from hw import screw_cut, insert_cut, Reg

ARM_Y, YAW, AZ = 150.0, 90.0, 65.5           # urdf_build PROTO_ARM_Y, PROTO_YAW, ARM_Z (yaw 135 until 2026-11-25)
BASE_Z = AZ - 40.0                            # base underside (its mount face is 40 mm under the servo mid-plane)
PAD = (BASE_Z - 12.0, BASE_Z)                 # pad z
# the dual robot's base (2026-11-25, user: try the mono's 0.45): square mecanum base, side BASE; the column (and the arms
# beside it) ROBOT_X forward of the base centre, 0.07 behind the front edge as on the 0.50 base (column 0.18) and the mono
BASE = 0.45
ROBOT_X = round(BASE / 2 - 0.07, 3)
# the dual robot's tote on the base (robot frame, m): a FLAT box behind the column, inner faces 7 mm inside the deck's back /
# side edges, front face 0.082 behind the column axis, floor top 0.263, walls 0.12. A funnel floor as the mono's was tried
# (2026-11-25): the arms reach all but a ~10 cm back strip, and the dropped boxes (lying, ~20 cm long) rest their ends on
# any slope there: food box b rolled 10 cm onto its side in Isaac. Flat it is.
def dual_tote():
    e = round(BASE / 2 - 0.007, 3); x1 = round(ROBOT_X - 0.082, 3)
    return dict(x=(-e, x1), y=(-e, e), z=(0.256, 0.376), wall=0.007)


GANTRY_CLEAR = dict(x=(-28.5, 1.5), y=63.5, z=20.5)   # pads stay above the gantry plate + V-wheel tops (z 20) here
BASE_M5 = [(-80.0, 13.5), (-80.0, -13.5), (-54.0, 13.5), (-54.0, -13.5)]
FRONT = dict(x=(21.0, 60.0), y=95.0, z=(1.5, BASE_Z), zm=12.5)   # 24 mm deep; lap face 12.5 (2026-11-16, was 13.5): 11 mm layers under the mast pocket (strength check: 10 mm gave SF 2.8, 8 mm 2.6 in bolt bearing)
REAR = dict(x=(-40.0, -31.0), y=75.0, z=(0.0, BASE_Z), zm=12.75)
CHEEK = dict(x=(-40.0, 60.0), y=(63.0, 75.0), z=(-40.0, BASE_Z))
LEG = dict(x=(-40.0, -28.0), y=(44.0, 75.0), z=(-80.0, BASE_Z))
LIP = dict(x=(-28.01, -21.0), y=(41.0, 62.5), z=(20.0, BASE_Z))
LAP_Y = 20.0
# scene camera mast (scene_cam.py, 2026-11-13): 4 x M3 x 8 inserts in the front bridge top for its flange (x 22..62, |y| <= 22)
CAM_INSERTS = [(42.0 + sx * 14.0, sy * 17.0) for sx in (-1, 1) for sy in (-1, 1)]
# the mast's bottom flange sits in a 2 mm pocket (2026-11-16): its screw heads are then 2 mm lower, so the arms' fork
# prongs pass > 2 mm over the flange (they passed 0.4 mm over it). A channel across the whole bridge (the flange is
# x 22..62, the bridge x 21..60: a closed back wall would be a 0.8 mm sliver); its side walls locate the mast in y.
CAM_POCKET = dict(depth=2.0, gap=0.2, hy=22.0)
CAM_Z = FRONT['z'][1] - CAM_POCKET['depth']     # 23.5: the mast's mount face (scene_cam.MOUNT)
LAP_INS = 6                                     # front lap inserts M5 x 6 (an M5 x 8 would leave 2.5 mm under the pocket floor; now 4.5)
LAP_CB = 0.0                                    # front lap screw heads flat on the underside: M5 x 16 goes 5 mm into the insert
# lap joints: front pair = M5 from below into M5 x 8 inserts in the upper layer (the camera flange covers the bridge top);
# rear pair = M5 through + nyloc underneath
SPLICE = [(35.0, 12.0), (35.0, -12.0), (50.0, 12.0), (50.0, -12.0), (-35.5, 12.0), (-35.5, -12.0)]
GANTRY_HOLES = [(sy * 50.0, z) for sy in (-1, 1) for z in (-12.5, -72.5)]

# the kit's lift parts (carriage frame), as in mono_ring.py
GANTRY = box(-27.5, -21.5, -62.5, 62.5, -105.0, 20.0)
WHEELS = union([box(-16.0, 0.5, 38.0, 62.0, -105.0, 20.0), box(-16.0, 0.5, -62.0, -38.0, -105.0, 20.0)])
COLUMN = box(-20.0, 20.0, -40.0, 40.0, -300.0, 600.0)
TOP_MOUNT = box(-30.0, 20.0, -40.0, 40.0, 20.0, 40.0)        # the top end mount's footprint above the plate top
FOOT = box(20.0, 26.0, -40.0, 40.0, -300.0, -54.0)           # the column foot angle (base z <= 0.366) at lift 0

PARTS, CONTEXT = {}, {}


def arm_frame(sy):
    return T(0, sy * ARM_Y, AZ) @ Rz(sy * YAW)


def base_parts():
    import build, arm
    if not arm.PARTS or 'base_top' not in arm.PARTS:
        build.build_parts(build.frames((0, 0, 0), 0, 0, 60.0))
    return union([arm.PARTS[k]['solid'] for k in ('base_top', 'base_bottom')])


def pad(sy, r):
    sh = (CS.circle(40.0, 96) + CS([[(-89.0, -24.5), (-46.0, -24.5), (-46.0, 24.5), (-89.0, 24.5)]])).hull()
    p = xf(sh.extrude(PAD[1] - PAD[0]).translate((0, 0, PAD[0] - AZ)), arm_frame(sy))
    cut = []
    for (x, y) in BASE_M5:
        q = arm_frame(sy) @ np.array([x, y, PAD[0] - AZ, 1.0])
        cut.append(screw_cut(tuple(q[:3]), (0, 0, 1), 'M5', PAD[1] - PAD[0], cb=0.0))
        r.add('SHCS', 'M5', tuple(q[:3]), (0, 0, 1), 20, 'arm base (01 + 02) to its pad, from below into its M5 x 8 inserts')
    return p, cut


def holder():
    r = Reg(); cut = []
    parts = []
    for sy in (-1, 1):
        p, c = pad(sy, r); cut += c
        # since the mounts turned to +-90 (2026-11-25) each base's pillar points at the column: its pad's inner end lies over
        # the gantry plate and the V-wheels (tops z 20). Keep the pad 0.5 mm above them there (a 5 mm skin; no screws there:
        # the base's M5s are at |y| 70 / 96)
        p = p - box(GANTRY_CLEAR['x'][0], GANTRY_CLEAR['x'][1], -GANTRY_CLEAR['y'], GANTRY_CLEAR['y'], PAD[0] - 1.0, GANTRY_CLEAR['z'])
        parts.append(p)
        y0, y1 = sorted((sy * CHEEK['y'][0], sy * CHEEK['y'][1]))
        parts.append(box(*CHEEK['x'], y0, y1, *CHEEK['z']))
        y0, y1 = sorted((sy * LEG['y'][0], sy * LEG['y'][1]))
        parts.append(box(*LEG['x'], y0, y1, *LEG['z']))
        y0, y1 = sorted((sy * LIP['y'][0], sy * LIP['y'][1]))
        parts.append(box(*LIP['x'], y0, y1, *LIP['z']))
    parts.append(box(*FRONT['x'], -FRONT['y'], FRONT['y'], *FRONT['z']))
    parts.append(box(*REAR['x'], -REAR['y'], REAR['y'], *REAR['z']))
    s = union(parts)
    hy = CAM_POCKET['hy'] + CAM_POCKET['gap']
    cut.append(box(FRONT['x'][0] - 1, FRONT['x'][1] + 1, -hy, hy, CAM_Z, FRONT['z'][1] + 1))      # mast flange pocket
    for (x, y) in CAM_INSERTS:                                    # scene camera mast flange
        cut.append(insert_cut((x, y, CAM_Z), (0, 0, -1), 'M3', length=8)); r.add('insert', 'M3', (x, y, CAM_Z), (0, 0, -1), 8, 'scene camera mast flange (pocket floor)')
    for (y, z) in GANTRY_HOLES:                                   # through the plate's own holes, nyloc in front of it
        p = (LEG['x'][0], y, z)
        cut.append(screw_cut(p, (1, 0, 0), 'M5', (LEG['x'][1] - LEG['x'][0]) + 6.0 + 4.0)); r.add('SHCS+nyloc', 'M5', p, (1, 0, 0), 25, 'legs to the gantry plate (its 5.2 mm holes at y +-50)')
    for (x, y) in SPLICE:                                         # the two halves' lap joints
        if x > 0:                                                 # front: from below into an insert in the upper layer
            z0, zm = FRONT['z'][0], FRONT['zm']
            cut.append(screw_cut((x, y, z0 + LAP_CB), (0, 0, 1), 'M5', zm - z0 - LAP_CB, cb=LAP_CB + 1.0 if LAP_CB else 0.0))
            cut.append(insert_cut((x, y, zm), (0, 0, 1), 'M5', length=LAP_INS))
            r.add('SHCS', 'M5', (x, y, z0 + LAP_CB), (0, 0, 1), 16, 'front lap joint, from below into the insert')
            r.add('insert', 'M5', (x, y, zm), (0, 0, 1), LAP_INS, 'front lap joint (upper layer, from the lap face)')
            continue
        z0, z1 = REAR['z']
        p = (x, y, z1)
        cut.append(screw_cut(p, (0, 0, -1), 'M5', z1 - z0, cb=0.0))
        r.add('SHCS+nyloc', 'M5', p, (0, 0, -1), int(math.ceil((z1 - z0 + 6.0) / 5.0) * 5), 'lap joint between the halves (nyloc underneath)')
    s = s - union(cut)
    CONTEXT['holder'] = s
    # split into two printable halves with lap joints in both bridges
    up = union([box(*FRONT['x'], -LAP_Y, LAP_Y, FRONT['zm'], 300), box(*REAR['x'], -LAP_Y, LAP_Y, REAR['zm'], 300)])
    lap = union([box(*FRONT['x'], -LAP_Y, LAP_Y, -300, 300), box(*REAR['x'], -LAP_Y, LAP_Y, -300, 300)])
    left = (s ^ box(-300, 300, LAP_Y, 400, -300, 300)) + ((s ^ lap) ^ up)
    left = left + (s ^ box(-300, 300, 0, LAP_Y, -300, 300) - lap)           # outside the bridges nothing crosses y 0..20
    right = (s ^ box(-300, 300, -400, -LAP_Y, -300, 300)) + ((s ^ lap) - up)
    right = right + (s ^ box(-300, 300, -LAP_Y, 0, -300, 300) - lap)
    rl, rr = Reg(), Reg()
    for it in r.items:
        (rl if it['p'][1] >= 0 else rr).items.append(it)
    PARTS['holder_left'] = dict(solid=left, color=(0.20, 0.55, 0.62), reg=rl, material='PETG', print_rot=Rx(-90),
                                note='left half (y >= 0): pad, cheek, leg, upper layer of both laps, scene camera mast inserts; prints on its outer side face (cheek down)')
    PARTS['holder_right'] = dict(solid=right, color=(0.18, 0.50, 0.58), reg=rr, material='PETG', print_rot=Rx(90),
                                 note='right half (y <= 0): pad, cheek, leg, lower layer of both laps; prints on its outer side face (cheek down)')


def mast_post():
    """The scene camera mast (scene_cam.py, two-arm version) as a stand-in: its flange in the pocket + its box (x 26..62,
    |y| <= 13) up to z 300."""
    return union([box(22.0, 62.0, -22.0, 22.0, CAM_Z, CAM_Z + 6.0), box(26.0, 62.0, -13.0, 13.0, CAM_Z, 300.0)])


def build_all():
    PARTS.clear(); CONTEXT.clear()
    holder()
    b = base_parts()
    CONTEXT['bases'] = union([xf(b, arm_frame(sy)) for sy in (-1, 1)])


def arm_moving(sy, q1, q2, q3, pitch, opening=60.0):
    """One arm's moving parts (upper arm .. fingers + their servos / bearings) in the carriage frame."""
    import build, arm
    F = build.frames((q1, q2, q3), pitch, 0.0, opening)
    build.build_parts(F)
    G = arm_frame(sy)
    out = [xf(p['solid'], G @ p['frame']) for nm, p in arm.PARTS.items() if not nm.startswith('base')]
    out += [xf(b['solid'], G) for b in build.bought(F) if not ('shoulder' in b['name'] or '_J1_' in b['name'])]
    return union(out)


def check():
    res, info = [], []
    h = CONTEXT['holder']
    for nm, e in (('gantry plate', GANTRY), ('V-wheels', WHEELS), ('beam (1 mm clearance)', xf(COLUMN, np.eye(4)) + box(-21, 21, -41, 41, -300, 600)),
                  ('top end mount footprint (above the plate top)', TOP_MOUNT), ('column foot at lift 0', FOOT)):
        v = (h ^ e).volume()
        if v > 0.5: res.append(f'holder x {nm}: {v:.1f} mm3')
    v = (h ^ CONTEXT['bases']).volume()
    if v > 0.5: res.append(f'holder x arm bases: {v:.1f} mm3')
    v = (mast_post() ^ CONTEXT['bases']).volume()
    if v > 0.5: res.append(f'camera post x arm bases: {v:.1f} mm3')
    # each half is one solid and the two halves don't overlap
    for k in ('holder_left', 'holder_right'):
        n = len([c for c in PARTS[k]['solid'].decompose() if c.volume() > 1.0])
        if n != 1: res.append(f'{k} is {n} pieces')
    v = (PARTS['holder_left']['solid'] ^ PARTS['holder_right']['solid']).volume()
    if v > 0.5: res.append(f'the halves overlap: {v:.1f} mm3')
    # arm sweep: each arm's moving parts against the holder + camera post (the URDF merges both into the carriage,
    # where urdf_selfcol / Isaac never check them against the arms)
    fixed = union([h, mast_post()])
    hits = collections.Counter(); n = 0
    for sy in (1,):                                   # the holder is symmetric (only the laps / socket differ): left arm
        for q1 in range(-115, 116, 23):
            for q2 in range(-125, 126, 50):
                for q3 in (-122, 0, 122):
                    for pitch in (0, 90):
                        n += 1
                        if (arm_moving(sy, q1, q2, q3, pitch) ^ fixed).volume() > 0.5:
                            hits[(sy, q1, q2, q3, pitch)] += 1
    info.append(f'arm sweep: {n} poses (left arm; the holder is symmetric); touching the holder or the camera post: {len(hits)}')
    if hits: info.append(f'  {sorted(hits)[:16]}')
    vol = sum(p['solid'].volume() for p in PARTS.values()) / 1000
    info.append(f'holder {vol:.0f} cm3 PETG (~{vol * 0.80 * 1.27:.0f} g at the usual fill estimate)')
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
        num = f'D{i:02d}_{nm}'
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
    _write(os.path.join(ctx, 'arm_bases_01_02.stl'), CONTEXT['bases'])
    _write(os.path.join(ctx, 'lift_kit_gantry_wheels_beam.stl'), union([GANTRY, WHEELS, COLUMN ^ box(-30, 30, -50, 50, -110, 140)]))
    _write(os.path.join(ctx, 'camera_post.stl'), mast_post())
    with open(os.path.join(out, 'printed_parts.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    with open(os.path.join(out, 'fasteners.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['kind', 'size', 'length_mm', 'qty', 'where'])
        for (k, s, L, n), q in sorted(fast.items(), key=lambda t: (str(t[0][1]), t[0][0], str(t[0][2]))):
            w.writerow([k, s, L, q, n])
    render(os.path.join(out, 'preview_dual_holder.png'))
    print(f'exported {len(rows)} parts to {out}')


def render(path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    BG = (0.906, 0.922, 0.941)

    def draw(ax, items, el, az, title, zoom=1.0, lim=None):
        T_, C_ = [], []
        for s, c in items:
            m = s.to_mesh(); t = np.asarray(m.vert_properties)[:, :3][np.asarray(m.tri_verts)]
            if len(t): T_.append(t); C_.append(np.tile(np.r_[c[:3]], (len(t), 1)))
        T_ = np.vstack(T_); C_ = np.vstack(C_)
        N = np.cross(T_[:, 1] - T_[:, 0], T_[:, 2] - T_[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
        Ld = np.array([math.cos(math.radians(el + 25)) * math.cos(math.radians(az + 30)),
                       math.cos(math.radians(el + 25)) * math.sin(math.radians(az + 30)), math.sin(math.radians(el + 25))])
        sh = 0.35 + 0.65 * np.abs(N @ Ld)
        ax.add_collection3d(Poly3DCollection(T_, facecolors=np.c_[C_ * sh[:, None], np.ones(len(sh))], edgecolor='none'))
        lo, hi = (T_.reshape(-1, 3).min(0), T_.reshape(-1, 3).max(0)) if lim is None else lim
        c = (lo + hi) / 2; r = (hi - lo).max() / 2 / zoom
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off(); ax.set_title(title, fontsize=13); ax.set_facecolor(BG)

    lift = [(GANTRY, (0.80, 0.80, 0.82)), (WHEELS, (0.35, 0.35, 0.38)), (COLUMN ^ box(-30, 30, -50, 50, -110, 200), (0.30, 0.33, 0.37))]
    halves = [(PARTS['holder_left']['solid'], PARTS['holder_left']['color']), (PARTS['holder_right']['solid'], PARTS['holder_right']['color'])]
    bases = [(CONTEXT['bases'], (0.93, 0.55, 0.20)), (mast_post(), (0.78, 0.80, 0.83))]
    fig = plt.figure(figsize=(21, 13), facecolor=BG)
    draw(fig.add_subplot(2, 2, 1, projection='3d'), halves + bases + lift, 26, -55, 'dual-arm holder with both arm bases (01 + 02) on the C-Beam carriage', 1.2)
    draw(fig.add_subplot(2, 2, 2, projection='3d'), halves + lift, 30, 125, 'from behind: legs on the gantry plate, rear bridge', 1.2)
    draw(fig.add_subplot(2, 2, 3, projection='3d'), halves + lift, 70, -90, 'from above: pads, front bridge (camera mast inserts), cheeks', 1.1)
    expl = [(xf(PARTS['holder_left']['solid'], T(0, 40, 0)), PARTS['holder_left']['color']),
            (xf(PARTS['holder_right']['solid'], T(0, -40, 0)), PARTS['holder_right']['color'])]
    draw(fig.add_subplot(2, 2, 4, projection='3d'), expl, 28, -55, 'the two printed halves and their lap joints', 1.2)
    plt.subplots_adjust(left=0, right=1, bottom=0, top=0.96, wspace=0, hspace=0.06)
    plt.savefig(path, dpi=90, facecolor=BG); plt.close(fig)


if __name__ == '__main__':
    import dual_holder as DH
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = args[0] if args else os.path.join('..', '..', 'CAD', 'dual_holder')
    DH.build_all()
    if '--no_check' not in sys.argv:
        res, info, _ = DH.check()
        for line in info: print(line)
        print('problems:', res if res else 'none')
    if '--no_export' not in sys.argv: DH.export(out)
