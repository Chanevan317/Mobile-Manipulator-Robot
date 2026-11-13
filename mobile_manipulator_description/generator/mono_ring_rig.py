"""Bench rig for the DRAFT mono shoulder ring (mono_ring.py), user 2026-11-11. Tests, before the rest of the mono arm
is built: the printed ball races (smoothness, play, preload), the gear + drive (backlash, servo multi-turn), homing,
the cable loop over +-180 deg, and the arm's tipping load.

Two new prints; every ring part (M03..M13 except the upper arm halves) is used as it is:
  rig_foot     base plate (C-clamped to a table edge) + a stand-in for the gantry plate (top edge at carriage z 20, with
               the same ESTIMATED M5 holes as the L bracket M12) + a hollow 40 x 80 beam stub through the hub
  test_lever   the arm's collar round the rotor (6 radial M3), as the upper arm; holes at 150 / 200 / 250 / 290 mm for weights.
               ~2.4 kg at 250 mm = the stretched arm + a 0.25 kg product (~6 N m)
Frame: the ring's carriage frame (mm). The lever points along +x (shoulder 0): clamp the rig with +x over the edge.

    python3 mono_ring_rig.py [out_dir]     (default ../../CAD/mono_ring_draft/bench_rig)
"""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import box, cyl, union, xf, T, Rx, Ry, Rz, print_mesh
from hw import screw_cut, insert_cut, Reg
import mono_ring as MR

FOOT_TOP = -85.0                 # below the lowest ring part (board mount -32, bracket leg -80)
FOOT_T = 6.0
BEAM_TOP = 120.0                 # beam stub top (through the hub and the top cap)
LEVER = dict(x0=70.0, x1=300.0, w=20.0, h=22.0, holes=(150.0, 200.0, 250.0, 290.0))   # top flush with the collar (z 65.5)
PARTS = {}


def rig_foot():
    r = Reg(); z0 = FOOT_TOP - FOOT_T
    foot = box(-95.0, 90.0, -90.0, 90.0, z0, FOOT_TOP)
    ribs = [box(-95.0, 90.0, y - 2.0, y + 2.0, FOOT_TOP - 0.01, FOOT_TOP + 8.0) for y in (-75.0, 75.0)]
    gantry = box(-27.5, -21.5, -62.5, 62.5, FOOT_TOP - 0.01, MR.DISC_Z[0])            # the kit plate's position; the disc sits on its top edge
    beam = box(-20.0, 20.0, -40.0, 40.0, FOOT_TOP - 0.01, BEAM_TOP) - box(-17.0, 17.0, -37.0, 37.0, FOOT_TOP + 3.0, BEAM_TOP + 1)
    s = union([foot, gantry, beam] + ribs)
    cut = []
    for (y, z) in MR.GANTRY_HOLES:          # the kit plate's holes the L bracket uses: M5 through, nyloc on the front face
        cut.append(xf(cyl(2.75, -1, 8, hole=True), T(-28.5, y, z) @ Ry(90)))
        r.add('SHCS+nyloc', 'M5', (-35.5, y, z), (1, 0, 0), 20, 'L bracket to the gantry stand-in')
    for (x, y) in ((-80.0, -60.0), (-80.0, 60.0), (75.0, -60.0), (75.0, 60.0)):      # optional screws into a board
        cut.append(cyl(2.75, z0 - 1, FOOT_TOP + 1, x, y, hole=True))
    cut.append(cyl(5.0, z0 - 1, FOOT_TOP + 1, 27.0, -45.0, hole=True))                 # cable exit under the board
    for (xa, xb) in ((-90.0, -47.0), (25.0, 85.0)):          # weight-saving windows (keep the rim, ribs and the stand-ins' roots)
        for (ya, yb) in ((-70.0, -46.0), (46.0, 70.0), (-34.0, 34.0)):
            if xa < 0 and abs(ya) < 40: continue           # under the beam / gantry stand-in roots: keep
            cut.append(box(xa, xb, ya, yb, z0 - 1, FOOT_TOP + 1))
    PARTS['rig_foot'] = dict(solid=s - union(cut), reg=r, color=(0.80, 0.80, 0.82), print_rot=np.eye(4),
                             note='lift stand-in: foot (C-clamp the rear corners, +x over the table edge), gantry plate '
                                  'stand-in (disc rests on its top edge, M12 bolts to it), 40 x 80 beam stub; prints flat')


def test_lever():
    L = LEVER; r = Reg(); zb = MR.TUBE_Z
    s = box(L['x0'], L['x1'], -L['w'], L['w'], zb, zb + L['h'])
    s = s - box(L['x0'] + 20.0, L['x1'] - 15.0, -L['w'] + 4.0, L['w'] - 4.0, zb + 4.0, zb + L['h'] + 1)   # open-top channel
    col, cut = MR.collar(r, 'lever collar to the rotor (radial), as the arm')   # the same collar as the upper arm
    s = union([s, col])
    for x in L['holes']:                     # weight holes (string / S-hook) through the floor
        cut.append(cyl(3.0, zb - 1, zb + 5, x, 0.0, hole=True))
    cut.append(box(L['x1'] - 3.0, L['x1'] + 1, -1.0, 1.0, zb + L['h'] - 3.0, zb + L['h'] + 1))   # tip mark (measure sag here)
    cut.append(box(L['x0'] - 1, L['x0'] + 18.0, -4.0, 4.0, zb + L['h'] - 6.0, zb + L['h'] + 1))  # cable from the tray exit
    PARTS['test_lever'] = dict(solid=s - union(cut), reg=r, color=(0.95, 0.75, 0.20), print_rot=Rx(180),
                               note='stands in for the upper arm: the same collar round the rotor (6 radial M3); weights at '
                                    '150 / 200 / 250 / 290 mm; prints upside down (top face + collar top on the bed)')


def build():
    MR.build_all(); PARTS.clear(); rig_foot(); test_lever()


def check():
    res = []
    fixed = {k: p['solid'] for k, p in MR.PARTS.items() if not p['moving']}
    fixed.update(MR.CONTEXT)
    foot = PARTS['rig_foot']['solid']
    for k, s in fixed.items():
        v = (foot ^ s).volume()
        if v > 0.5: res.append(f'rig foot x {k}: {v:.1f} mm3')
    v = (foot ^ xf(MR.SERVO, MR.servo_frame())).volume()
    if v > 0.5: res.append(f'rig foot x drive servo: {v:.1f} mm3')
    turning = [MR.PARTS[k]['solid'] for k in ('rotor_bottom', 'rotor_middle', 'rotor_top', 'cable_tray')] + [PARTS['test_lever']['solid']]
    allfixed = union(list(fixed.values()) + [foot])
    pin = MR.PARTS['ring_pinion']['solid'] - cyl(MR.MOD * MR.ZP / 2 + MR.MOD + 0.01, MR.GEAR_Z[0] - 0.01, MR.GEAR_Z[1] + 1, *MR.PIN_C)
    allfixed = union([allfixed - MR.PARTS['ring_pinion']['solid'], pin])
    worst = {}
    for a in range(0, 360, 10):
        v = (xf(union(turning), Rz(a)) ^ allfixed).volume()
        if v > 0.5: worst[a] = round(v, 1)
    if worst: res.append(f'lever / rotor hit fixed parts at {worst}')
    lev = PARTS['test_lever']['solid']
    for k in ('rotor_middle', 'rotor_top', 'cable_tray'):
        v = (lev ^ MR.PARTS[k]['solid']).volume()
        if v > 0.5: res.append(f'lever x {k}: {v:.1f} mm3')
    # load: moment and the lever's bending stress at its root (solid-wall estimate: 2 x 4 mm walls + 4 mm floor)
    m = 2.4 * 9.81 * 0.25
    return res, m


def export(out):
    from stlio import write_stl
    import csv
    for d in ('print_ready_stl', 'assembly_stl'):
        os.makedirs(os.path.join(out, d), exist_ok=True)
        for f in os.listdir(os.path.join(out, d)):
            if f.endswith('.stl'): os.remove(os.path.join(out, d, f))
    rows = []
    for i, (nm, p) in enumerate(PARTS.items(), 1):
        num = f'R{i:02d}_{nm}'
        V, F = print_mesh(p['solid']); write_stl(os.path.join(out, 'assembly_stl', num + '.stl'), V, F)
        s = xf(p['solid'], p['print_rot']); bb = s.bounding_box()
        s = s.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2])); bb = s.bounding_box()
        V, F = print_mesh(s); write_stl(os.path.join(out, 'print_ready_stl', num + '.stl'), V, F)
        rows.append(dict(file=num + '.stl', size_mm=f'{bb[3]-bb[0]:.1f} x {bb[4]-bb[1]:.1f} x {bb[5]-bb[2]:.1f}',
                         volume_cm3=round(p['solid'].volume() / 1000, 1), note=p['note']))
    with open(os.path.join(out, 'rig_parts.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    render(os.path.join(out, 'preview_ring_bench_rig.png'))
    return rows


def render(path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    items = [(p['solid'], p['color']) for k, p in MR.PARTS.items() if not k.startswith('mono_upper_arm')]
    items += [(MR.balls(), (0.75, 0.77, 0.80)), (xf(MR.SERVO, MR.servo_frame()), (0.12, 0.12, 0.14)),
              (MR.CONTEXT['servo_board'], (0.10, 0.35, 0.20))]
    items += [(p['solid'], p['color']) for p in PARTS.values()]

    def draw(ax, its, el, az, cut=None, zoom=1.0, center=None, title=''):
        tris, cols = [], []
        for s, c in its:
            if cut is not None: s = s ^ cut
            m = s.to_mesh(); t = np.asarray(m.vert_properties)[:, :3][np.asarray(m.tri_verts)]
            if len(t): tris.append(t); cols.append(np.tile(np.r_[c[:3]], (len(t), 1)))
        T_ = np.vstack(tris); C = np.vstack(cols)
        N = np.cross(T_[:, 1] - T_[:, 0], T_[:, 2] - T_[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
        Lv = np.array([np.cos(np.radians(el + 25)) * np.cos(np.radians(az + 30)), np.cos(np.radians(el + 25)) * np.sin(np.radians(az + 30)), np.sin(np.radians(el + 25))])
        sh = 0.35 + 0.65 * np.abs(N @ Lv)
        ax.add_collection3d(Poly3DCollection(T_, facecolors=np.c_[C * sh[:, None], np.ones(len(sh))], edgecolor='none'))
        lo, hi = T_.reshape(-1, 3).min(0), T_.reshape(-1, 3).max(0); c = (lo + hi) / 2 if center is None else np.array(center)
        r = (hi - lo).max() / 2 / zoom
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off(); ax.set_title(title, fontsize=13)
    fig = plt.figure(figsize=(20, 8), facecolor='white')
    draw(fig.add_subplot(1, 2, 1, projection='3d'), items, 25, -60, zoom=1.05,
         title='ring bench rig: foot (lift stand-in) + the ring parts + test lever (weights at 150..290 mm)')
    draw(fig.add_subplot(1, 2, 2, projection='3d'), items, 0, -90, cut=box(-300, 400, 0, 200, -200, 300), zoom=1.3,
         center=(60, 0, 20), title='section (y = 0): foot, gantry stand-in + L bracket, ball races, lever collar round the rotor')
    plt.tight_layout(); plt.savefig(path, dpi=55); plt.close(fig)


if __name__ == '__main__':
    import mono_ring_rig as RR
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join('..', '..', 'CAD', 'mono_ring_draft', 'bench_rig')
    RR.build()
    res, m = RR.check()
    print(f'test load: 2.4 kg at 250 mm = {m:.1f} N m tipping moment')
    print('problems:', res if res else 'none')
    for row in RR.export(out): print('  ', row['file'], row['size_mm'], f"{row['volume_cm3']} cm3")
