"""Case for the lift joystick (user, 2026-11-11): a KY-023 dual-axis thumbstick module (only its front-back axis is used:
push forward = lift up, pull back = down, let go = stop) + a Raspberry Pi Pico as the USB device. Both bought.

The Pico's dimensions are its datasheet's (exact). The KY-023's are ASSUMED from listings (they disagree: 34 x 26,
34 x 32, 40 x 27 mm; holes "M3.5, 27 / 20 mm"): MEASURE the real module and change KY below. The lid (with the
front-back slot around the stick's shaft) is the part that depends on them most; it's a separate, quick reprint.

Frame: mm, z up from the desk, x to the right, y = front (away from the operator = push = up).
    python3 lift_stick.py [out_dir]       (default ../../leader_arm_CAD/lift_stick)
"""
import math, os, sys, csv
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import box, cyl, union, xf, T, Rx, Ry, Rz, mesh_arrays

# Raspberry Pi Pico (datasheet): 51 x 21 x 1.0 mm board, 4 x dia 2.1 holes on 47.0 x 11.4, micro-USB on the -x end
PICO = dict(L=51.0, W=21.0, t=1.0, hole=2.1, hx=47.0 / 2, hy=11.4 / 2, usb=(8.0, 3.2), x=-30.0)
# KY-023 (ASSUMED, measure!): board L x W x t, hole dia and spacing, stick centre (from the board centre), dome top
# above the board's top face, shaft dia, the stock cap's underside above the board
KY = dict(L=34.0, W=26.0, t=1.6, hole=3.2, hx=27.0 / 2, hy=20.0 / 2, stick=(0.0, 0.0), dome=17.0, shaft=4.0, cap=22.0, x=32.0)
WALL, FLOOR, LID = 2.4, 3.0, 2.0
STAND_PICO = 4.0                                   # Pico standoff height (room for its bottom-side test pads)
GATE_Z = KY['dome'] + 1.0                          # slot plate underside above the KY board top: just over the dome
SLOT_HALF = 10.0                                   # slot half length (front-back): the stick's own stops limit the travel
PARTS = {}


def layout():
    """z of each board's top face, and the case's inside height."""
    ky_top = FLOOR + 6.0 + KY['t']                 # KY on 6 mm standoffs
    lid_z = ky_top + GATE_Z                        # lid underside
    return ky_top, lid_z


def body():
    ky_top, lid_z = layout()
    x0 = PICO['x'] - PICO['L'] / 2 - 4.0; x1 = KY['x'] + KY['L'] / 2 + 4.0
    y0 = -max(PICO['W'], KY['W']) / 2 - 7.0; y1 = -y0            # 7 mm: the lid screw posts clear the boards' corners
    outer = box(x0 - WALL, x1 + WALL, y0 - WALL, y1 + WALL, 0, lid_z)
    s = outer - box(x0, x1, y0, y1, FLOOR, lid_z + 1)
    posts = []
    for sx in (-1, 1):                             # Pico standoffs (M2 self-tap, dia 1.8 pilot)
        for sy in (-1, 1):
            x, y = PICO['x'] + sx * PICO['hx'], sy * PICO['hy']
            posts.append(cyl(2.4, FLOOR - 0.01, FLOOR + STAND_PICO, x, y) - cyl(0.9, FLOOR, FLOOR + STAND_PICO + 1, x, y, hole=True))
    for sx in (-1, 1):                             # KY standoffs (M3 self-tap, dia 2.6 pilot)
        for sy in (-1, 1):
            x, y = KY['x'] + sx * KY['hx'], sy * KY['hy']
            posts.append(cyl(3.0, FLOOR - 0.01, FLOOR + 6.0, x, y) - cyl(1.3, FLOOR, FLOOR + 7, x, y, hole=True))
    corners = [(x0 + 3.0, y0 + 3.0), (x0 + 3.0, y1 - 3.0), (x1 - 3.0, y0 + 3.0), (x1 - 3.0, y1 - 3.0)]
    for (x, y) in corners:                         # lid screw posts (M2.5 self-tap)
        posts.append(box(x - 3.0, x + 3.0, y - 3.0, y + 3.0, FLOOR - 0.01, lid_z) - cyl(1.0, FLOOR, lid_z + 1, x, y, hole=True))
    ears = [box(x - 7.0, x + 7.0, y0 - WALL - 10.0, y0 - WALL + 0.01, 0, FLOOR) for x in (x0 + 12.0, x1 - 12.0)]
    s = union([s] + posts + ears)
    pz = FLOOR + STAND_PICO + PICO['t']            # USB opening on the -x wall, centred on the Pico's connector
    cut = [box(x0 - WALL - 1, x0 + 0.01, -PICO['usb'][0] / 2 - 1.0, PICO['usb'][0] / 2 + 1.0, pz - 0.5, pz + PICO['usb'][1] + 1.5)]
    cut += [cyl(2.2, -1, FLOOR + 1, x, y0 - WALL - 5.0, hole=True) for x in (x0 + 12.0, x1 - 12.0)]   # M4 to the desk
    PARTS['stick_case'] = dict(solid=s - union(cut), print_rot=np.eye(4), color=(0.25, 0.25, 0.28),
                               note='case: Pico + KY-023 on standoffs, USB opening, 2 ears for M4 / rubber feet; prints flat')
    return (x0, x1, y0, y1, corners)


def lid(dims):
    x0, x1, y0, y1, corners = dims
    ky_top, lid_z = layout()
    s = box(x0 - WALL, x1 + WALL, y0 - WALL, y1 + WALL, lid_z, lid_z + LID)
    sx_, sy_ = KY['x'] + KY['stick'][0], KY['stick'][1]
    w = KY['shaft'] / 2 + 0.4
    slot = union([box(sx_ - w, sx_ + w, sy_ - SLOT_HALF + w, sy_ + SLOT_HALF - w, lid_z - 1, lid_z + LID + 1),
                  cyl(w, lid_z - 1, lid_z + LID + 1, sx_, sy_ - SLOT_HALF + w), cyl(w, lid_z - 1, lid_z + LID + 1, sx_, sy_ + SLOT_HALF - w)])
    cut = [slot] + [cyl(1.4, lid_z - 1, lid_z + LID + 1, x, y, hole=True) for (x, y) in corners]
    # arrow next to the slot: "up" toward +y (engraved 0.6 mm)
    ax = sx_ + 9.0
    cut += [box(ax - 1.0, ax + 1.0, sy_ - 6.0, sy_ + 4.0, lid_z + LID - 0.6, lid_z + LID + 1),
            box(ax - 3.0, ax + 3.0, sy_ + 4.0, sy_ + 6.0, lid_z + LID - 0.6, lid_z + LID + 1)]
    PARTS['stick_lid'] = dict(solid=s - union(cut), print_rot=Rx(180), color=(0.85, 0.45, 0.15),
                              note='lid with the front-back slot around the stick shaft (blocks sideways motion); '
                                   'depends most on the KY-023 measurements; prints top down')


def build():
    PARTS.clear(); dims = body(); lid(dims)


def boards():
    """Envelopes of the two boards + the stick (assumed), for the clash check and the assembly view."""
    ky_top, lid_z = layout()
    pz0 = FLOOR + STAND_PICO
    pico = box(PICO['x'] - PICO['L'] / 2, PICO['x'] + PICO['L'] / 2, -PICO['W'] / 2, PICO['W'] / 2, pz0, pz0 + PICO['t'] + 3.0)
    kz0 = ky_top - KY['t']
    ky = box(KY['x'] - KY['L'] / 2, KY['x'] + KY['L'] / 2, -KY['W'] / 2, KY['W'] / 2, kz0, ky_top)
    sx_, sy_ = KY['x'] + KY['stick'][0], KY['stick'][1]
    stick = union([box(sx_ - 8.0, sx_ + 8.0, sy_ - 8.0, sy_ + 8.0, ky_top, ky_top + KY['dome']),
                   cyl(KY['shaft'] / 2, ky_top, ky_top + KY['cap'], sx_, sy_), cyl(12.0, ky_top + KY['cap'], ky_top + KY['cap'] + 8.0, sx_, sy_)])
    return union([pico, ky, stick])


def check():
    b = boards(); res = []
    for k, p in PARTS.items():
        v = (p['solid'] ^ b).volume()
        if v > 0.5: res.append((k, round(v, 1)))
    ky_top, lid_z = layout()
    return res, lid_z, ky_top


def export(out):
    from stlio import write_stl
    for d in ('print_ready_stl', 'assembly_stl'):
        os.makedirs(os.path.join(out, d), exist_ok=True)
        for f in os.listdir(os.path.join(out, d)):
            if f.endswith('.stl'): os.remove(os.path.join(out, d, f))
    rows = []
    for i, (nm, p) in enumerate(PARTS.items(), 1):
        num = f'S{i}_{nm}'
        V, F = mesh_arrays(p['solid']); write_stl(os.path.join(out, 'assembly_stl', num + '.stl'), V, F)
        s = xf(p['solid'], p['print_rot']); bb = s.bounding_box(); s = s.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2]))
        bb = s.bounding_box(); V, F = mesh_arrays(s); write_stl(os.path.join(out, 'print_ready_stl', num + '.stl'), V, F)
        rows.append(dict(file=num + '.stl', size_mm=f'{bb[3]-bb[0]:.1f} x {bb[4]-bb[1]:.1f} x {bb[5]-bb[2]:.1f}',
                         volume_cm3=round(p['solid'].volume() / 1000, 1), note=p['note']))
    V, F = mesh_arrays(boards()); write_stl(os.path.join(out, 'assembly_stl', 'boards_assumed.stl'), V, F)
    with open(os.path.join(out, 'parts.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    return rows


if __name__ == '__main__':
    import lift_stick as LS
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join('..', '..', 'leader_arm_CAD', 'lift_stick')
    LS.build()
    res, lid_z, ky_top = LS.check()
    print('boards / stick clear of the case and lid:', 'yes' if not res else res)
    print(f'lid underside {lid_z:.1f} mm above the desk; KY board top {ky_top:.1f}; slot {2 * LS.SLOT_HALF:.0f} mm front-back x '
          f'{LS.KY["shaft"] + 0.8:.1f} mm')
    for r in LS.export(out): print('  ', r['file'], r['size_mm'], r['volume_cm3'], 'cm3')


def render(path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    case, lidp = PARTS['stick_case'], PARTS['stick_lid']

    def draw(ax, its, el, az, zoom=1.0, cut=None, title=''):
        tris, cols = [], []
        for s, c in its:
            if cut is not None: s = s ^ cut
            m = s.to_mesh(); t = np.asarray(m.vert_properties)[:, :3][np.asarray(m.tri_verts)]
            if len(t): tris.append(t); cols.append(np.tile(np.r_[c[:3]], (len(t), 1)))
        T_ = np.vstack(tris); C = np.vstack(cols)
        N = np.cross(T_[:, 1] - T_[:, 0], T_[:, 2] - T_[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
        L = np.array([np.cos(np.radians(el + 25)) * np.cos(np.radians(az + 30)), np.cos(np.radians(el + 25)) * np.sin(np.radians(az + 30)), np.sin(np.radians(el + 25))])
        sh = 0.35 + 0.65 * np.abs(N @ L)
        ax.add_collection3d(Poly3DCollection(T_, facecolors=np.c_[C * sh[:, None], np.ones(len(sh))], edgecolor='none'))
        lo, hi = T_.reshape(-1, 3).min(0), T_.reshape(-1, 3).max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2 / zoom
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off(); ax.set_title(title, fontsize=13)
    b = (boards(), (0.10, 0.35, 0.20))
    fig = plt.figure(figsize=(20, 7), facecolor='white')
    draw(fig.add_subplot(1, 3, 1, projection='3d'), [(case['solid'], case['color']), (lidp['solid'], lidp['color']), b], 30, -60, zoom=1.1,
         title='assembled (stick + cap above the lid)')
    draw(fig.add_subplot(1, 3, 2, projection='3d'), [(case['solid'], case['color']), (lidp['solid'].translate((0, 0, 30)), lidp['color']), b],
         35, -60, zoom=1.0, title='lid lifted: Pico (left, USB out the end), KY-023 (right)')
    sx_ = KY['x'] + KY['stick'][0]
    draw(fig.add_subplot(1, 3, 3, projection='3d'), [(case['solid'], case['color']), (lidp['solid'], lidp['color']), b], 0, 180, zoom=1.6,
         cut=box(sx_, 200, -200, 200, -10, 200), title='cut through the stick: slot just over the dome, around the shaft')
    plt.tight_layout(); plt.savefig(path, dpi=60); plt.close(fig)
