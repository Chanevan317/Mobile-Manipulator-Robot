"""HANDHELD case for the lift joystick (user, 2026-11-11; on the table or in the hand): a KY-023 dual-axis thumbstick module (only its front-back axis is used:
push forward = lift up, pull back = down, let go = stop) + a Raspberry Pi Pico as the USB device. Both bought.

The Pico's dimensions are its datasheet's (exact). The KY-023's are ASSUMED from listings (they disagree: 34 x 26,
34 x 32, 40 x 27 mm; holes "M3.5, 27 / 20 mm"): MEASURE the real module and change KY below. The lid (with the
front-back slot around the stick's shaft) is the part that depends on them most; it's a separate, quick reprint.

Frame: mm, z up from the case bottom, x = toward the tip (= push = up), y to the left.
    python3 lift_stick.py [out_dir]       (default ../../leader_arm_CAD/lift_stick)
"""
import math, os, sys, csv
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import box, cyl, union, xf, T, Rx, Ry, Rz, mesh_arrays

# HANDHELD (user, 2026-11-11: "hand held like it can be placed on the table or lifted up and held by hand"; then "I
# didn't see you redesign anything"): a WEDGE like a controller grip. Thin at the back (14 mm, in the palm), full height
# from the middle to the front (29 mm, room for the stick), all edges rounded (hull of spheres), flat bottom for the
# table. Split like a remote: a bottom tray holds both boards, a top shell carries the slot; 3 screws from UNDERNEATH
# (nothing on the top under the hand). x = toward the tip (= push = lift UP), the Pico at the back (USB out the back
# end, toward the wrist), the stick near the front under the thumb.
# Raspberry Pi Pico (datasheet): 51 x 21 x 1.0 mm board, 4 x dia 2.1 holes on 47.0 x 11.4, micro-USB on one short end
PICO = dict(L=51.0, W=21.0, t=1.0, hole=2.1, hx=47.0 / 2, hy=11.4 / 2, usb=(8.0, 3.2), x=1.8 + 25.5)
# KY-023 (ASSUMED, measure!): board L x W x t, hole dia and spacing, stick centre (from the board centre), dome top
# above the board's top face, shaft dia, the stock cap's underside above the board
KY = dict(L=34.0, W=26.0, t=1.6, hole=3.2, hx=27.0 / 2, hy=20.0 / 2, stick=(0.0, 0.0), dome=17.0, shaft=4.0, cap=22.0, x=58.0 + 17.0)
WALL, FLOOR = 2.4, 2.4
RS = 7.0                                           # rounding radius of the outside
SH = dict(x0=-2.4, x1=101.4, hw=18.7, h_back=14.0, h_full=29.2, x_full=52.0)   # outside: back / front x, half width, heights
SPLIT_Z = 7.0                                      # bottom tray / top shell
STAND_PICO, STAND_KY = 3.0, 5.0
SLOT_HALF = 10.0                                   # slot half length (front-back): the stick's own stops limit the travel
BOX_R = 400.0                                      # on the table: keep its centre >= this far from the leader's pedestal axis
LOW_R = 313.0                                      # leader parts come below desk + 50 mm within this radius (610-pose sample)
SCREWS = [(3.5, 13.5), (3.5, -13.5), (96.0, 0.0)]  # M2.5 from underneath, into the top shell's bosses
PARTS = {}


def layout():
    """z of the KY board's top face and of the top shell's inside at the stick."""
    ky_top = FLOOR + STAND_KY + KY['t']
    return ky_top, SH['h_full'] - WALL


def shell(e):
    """Outside (e = 0) or inside (e = -WALL) of the wedge: hull of spheres (offset = smaller spheres, same centres)."""
    from lib import M
    r = RS + e; c = []
    yb = SH['hw'] - RS
    xb, xm, xf_ = SH['x0'] + RS, SH['x_full'], SH['x1'] - RS
    for y in (-yb, yb):
        c += [(xb, y, RS), (xb, y, SH['h_back'] - RS if SH['h_back'] - RS > RS else RS),
              (xm, y, RS), (xm, y, SH['h_full'] - RS), (xf_, y, RS), (xf_, y, SH['h_full'] - RS)]
    return M.batch_hull([M.sphere(r, 32).translate(p) for p in c])


def body():
    ky_top, ceil = layout()
    outer, inner = shell(0.0), shell(-WALL)
    wall = outer - inner
    low = box(-50, 150, -50, 50, -1, SPLIT_Z); high = box(-50, 150, -50, 50, SPLIT_Z, 60)
    posts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = PICO['x'] + sx * PICO['hx'], sy * PICO['hy']                      # Pico standoffs (M2 self-tap)
            posts.append(cyl(2.4, FLOOR - 0.5, FLOOR + STAND_PICO, x, y) - cyl(0.9, FLOOR, FLOOR + STAND_PICO + 1, x, y, hole=True))
            x, y = KY['x'] + sx * KY['hx'], sy * KY['hy']                            # KY standoffs (M3 self-tap)
            posts.append(cyl(3.0, FLOOR - 0.5, FLOOR + STAND_KY, x, y) - cyl(1.3, FLOOR, FLOOR + STAND_KY + 1, x, y, hole=True))
    bosses = union([cyl(2.8, 0, 60, x, y) for (x, y) in SCREWS]) ^ inner
    from lib import cone
    scut = union([cyl(1.4, -1, SPLIT_Z + 0.2, x, y, hole=True) for (x, y) in SCREWS] +              # clearance through the tray
                 [cone(2.7, 1.4, -0.01, 1.3, x, y) for (x, y) in SCREWS] +                          # countersink underneath
                 [cyl(1.0, SPLIT_Z, 60, x, y, hole=True) for (x, y) in SCREWS])                     # pilot in the top shell
    usb_z = FLOOR + STAND_PICO + PICO['t']
    usb = box(SH['x0'] - 1, 0.5, -PICO['usb'][0] / 2 - 1.5, PICO['usb'][0] / 2 + 1.5, usb_z - 1.0, usb_z + PICO['usb'][1] + 2.0)
    feet = union([cyl(5.0, -1, 1.0, x, y) for x in (16.0, 86.0) for y in (-9.0, 9.0)])
    tray = union([(wall ^ low), (bosses ^ low)] + posts) - union([scut, usb, feet])
    sx_, sy_ = KY['x'] + KY['stick'][0], KY['stick'][1]
    w = KY['shaft'] / 2 + 0.4
    slot = union([box(sx_ - SLOT_HALF + w, sx_ + SLOT_HALF - w, sy_ - w, sy_ + w, ceil - 5, 60),
                  cyl(w, ceil - 5, 60, sx_ - SLOT_HALF + w, sy_), cyl(w, ceil - 5, 60, sx_ + SLOT_HALF - w, sy_)])
    ay, zt = sy_ + 9.0, SH['h_full']
    arrow = union([box(sx_ - 6.0, sx_ + 4.0, ay - 1.0, ay + 1.0, zt - 0.6, zt + 1), box(sx_ + 4.0, sx_ + 6.0, ay - 3.0, ay + 3.0, zt - 0.6, zt + 1)])
    top = union([(wall ^ high), (bosses ^ high)]) - union([scut, usb, slot, arrow])
    PARTS['stick_tray'] = dict(solid=tray, print_rot=np.eye(4), color=(0.25, 0.25, 0.28),
                               note='bottom tray: Pico (back) + KY-023 (front) on standoffs, USB opening, 3 countersunk M2.5 '
                                    'from underneath, 4 rubber-feet recesses; prints flat')
    PARTS['stick_top'] = dict(solid=top, print_rot=Rx(180), color=(0.85, 0.45, 0.15),
                              note='top shell (the wedge): front-back slot around the stick shaft, "up" arrow to the tip; '
                                   'depends most on the KY-023 measurements; prints upside down')


def build():
    PARTS.clear(); body()


def boards():
    """Envelopes of the two boards + the stick (assumed), for the clash check and the views."""
    ky_top, ceil = layout()
    pz0 = FLOOR + STAND_PICO
    pico = box(PICO['x'] - PICO['L'] / 2, PICO['x'] + PICO['L'] / 2, -PICO['W'] / 2, PICO['W'] / 2, pz0, pz0 + PICO['t'] + 3.0)
    kz0 = ky_top - KY['t']
    ky = box(KY['x'] - KY['L'] / 2, KY['x'] + KY['L'] / 2, -KY['W'] / 2, KY['W'] / 2, kz0, ky_top)
    sx_, sy_ = KY['x'] + KY['stick'][0], KY['stick'][1]
    stick = union([box(sx_ - 8.0, sx_ + 8.0, sy_ - 8.0, sy_ + 8.0, ky_top, ky_top + KY['dome']),
                   cyl(KY['shaft'] / 2, ky_top, ky_top + KY['cap'], sx_, sy_), cyl(12.0, ky_top + KY['cap'], ky_top + KY['cap'] + 8.0, sx_, sy_)])
    return union([pico, ky, stick])


def hand():
    """A simple right hand holding it: palm on the right side of the thin back, fingers under, thumb on the stick cap."""
    from lib import M
    cap_top = layout()[0] + KY['cap'] + 8.0
    sph = lambda p, r: M.sphere(r, 24).translate(p)
    cap = lambda a, b, r: M.batch_hull([sph(a, r), sph(b, r)])
    palm = M.batch_hull([sph((2.0, -32.0, 12.0), 12.0), sph((44.0, -32.0, 12.0), 12.0), sph((2.0, -30.0, -6.0), 11.0),
                         sph((44.0, -30.0, -6.0), 11.0), sph((-24.0, -26.0, 6.0), 14.0)])
    fingers = []
    for k, x in enumerate((10.0, 25.0, 40.0, 54.0)):
        r = 7.5 - 0.6 * (k == 3)
        fingers += [cap((x, -26.0, -9.0), (x, 16.0, -9.0), r), cap((x, 16.0, -9.0), (x, 27.0, 5.0), r)]
    thumb = union([cap((36.0, -31.0, 24.0), (58.0, -14.0, cap_top + 1.0), 8.5),
                   cap((58.0, -14.0, cap_top + 1.0), (KY['x'] - 2.0, 0.0, cap_top + 7.0), 8.0)])
    return union([palm, thumb] + fingers)


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
    print(f'top shell inside at the stick {lid_z:.1f} mm (dome top {ky_top + LS.KY["dome"]:.1f}); KY board top {ky_top:.1f}; slot {2 * LS.SLOT_HALF:.0f} mm front-back x '
          f'{LS.KY["shaft"] + 0.8:.1f} mm')
    for r in LS.export(out): print('  ', r['file'], r['size_mm'], r['volume_cm3'], 'cm3')


def render(path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

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
    tray, top = PARTS['stick_tray'], PARTS['stick_top']
    fig = plt.figure(figsize=(22, 13), facecolor='white')
    draw(fig.add_subplot(2, 3, 1, projection='3d'), [(tray['solid'], tray['color']), (top['solid'], top['color']), b,
         (hand(), (0.93, 0.76, 0.62))], 25, -125, zoom=1.0, title='in the hand: palm on the thin back, thumb on the stick')
    draw(fig.add_subplot(2, 3, 2, projection='3d'), [(tray['solid'], tray['color']), (top['solid'].translate((0, 0, 28)), top['color']), b],
         30, -60, zoom=1.0, title='top shell lifted: Pico at the back (USB out the end), KY-023 at the front')
    draw(fig.add_subplot(2, 3, 3, projection='3d'), [(tray['solid'], tray['color']), (top['solid'], top['color']), b], 0, -90, zoom=1.1,
         cut=box(-50, 150, 0, 60, -10, 80), title='cut along the middle: thin at the back, full height over the stick')
    # beside the mono leader: box centre 40 cm to the operator's left of the pedestal axis (+Y), desk at z = -H_ARM
    import leader as LD
    if not LD.PARTS: LD.build()
    place = T(0, BOX_R, -LD.H_ARM) @ Rz(90) @ T(-(SH['x0'] + SH['x1']) / 2, 0, 0)
    stick = [(xf(tray['solid'], place), tray['color']), (xf(top['solid'], place), top['color']), (xf(boards(), place), (0.10, 0.35, 0.20))]
    desk = (box(-450, 450, -350, 550, -LD.H_ARM - 4.0, -LD.H_ARM), (0.93, 0.93, 0.90))
    zone = (cyl(LOW_R, -LD.H_ARM, -LD.H_ARM + 0.6) - cyl(LOW_R - 4.0, -LD.H_ARM - 1, -LD.H_ARM + 1), (0.85, 0.20, 0.20))
    for k, (q, el, az, title) in enumerate([((0, 0, 0, 0, 0), 30, -140, f'joystick {BOX_R / 10:.0f} cm from the pedestal (red circle: leader can come low inside {LOW_R / 10:.0f} cm)'),
                                            ((90, -40, 40, 90, 0), 20, -20, 'reaching toward it, hand down: stays inside the circle'),
                                            ((90, 0, 0, 60, 0), 0, 0, 'side view: stretched toward it, hand tilted down 60 deg')]):
        its = [(s_, c_) for nm, s_, c_, bd in LD.world(q, 0.0, variant='mono')] + stick + [zone]   # no desk plane: it hides the box
        draw(fig.add_subplot(2, 3, 4 + k, projection='3d'), its, el, az, zoom=1.05, title=title)
    plt.tight_layout(); plt.savefig(path, dpi=55); plt.close(fig)
