"""Scene camera (user 2026-11-13): a Luxonis OAK-D Pro (AF) on a pitch servo (Feetech STS3215 7.4 V), riding on the
CARRIAGE (it moves up and down with the arms, as the old scene camera did), in front of the column. Built from
SWAPPABLE printed parts joined by the same flange, so a different height (or later a forward offset) is one
replacement part.

Units mm. CARRIAGE frame: origin on the column axis at the carriage origin, x forward, z up (= the frame of
dual_holder.py and mono_ring.py).

    mount         not a part of its own: two-arm robot = a 2 mm pocket across the dual holder's front bridge (4 x M3 x 8
                  inserts in its floor, z 23.5); one-arm robot = a pedestal on the mono ring's clamp (inserts at z 90,
                  over the turning cable tray and rotor ribs)
    mast_<L>      36 (x) x 26 (y) box, 3 mm walls, hollow for the USB + servo cables (window at the top front, exit at the
                  bottom), a flange at each end; prints lying on its front face. L = flange to flange: 240 (two-arm) /
                  173 (one-arm) put the lens 0.33 m over the carriage origin
    head          flange + crossbar + the STS3215 cradle (output face toward the camera, body hanging down) on the -y
                  side + the pivot upright on the +y side
    cradle        bolted to the camera's two back M4 holes (VESA 75) on 3 mm standoffs (cooling fins stay open); one
                  arm on the servo horn (4 x M3), the other on an M5 pivot bolt; the bottom (USB-C plug) left open
FLANGE: 40 (x) x 44 (y) x 6, 4 x M3 from above into M3 x 8 inserts in the part below, on (MX +-14, +-17); x 22..62:
clear of the beam and of the lift's top end mount (both x <= 20).
Camera (from Luxonis' STEP, generator/oakd/): 97 x 29.7 x 23.4 with the bezel, lenses 4.8 mm above the body centre,
back M4 pair 75 mm apart, USB-C in the bottom 13.5 mm right of centre, tripod 1/4-20 in the bottom near the left end.

    python3 scene_cam.py [out_dir]     checks + export (default ../../CAD/scene_cam)
"""
import math, os, sys, csv, collections
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import M, CS, box, cyl, ring, union, xf, T, Rx, Ry, Rz, basis, along, print_mesh
from hw import screw_cut, insert_cut, nut_cut, Reg
from joint import SERVO, HORN_HOLES, OUT_FACE, RIM, CTR_D, OUT_CTR, SO100, case_screw

MX = 42.0                                     # mast axis x: the flange (x 22..62) stays clear of the beam and the top end mount
FL = dict(wx=40.0, wy=44.0, t=6.0, bx=14.0, by=17.0)   # flange; bolts on (MX +- bx, +-by)
# mast box (2026-11-16, stiffer: was 24 x 24 x 2.5 centred on MX, printed standing): x 26..62 (6 mm in front of the
# beam / end mounts at x <= 20; front face flush with the flanges, so it prints LYING on its front face: no supports,
# and bending then runs along the layers), y +-13 (the flange screw heads at y +-17 and their counterbores stay
# outside), wall 3 (with 4 perimeters per surface the wall prints solid). Cavity peaked at the back (45 deg) so the
# lying print has no long bridge.
BOX = dict(x0=26.0, x1=62.0, wy=26.0, wall=3.0, chamfer=14.0)
FL0 = dict(t=6.0, cb=3.2, screw=10, note='mast to its mount (from above, beside the box; ball-end key)')   # bottom flange: on the dual robot it sits in the holder's 2 mm pocket
CABLE_WIN = dict(wy=16.0, h=14.0)             # cable window in the front face under the top chamfer (USB-C plug passes)
MOUNT = {'prototype': dict(z=23.5, L=240.0), 'prototype_mono_arm': dict(z=90.0, L=173.0)}   # mount face (holder bridge pocket floor dual_holder.CAM_Z / clamp pedestal), mast length
AX = MX + 23.0                                # tilt axis x: 23 mm in front of the mast (the USB-C plug clears the beam at 80 deg)
HEAD_AXIS = 62.0                              # head flange face -> tilt axis (the USB-C plug under the camera clears the crossbar)
LENS_DZ = 330.0                               # lens over the carriage origin (study: 0.30-0.35 best, scene_cam README)
# camera (camera frame C: origin at the body centre, x forward along the lenses, y left, z up)
CAM = dict(x=(-11.68, 11.68), y=(-48.55, 48.55), z=(-14.86, 14.86), lens=(11.7, 0.0, 4.8), vesa=[(-37.5, -0.8), (37.5, -0.8)],
           usb=dict(y=13.5, x=4.7), axis_x=-3.0)
STANDOFF, BACK_T = 3.0, 4.0
ARM_T, ARM_GAP = 6.0, 1.0
ARM_TP = 8.0                                  # the pivot-side arm (room for the M5 x 6 insert)
PITCH = (-10.0, 80.0)                         # deg, + = tip down (as the arm's wrist pitch)
# the servo: output face toward +y (the camera), body hanging down (servo -x = down)
HORN_Y = -(CAM['y'][1] + ARM_GAP + ARM_T)     # horn face y = the horn-side arm's outer face (-55.55)
SRV_Y = HORN_Y - OUT_FACE                     # servo origin y
PIVOT_Y = CAM['y'][1] + ARM_GAP               # the idler-side arm's inner face (+49.55)

PARTS, CONTEXT = {}, {}


def add(name, solid, color, reg, note, print_rot=np.eye(4), material='PETG'):
    PARTS[name] = dict(solid=solid, color=color, reg=reg, note=note, print_rot=print_rot, material=material)


def flange(z0, cb=0.0):
    """Flange at z0 .. z0 + t around the mast axis with the 4 clearance holes (M3 from above), heads sunk cb deep."""
    f = box(MX - FL['wx'] / 2, MX + FL['wx'] / 2, -FL['wy'] / 2, FL['wy'] / 2, z0, z0 + FL['t'])
    cut = [screw_cut((MX + sx * FL['bx'], sy * FL['by'], z0 + FL['t'] - cb), (0, 0, -1), 'M3', FL['t'] - cb + 0.5, cb=cb + 30.0 if cb else 0.0)
           for sx in (-1, 1) for sy in (-1, 1)]
    return f, cut


def flange0():
    """The mast's bottom flange (mast frame z 0 = its underside), screw heads sunk FL0['cb']."""
    return flange(0.0, cb=FL0['cb']) if FL0['t'] == FL['t'] else _flange_t(FL0['t'], FL0['cb'])


def _flange_t(t, cb):
    f = box(MX - FL['wx'] / 2, MX + FL['wx'] / 2, -FL['wy'] / 2, FL['wy'] / 2, 0.0, t)
    cut = [screw_cut((MX + sx * FL['bx'], sy * FL['by'], t - cb), (0, 0, -1), 'M3', t - cb + 0.5, cb=cb + 30.0)
           for sx in (-1, 1) for sy in (-1, 1)]
    return f, cut


def flange_inserts(z_top, reg, note):
    """The 4 M3 x 8 inserts in the top of the part below a flange."""
    cut = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = (MX + sx * FL['bx'], sy * FL['by'], z_top)
            cut.append(insert_cut(p, (0, 0, -1), 'M3', length=8)); reg.add('insert', 'M3', p, (0, 0, -1), 8, note)
    return cut


def flange_screws(z_top, reg, note, length=14):
    for sx in (-1, 1):
        for sy in (-1, 1):
            reg.add('SHCS', 'M3', (MX + sx * FL['bx'], sy * FL['by'], z_top), (0, 0, -1), length, note)


def head_frame(z_head):
    """Tilt-axis frame at the camera's zero pitch: origin on the axis, x forward, y left, z up."""
    return T(AX, 0.0, z_head + HEAD_AXIS)


def servo_frame(z_head):
    """STS3215: servo z = -y (output face toward +y), servo x = +z (body hangs down along servo -x), origin on the axis."""
    return T(AX, SRV_Y, z_head + HEAD_AXIS) @ basis((0, 0, 1), (-1, 0, 0), (0, -1, 0))


# ------------------------------------------------------------------ parts
def mast(L):
    """Mast in its own frame (lower flange face at z 0)."""
    r = Reg()
    x0, x1, hy, w = BOX['x0'], BOX['x1'], BOX['wy'] / 2, BOX['wall']
    b = box(x0, x1, -hy, hy, 0, L)
    yi = hy - w                                                    # cavity half width
    inner = M.hull(union([box(x0 + w + yi, x1 - w, -yi, yi, -1, L + 1), box(x0 + w, x0 + w + 0.2, -0.1, 0.1, -1, L + 1)]))
    f0, c0 = flange0()                                             # dual: sits in the holder's 2 mm pocket (screw heads lower)
    f1, _ = flange(L - FL['t'])
    zc = L - FL['t'] - BOX['chamfer']
    ch = M.hull(union([box(x0, x1, -hy, hy, zc, L - FL['t']),
                       box(MX - FL['wx'] / 2, MX + FL['wx'] / 2, -FL['wy'] / 2, FL['wy'] / 2, L - FL['t'] - 0.01, L - FL['t'])]))
    win = box(x1 - w - 1, x1 + 1, -CABLE_WIN['wy'] / 2, CABLE_WIN['wy'] / 2, zc - 2.0 - CABLE_WIN['h'], zc - 2.0)   # cables in (top, front)
    exit_ = box(MX - 8.0, MX + 8.0, -hy - 1, -hy + w + 1, FL0['t'] + 4.0, FL0['t'] + 18.0)       # cables out at the bottom (-y side)
    s = union([b, f0, f1, ch]) - union([inner, win, exit_] + c0 + flange_inserts(L, r, 'mast top (head)'))
    flange_screws(FL0['t'] - FL0['cb'], r, FL0['note'], length=FL0['screw'])
    add(f'mast_{int(L)}', s, (0.20, 0.55, 0.62), r,
        f'{int(L)} mm flange to flange, {x1 - x0:.0f} x {2 * hy:.0f} box (wall {w:.0f}), hollow for the cables (window at the top front, exit at the bottom); prints LYING on its front face (no supports)',
        print_rot=Ry(90))
    return s


def head(zh):
    """zh: the head's flange face (column-top frame)."""
    r = Reg()
    f, fcut = flange(zh)
    fcut += [cyl(3.3, zh + FL['t'] - 0.01, zh + FL['t'] + 11.0, MX + sx * FL['bx'], sy * FL['by'], hole=True) for sx in (-1, 1) for sy in (-1, 1)]   # key access through the crossbar
    flange_screws(zh + FL['t'], r, 'head to the mast (into its inserts)')
    za = zh + HEAD_AXIS
    Fs = servo_frame(zh)
    bar = box(MX - 14.0, AX + 14.8, SRV_Y - RIM - 4.0, PIVOT_Y + ARM_TP + 9.0, zh + FL['t'] - 0.01, zh + FL['t'] + 10.0)
    # servo cradle: walls round the case (servo y +-12.4 -> world x; servo x -35.2 .. 10.2 -> world z), open toward +y
    # (the horn) with lips over the output-face rim corners, a floor under its rear end, a strap on the idler face
    W = 2.4
    yi, yo = SRV_Y - RIM, SRV_Y + RIM                          # idler face (outer), output face rim (inner)
    walls = box(AX - 12.4 - W, AX + 12.4 + W, yi, yo + 2.0, min(za - 35.2 - W, zh + FL['t'] + 9.99), za + 10.2 + W)
    pocket = box(AX - 12.45, AX + 12.45, yi - 1, yo + 0.01, za - 35.25, za + 10.25)
    horn_win = box(AX - 10.4, AX + 10.4, yo - 0.01, yo + 3.0, za - 35.3, za + 10.3)        # lips stay at |x| > 10.4
    up = box(AX - 10.0, AX + 10.0, PIVOT_Y + ARM_TP + 1.0, PIVOT_Y + ARM_TP + 9.0, zh + FL['t'], za + 14.0)  # pivot upright
    s = union([f, bar, walls, up, xf(along(cyl(14.0, PIVOT_Y + ARM_TP + 1.0, PIVOT_Y + ARM_TP + 9.0), 'y'), T(AX, 0, za))])
    cut = fcut + [pocket, horn_win, xf(along(cyl(2.65, PIVOT_Y, PIVOT_Y + ARM_TP + 12.0, hole=True), 'y'), T(AX, 0, za)),     # M5 pivot (5.3)
                  box(AX - 6.0, AX + 6.0, yi - 1, yi + 6.0, za - 30.0, za - 10.0)]          # servo cable out of the strap side
    # strap across the idler face at the case screws (SO-100 idler pattern (-29, +-10.25)), M2 into the walls
    sr = Reg(); zs = za - 29.0
    strap = box(AX - 21.0, AX + 21.0, yi - 3.0, yi, zs - 5.0, zs + 5.0)
    sc = []
    for (xc, yc) in SO100['idler']:
        pc = Fs @ np.array([xc, yc, RIM + 2.0, 1.0])
        sc.append(case_screw(tuple(pc[:3]), (0, 1, 0), sr, 'servo case (idler face)', cb=1.0))
    for sx in (-1, 1):
        p = (AX + sx * 17.6, yi - 3.0, zs)
        sc.append(screw_cut(p, (0, 1, 0), 'M2', 3.0 + 3.0, cb=0.0)); sr.add('SHCS', 'M2', p, (0, 1, 0), 6, 'strap to cradle')
        cut.append(insert_cut((AX + sx * 17.6, yi, zs), (0, 1, 0), 'M2', length=4)); r.add('insert', 'M2', (AX + sx * 17.6, yi, zs), (0, 1, 0), 4, 'strap')
    r.add('SHCS', 'M5', (AX, PIVOT_Y + ARM_TP + 9.0, za), (0, -1, 0), 16, 'pivot bolt (into the cradle arm insert; turns in the upright: washer)')
    add('head', s - union(cut), (0.55, 0.57, 0.60), r, 'flange + crossbar + STS3215 cradle (output toward the camera) + pivot upright; prints flange down')
    add('servo_strap', strap - union(sc), (0.55, 0.57, 0.60), sr, 'holds the pitch servo in the head; prints flat')
    CONTEXT['servo'] = xf(SERVO, Fs)


def cradle(zh):
    """Camera frame C (origin body centre, at zero pitch: x forward, y left, z up); the axis sits at C (axis_x, 0, 0)."""
    r = Reg(); C = CAM
    bx0 = C['x'][0] - STANDOFF - BACK_T
    back = box(bx0, C['x'][0] - STANDOFF, C['y'][0] - ARM_GAP - ARM_T, C['y'][1] + ARM_GAP + ARM_TP, C['z'][0], C['z'][1])
    offs = [xf(along(cyl(4.5, C['x'][0] - STANDOFF - 0.01, C['x'][0]), 'x'), T(0, y, z)) for (y, z) in C['vesa']]
    arms = []
    for sy in (-1, 1):
        y0, y1 = (C['y'][1] + ARM_GAP, C['y'][1] + ARM_GAP + ARM_TP) if sy > 0 else (C['y'][0] - ARM_GAP - ARM_T, C['y'][0] - ARM_GAP)
        plate = box(bx0, C['axis_x'], y0, y1, C['z'][0], C['z'][1])
        hub = xf(along(cyl(13.0, y0, y1), 'y'), T(C['axis_x'], 0, 0))
        arms += [plate, hub]
    s = union([back] + offs + arms)
    cut = []
    for (y, z) in C['vesa']:                  # M4 into the camera's back holes
        p = (bx0, y, z)
        cut.append(screw_cut(p, (1, 0, 0), 'M4', BACK_T + STANDOFF, cb=0.0)); r.add('SHCS', 'M4', p, (1, 0, 0), 12, 'camera back M4 (VESA 75); about 5 mm into the camera')
    # horn side (-y): 4 x M3 into the output horn + centre-screw recess; idler side (+y): M5 insert for the pivot bolt
    ya = C['y'][0] - ARM_GAP - ARM_T                                  # the arm's outer face = the horn face
    for (a, b) in HORN_HOLES:                                         # horn plane: servo (x, y) -> C (z = x_s, x = -y_s)
        p = (C['axis_x'] - b, ya + ARM_T + 0.01, a)
        cut.append(screw_cut(p, (0, -1, 0), 'M3', ARM_T, cb=0.0)); r.add('SHCS', 'M3', p, (0, -1, 0), 8, 'cradle to the servo horn')
    cut.append(xf(along(cyl(CTR_D / 2, ya - 0.1, ya + (OUT_CTR - OUT_FACE)), 'y'), T(C['axis_x'], 0, 0)))
    cut.append(insert_cut((C['axis_x'], C['y'][1] + ARM_GAP + ARM_TP, 0.0), (0, -1, 0), 'M5', length=6)); r.add('insert', 'M5', (C['axis_x'], C['y'][1], 0), (0, -1, 0), 6, 'pivot bolt')
    s = s - union(cut)
    G = head_frame(zh) @ T(-C['axis_x'], 0, 0)                        # C -> column-top frame at pitch 0
    CONTEXT['C_to_top'] = G
    add('camera_cradle', xf(s, G), (0.20, 0.55, 0.62), r, 'holds the OAK-D Pro by its 2 back M4 (standoffs: the fins stay open), horn arm + pivot arm; prints back plate down')
    CONTEXT['cradle_C'] = s


def camera_envelope():
    C = CAM
    body = box(C['x'][0], C['x'][1], C['y'][0], C['y'][1], C['z'][0], C['z'][1])
    plug = box(C['usb']['x'] - 3.5, C['usb']['x'] + 3.5, C['usb']['y'] - 6.5, C['usb']['y'] + 6.5, C['z'][0] - 28.0, C['z'][0] + 0.01)   # USB-C plug + strain relief
    return body, plug


def cbeam_top(length=150.0):
    """The real C-Beam top for previews (generator/cbeam: the kit's top end mount + the beam's last `length` mm), as
    (triangles, colour) in the column-top frame (the lift frame shifted down by the beam + end mount: z 0 = mount top)."""
    from stlio import read_stl
    from manifold3d import Manifold, Mesh
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cbeam')
    out = []
    for nm, col, clip in (('top', (0.78, 0.80, 0.83), None), ('beam', (0.30, 0.33, 0.37), length), ('column_hw', (0.70, 0.72, 0.75), length)):
        T3 = read_stl(os.path.join(d, nm + '.stl')).astype(float) - np.array([0, 0, 1012.0])
        if clip is not None:
            V, inv = np.unique(np.round(T3.reshape(-1, 3), 4), axis=0, return_inverse=True)
            try:
                m = Manifold(Mesh(vert_properties=V.astype(np.float32), tri_verts=inv.reshape(-1, 3).astype(np.uint32)))
                m = m ^ box(-60, 60, -60, 60, -12.0 - clip, -11.99)
                mm = m.to_mesh(); T3 = np.asarray(mm.vert_properties)[:, :3][np.asarray(mm.tri_verts)]
            except Exception:
                T3 = T3[(T3[..., 2] > -12.0 - clip).all(axis=1)]
        out.append((T3, col))
    return out


def camera_mesh_C():
    """The Luxonis enclosure mesh in the camera frame C (mm): enclosure (xe, ye, ze) -> (ze + 1.48, xe - 44.5, ye + 11.8)."""
    from stlio import read_stl
    E = read_stl(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'oakd', 'oakd_pro_enclosure.stl')).astype(float)
    return np.stack([E[..., 2] + 1.48, E[..., 0] - 44.5, E[..., 1] + 11.8], axis=-1)


def build_all(layout='prototype'):
    PARTS.clear(); CONTEXT.clear()
    m = MOUNT[layout]; L = m['L']
    mast_s = mast(L)
    PARTS[f'mast_{int(L)}']['solid'] = xf(mast_s, T(0, 0, m['z']))
    zh = m['z'] + L
    head(zh); cradle(zh)
    CONTEXT['zh'] = zh; CONTEXT['layout'] = layout
    body, plug = camera_envelope()
    CONTEXT['cam_body_C'], CONTEXT['cam_plug_C'] = body, plug


def pitched(sol_C, deg, zh):
    """A solid given in camera frame C, at pitch `deg` (+ = down), in the column-top frame."""
    return xf(sol_C, head_frame(zh) @ Ry(deg) @ T(-CAM['axis_x'], 0, 0))


def mount_context(layout):
    """The robot side under the mast, carriage frame: (fixed solids dict, turning solids for the mono ring or None)."""
    if layout == 'prototype':
        import dual_holder as DH
        DH.build_all()
        fixed = {'holder_left': DH.PARTS['holder_left']['solid'], 'holder_right': DH.PARTS['holder_right']['solid'], 'arm_bases': DH.CONTEXT['bases']}
        return fixed, None
    import mono_ring as MR
    MR.build_all()
    fixed = {k: p['solid'] for k, p in MR.PARTS.items() if not p['moving']}
    turning = union([p['solid'] for p in MR.PARTS.values() if p['moving']])
    return fixed, turning


def lift_kit():
    """The kit's carriage (plate, wheels, nut block) and a 1 m beam window, carriage frame (lift 0.3 for the beam)."""
    from stlio import read_stl
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cbeam')
    car = read_stl(os.path.join(d, 'carriage.stl')).astype(float) + np.array([0, 0, 20.0 - 62.5])
    beam = read_stl(os.path.join(d, 'beam.stl')).astype(float) + np.array([0, 0, 313.0 - 420.0 - 300.0])
    hw = read_stl(os.path.join(d, 'column_hw.stl')).astype(float) + np.array([0, 0, 313.0 - 420.0 - 300.0])
    from manifold3d import Manifold, Mesh

    def window(t, z0, z1):                        # cut the long beam faces to a window (as a solid, part by part)
        V, inv = np.unique(np.round(t.reshape(-1, 3), 4), axis=0, return_inverse=True)
        m = Manifold(Mesh(vert_properties=V.astype(np.float32), tri_verts=inv.reshape(-1, 3).astype(np.uint32)))
        if m.is_empty() or m.volume() <= 0:
            return t[((t[..., 2] > z0) & (t[..., 2] < z1)).all(axis=1)]
        mm = (m ^ box(-60, 60, -60, 60, z0, z1)).to_mesh()
        return np.asarray(mm.vert_properties)[:, :3][np.asarray(mm.tri_verts)]
    from render_robot import subdivide
    return [(car, (0.30, 0.33, 0.37)), (subdivide(window(beam, -160, 470), 12.0), (0.24, 0.26, 0.30)), (subdivide(window(hw, -160, 470), 12.0), (0.70, 0.72, 0.75))]


def check(layout):
    res, info = [], []
    zh = CONTEXT['zh']
    fixed = {k: p['solid'] for k, p in PARTS.items() if k not in ('camera_cradle',)}
    fixed['servo'] = CONTEXT['servo']
    names = list(fixed)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if {a, b} in ({'head', 'servo'}, {'servo_strap', 'servo'}, {'head', 'servo_strap'}): continue
            v = (fixed[a] ^ fixed[b]).volume()
            if v > 0.5: res.append(f'{a} x {b}: {v:.1f} mm3')
    turning = union([CONTEXT['cradle_C'], CONTEXT['cam_body_C'], CONTEXT['cam_plug_C']])
    sweep = [pitched(turning, d, zh) for d in range(int(PITCH[0]), int(PITCH[1]) + 1, 10)]
    ours = union(list(fixed.values()) + sweep)
    # beam + both end mounts (x <= 20, |y| <= 40) at any lift, kit carriage plate / wheels
    for nm, sol in (('beam / end mounts (any lift)', box(-30.0, 20.0, -40.0, 40.0, -600.0, 1200.0)),
                    ('gantry plate', box(-27.5, -21.5, -62.5, 62.5, -105.0, 20.0)),
                    ('V-wheels', union([box(-16.0, 0.5, 38.0, 62.0, -105.0, 20.0), box(-16.0, 0.5, -62.0, -38.0, -105.0, 20.0)]))):
        v = (ours ^ sol).volume()
        if v > 0.1: res.append(f'camera mount x {nm}: {v:.1f} mm3')
    # the mount under it (holder + arm bases / mono ring: fixed parts, and the rotor + tray all the way round)
    env, rot = mount_context(layout)
    for nm, sol in env.items():
        v = (ours ^ sol).volume()
        if v > 0.1: res.append(f'camera mount x {nm}: {v:.1f} mm3')
    if rot is not None:
        worst = max(((ours ^ xf(rot, Rz(a))).volume(), a) for a in range(0, 360, 10))
        if worst[0] > 0.1: res.append(f'camera mount x mono rotor / tray at {worst[1]} deg: {worst[0]:.1f} mm3')
        info.append(f'mono rotor + cable tray all the way round: closest {worst[0]:.2f} mm3')
    # the mast's bottom flange sits on the mount's insert face
    mast = fixed[[k for k in names if k.startswith('mast')][0]]
    z0 = MOUNT[layout]['z']
    seat = (union(list(env.values())) ^ box(MX - 20, MX + 20, -22, 22, z0 - 0.5, z0)).volume()
    info.append(f'{layout}: mount face at z {z0}: {seat / 1000:.2f} cm3 of mount in the 0.5 mm under the flange (seat area ~{seat / 0.5:.0f} mm2 of 1760)')
    # pitch sweep: cradle + camera + USB plug against the head, the strap, the servo and the mast
    still = union([fixed['head'], fixed['servo_strap'], CONTEXT['servo'] - xf(along(cyl(11.0, -100, 100), 'y'), head_frame(zh)), mast])
    free = [(d, (pitched(turning, d, zh) ^ still).volume()) for d in range(-40, 121, 5)]
    lo = min(d for d, v in free if all(vv <= 0.5 for dd, vv in free if d <= dd <= 0))
    hi = max(d for d, v in free if all(vv <= 0.5 for dd, vv in free if 0 <= dd <= d))
    info.append(f'pitch clear from {lo} to {hi} deg (camera, cradle and USB plug against the head, servo and mast); limits {PITCH}')
    if lo > PITCH[0] or hi < PITCH[1]: res.append(f'pitch range {PITCH} not clear (clear {lo}..{hi})')
    lens = head_frame(zh) @ T(-CAM['axis_x'], 0, 0) @ np.array([*CAM['lens'], 1.0])
    info.append(f'lens at pitch 0: {lens[2]:.1f} mm over the carriage origin, {lens[0]:.1f} mm in front of the column axis')
    return res, info


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
    build_all('prototype_mono_arm'); mono_mast = dict(PARTS[f"mast_{int(MOUNT['prototype_mono_arm']['L'])}"])
    build_all('prototype')
    order = [f"mast_{int(MOUNT['prototype']['L'])}", 'mono', 'head', 'servo_strap', 'camera_cradle']
    for i, nm in enumerate(order, 1):
        if nm == 'mono':
            nm = f"mast_{int(MOUNT['prototype_mono_arm']['L'])}"; p = mono_mast
        else:
            p = PARTS[nm]
        num = f'C{i:02d}_{nm}'
        _write(os.path.join(out, 'assembly_stl', num + '.stl'), p['solid'])
        s = p['solid']
        if nm == 'camera_cradle': s = xf(CONTEXT['cradle_C'], Ry(90))             # back plate down
        if nm == 'servo_strap': s = xf(s, Rx(90))
        if nm.startswith('mast'): s = xf(s, p['print_rot'])                        # lying on its front face
        bb = s.bounding_box(); s = s.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2])); bb = s.bounding_box()
        _write(os.path.join(out, 'print_ready_stl', num + '.stl'), s)
        rows.append(dict(file=num + '.stl', part=nm, material=p['material'], size_mm=f'{bb[3]-bb[0]:.1f} x {bb[4]-bb[1]:.1f} x {bb[5]-bb[2]:.1f}',
                         volume_cm3=round(p['solid'].volume() / 1000, 1), note=p['note']))
        if nm.startswith('mast') and p is mono_mast: continue                      # one robot's set: count the dual mast only
        for it in p['reg'].items:
            fast[(it['kind'], it['size'], it['length'], it['note'])] += 1
    fast[('servo', 'Feetech STS3215 7.4 V', '', 'camera pitch')] = 1
    fast[('camera', 'Luxonis OAK-D Pro AF', '', 'scene camera (+ its USB-C Y-adapter for the IR projector / LED)')] = 1
    _write(os.path.join(out, 'assembly_stl', 'context', 'pitch_servo_STS3215.stl'), CONTEXT['servo'])
    _write(os.path.join(out, 'assembly_stl', 'context', 'oakd_pro_envelope.stl'), pitched(union([CONTEXT['cam_body_C'], CONTEXT['cam_plug_C']]), 0, CONTEXT['zh']))
    with open(os.path.join(out, 'printed_parts.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    with open(os.path.join(out, 'fasteners.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['kind', 'size', 'length_mm', 'qty', 'where'])
        for (k, s, L, n), q in sorted(fast.items(), key=lambda t: (str(t[0][1]), t[0][0], str(t[0][2]))):
            w.writerow([k, s, L, q, n])
    render(os.path.join(out, 'preview_scene_cam.png'))
    print(f'exported {len(rows)} parts to {out}')


def render(path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    BG = (0.906, 0.922, 0.941)
    camE = camera_mesh_C()

    def tris_of(s):
        m = s.to_mesh(); return np.asarray(m.vert_properties)[:, :3][np.asarray(m.tri_verts)]

    def draw(ax, items, el, az, title, lim):
        r_ = lim[1]
        from render_robot import subdivide
        T_, C_ = [], []
        for t, c in items:
            t = subdivide(t, 12.0 if r_ > 120 else 6.0)
            T_.append(t); C_.append(np.tile(np.r_[c[:3]], (len(t), 1)))
        T_ = np.vstack(T_); C_ = np.vstack(C_)
        N = np.cross(T_[:, 1] - T_[:, 0], T_[:, 2] - T_[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
        Ld = np.array([math.cos(math.radians(el + 25)) * math.cos(math.radians(az + 30)), math.cos(math.radians(el + 25)) * math.sin(math.radians(az + 30)), math.sin(math.radians(el + 25))])
        sh = 0.35 + 0.65 * np.abs(N @ Ld)
        ax.add_collection3d(Poly3DCollection(T_, facecolors=np.c_[C_ * sh[:, None], np.ones(len(sh))], edgecolor='none'))
        c, r = lim
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off(); ax.set_title(title, fontsize=13); ax.set_facecolor(BG)

    def scene(layout, deg):
        build_all(layout); zh = CONTEXT['zh']
        G = head_frame(zh) @ Ry(deg) @ T(-CAM['axis_x'], 0, 0)
        P = camE.reshape(-1, 3); P = (G[:3, :3] @ P.T).T + G[:3, 3]
        items = [(tris_of(p['solid']), p['color']) for k, p in PARTS.items() if k != 'camera_cradle']
        items += [(tris_of(CONTEXT['servo']), (0.12, 0.12, 0.14)), (P.reshape(-1, 3, 3), (0.15, 0.15, 0.17)),
                  (tris_of(pitched(CONTEXT['cradle_C'], deg, zh)), PARTS['camera_cradle']['color'])]
        return items, zh

    kit = lift_kit()
    fig = plt.figure(figsize=(21, 9), facecolor=BG)
    # two-arm: dual holder + both arm bases
    import dual_holder as DH
    DH.build_all()
    dual_mount = [(tris_of(DH.PARTS['holder_left']['solid']), DH.PARTS['holder_left']['color']),
                  (tris_of(DH.PARTS['holder_right']['solid']), DH.PARTS['holder_right']['color']),
                  (tris_of(DH.CONTEXT['bases']), (0.93, 0.55, 0.20))]
    cam, zh = scene('prototype', 20)
    draw(fig.add_subplot(1, 3, 1, projection='3d'), kit + dual_mount + cam, 16, -55,
         f"two-arm robot: mast {int(MOUNT['prototype']['L'])} on the holder's front bridge", (np.array([20.0, 0.0, 150.0]), 175.0))
    # one-arm: mono ring (fixed + rotor + tray)
    import mono_ring as MR
    MR.build_all()
    mono_mount = [(tris_of(p['solid']), p['color']) for p in MR.PARTS.values()]
    cam, zh = scene('prototype_mono_arm', 20)
    draw(fig.add_subplot(1, 3, 2, projection='3d'), kit + mono_mount + cam, 16, -55,
         f"one-arm robot: mast {int(MOUNT['prototype_mono_arm']['L'])} on the ring clamp's pedestal", (np.array([20.0, 0.0, 150.0]), 175.0))
    # head close-up, tilted down
    cam, zh = scene('prototype', 55)
    draw(fig.add_subplot(1, 3, 3, projection='3d'), cam, 20, -40, 'head: STS3215 pitch, camera 55 deg down',
         (np.array([MX, -15.0, zh + HEAD_AXIS - 10.0]), 90.0))
    plt.subplots_adjust(left=0, right=1, bottom=0, top=0.93, wspace=-0.05)
    plt.savefig(path, dpi=90, facecolor=BG); plt.close(fig)


if __name__ == '__main__':
    import scene_cam as SC
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = args[0] if args else os.path.join('..', '..', 'CAD', 'scene_cam')
    bad = []
    for lay in ('prototype', 'prototype_mono_arm'):
        SC.build_all(lay)
        res, info = SC.check(lay)
        for line in info: print(line)
        bad += res
    print('problems:', bad if bad else 'none')
    if '--no_export' not in sys.argv: SC.export(out)
