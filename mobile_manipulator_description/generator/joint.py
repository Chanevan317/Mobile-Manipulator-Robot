"""Clevis joint around a Feetech STS3215 (the SO-100's servo), modelled in the servo ("joint") frame:
   Z = output shaft (horn side +Z), servo body toward -X, case width along Y.
Measured servo (SO-ARM100 CAD, STS3215_03a):
   case X -35.2..10.2, Y +-12.4, faces Z +-14.4, rims to +-15.9, horn disc dia 20 face Z=+17.7 (4x M3 tapped on a 9.9 square),
   idler disc face Z=-18.7, idler nub to -20.2, connectors on the horn face at X=-14.05, Y=+-2.8..7.6 (housing tops at Z=19.4),
   M2 case holes on the idler face at (X,Y)=(-29.0, +-10.25) and (-8.3, +-10.25)."""
import math
import numpy as np
from lib import M, CS, box, cyl, ring, union, xf, T, Rz
from hw import screw_cut, insert_cut, Reg
from servo import servo_solid

SERVO = servo_solid()
P = dict(
    c=0.0,           # default sweep clearance around the servo (SO-100 fit: line-to-line)
    cz=0.0,          # default sweep clearance over the faces along the exit direction
    R=36.0,          # tongue collar radius
    WY=20.5,         # tongue body half-width
    seat=26.075,     # 6808 outer race seat radius (52.15 bore)
    hub=19.95,       # hub radius in the 6808 inner race (39.9)
    # Z stations, horn (top) side
    lipT=(25.5, 28.0), brgT=(28.0, 35.0), topT=35.0, prongT=(35.6, 41.6),
    # Z stations, idler (bottom) side
    brgB=(-27.0, -20.0), botB=-27.0, prongB=(-33.6, -27.6),
    prongR=38.0, prongW=30.0, baseT=8.0,
)
HORN_HOLES = [(sx * 4.95, sy * 4.95) for sx in (-1, 1) for sy in (-1, 1)]

# SO-100 servo socket (measured 2026-11-06 from the SO-ARM100 leader print files, fitted to the STS3215 model).
# The SO-100 holds the servo with M2x6 self-tapping screws through a 2.0 mm wall into the case pilot holes:
# hole dia 2.0, head counterbore dia 4.0. Arm joints use 2 + 2 screws at the rear end of the case.
# The pocket is line-to-line with the case: the horn face sits flush, the idler side has 0.1 mm play.
SO100 = dict(
    c=0.0,                 # radial clearance around the case
    cz_horn=0.0,           # play over the horn-face rim (Z = +15.9)
    cz_idler=0.1,          # play over the idler-face rim (Z = -15.9)
    wall=2.0,              # plastic under the screw head
    hole=2.0, cb=4.0,      # M2 through-hole and head counterbore diameters
    horn=[(-32.8, 10.25), (-32.8, -10.25)],     # horn-face case holes used
    idler=[(-29.0, 10.25), (-29.0, -10.25)],    # idler-face case holes used
    screw_len=6,
)
RIM = 15.9                 # case rims on both faces (|Z|)
# Horn centre screw (not in the servo model): the horn's centre boss is dia 6 up to Z = 18.45. The SO-100 leaves the
# head open (dia 5.2 through its 6 mm horn plate). Here: a dia 7.2 recess with 3.5 mm over the boss top, enough for an
# M3 socket head (3.0 mm) or pan head (2.4 mm). It stays below the four horn-screw heads (seat Z = 25.6).
CTR_D, CTR_TOP = 7.2, 18.45 + 3.5
# Which face drives (user, 2026-11-06, from the real servo): the face WITH the cable sockets (+Z here) carries the
# free-spinning idler; the opposite face (-Z) is the driven output. Its horn face is at Z = -18.7 (4 x M3 on the
# 9.9 mm square, disc 2.8 mm thick) and its centre screw / nub reaches Z = -20.2. As on the SO-100, the next part is
# screwed to BOTH faces: the bottom fork drives through the output, the top fork is screwed to the idler.
OUT_FACE, OUT_NUB = 18.7, 20.2
OUT_CTR = OUT_NUB + 3.5             # centre-screw recess depth on the output side (3.5 mm past the nub)

_sweep_cache = {}


def case_screw(p, d, reg=None, note='servo case (SO-100)', cb=40.0):
    """SO-100 servo-case screw: M2x6 self-tap. p = head seat (2.0 mm of wall before the case), d = into the servo.
    Returns the cutter: dia 2.0 through the wall + dia 4.0 head counterbore `cb` mm back from the seat."""
    S = SO100
    if reg is not None: reg.add('selftap', 'M2', p, d, S['screw_len'], note)
    return screw_cut(p, d, 'M2', S['wall'], cb=cb, clear=S['hole'], cb_dia=S['cb'])


def _sarea(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def sweep(solid, exit_sign, zlo, zhi, c=None, cz=None, dz=0.1):
    """Space a solid needs inside a part if it is slid out of that part along exit_sign*Z (part frame).
    Layers between zlo and zhi; XY clearance c, clearance cz over faces that face the exit direction's opposite."""
    c = P['c'] if c is None else c; cz = P['cz'] if cz is None else cz
    n = int(round((zhi - zlo) / dz))
    zs = [zlo + (k + 0.5) * dz for k in range(n)]
    secs = []
    for z in zs:
        sl = solid.slice(z)
        if sl.area() > 0:
            outer = [p for p in sl.to_polygons() if _sarea(p) > 0]      # drop holes: nothing may stay inside a hole
            sl = CS(outer)
            secs.append(sl.offset(c, circular_segments=16).simplify(0.01))
        else:
            secs.append(None)
    shift = int(round(cz / dz))
    cums = [None] * n; cum = None
    order = range(n - 1, -1, -1) if exit_sign < 0 else range(n)
    for k in order:
        if secs[k] is not None:
            cum = secs[k] if cum is None else (cum + secs[k]).simplify(0.01)
        cums[k] = cum
    def get(k):
        j = k - shift if exit_sign < 0 else k + shift
        j = min(max(j, 0), n - 1)
        return cums[j]
    layers = []; k = 0
    while k < n:
        sec = get(k)
        if sec is None:
            k += 1; continue
        a = sec.area(); k2 = k + 1
        while k2 < n and get(k2) is not None and abs(get(k2).area() - a) < 1e-6:
            k2 += 1
        z0 = zlo + k * dz - (0.5 if k == 0 else 0.001); z1 = zlo + k2 * dz + (0.5 if k2 == n else 0.001)
        layers.append(sec.extrude(z1 - z0).translate((0, 0, z0))); k = k2
    return union(layers)


def _pocket(sign):
    key = ('p', sign)
    if key not in _sweep_cache:
        S = SO100
        _sweep_cache[key] = (sweep(SERVO, -1, 0.0, 20.6, c=S['c'], cz=S['cz_horn']) if sign > 0 else
                             sweep(SERVO, +1, -20.6, 0.0, c=S['c'], cz=S['cz_idler']))
    return _sweep_cache[key]


def tongue(sign, x0=-46.0, bolts='up', slot='rear', reg=None, extra=None, bolt_rear=True, port=False):
    """One clamshell half of the parent side of a joint (servo pocket + bearing seat).
    bolts: 'up' = M3 from the bottom half into heat-set inserts in the top half; 'down' = the reverse.
    The servo socket is the SO-ARM100 one (see SO100): same fit, hole sizes and case screws as the SO-100,
    so a servo moves between the two arms with its screws."""
    R, WY = P['R'], P['WY']
    if sign > 0:
        z0, z1 = 0.0, P['topT']
    else:
        z0, z1 = P['botB'], 0.0
    body = union([cyl(R, z0, z1), box(x0, 0, -WY, WY, z0, z1)] + ([extra] if extra is not None else []))
    cut = [_pocket(sign)]
    if sign > 0:
        # connector chamber; narrower behind the connectors to keep a wall around the rear horn-face screws
        chamber = [box(-34.5, -3.0, -7.0, 7.0, 15.6, 26.5), box(-28.0, -3.0, -10.5, 10.5, 15.6, 26.5),
                   box(-20.0, -8.0, -11.0, 11.0, 14.0, 15.7)]              # the servo model's connector cavities (no plastic plugs in them)
        cut += chamber + [                                                # connector + cable chamber over the case
                cyl(12.5, 14.2, 25.6, hole=True),                       # rotating hub boss clearance (from 14.2: no sliver in the face groove)
                cyl(21.5, 25.5, 28.01, hole=True),                      # inside the bearing lip
                # (design B, 2026-11-09: the cable goes from the chamber over the idler to the axis and up through the
                # next part's CABLE_HOLE; it only twists there, so no ring channel is needed)
                cyl(P['seat'], P['brgT'][0], z1 + 1, hole=True)]        # 6808 outer race seat
        if slot == 'rear':
            cut.append(box(x0 - 1, -34.0, -7.0, 7.0, 8.0, 26.5))       # 1.25 mm web beside the horn-face screws
            if port:   # cable in from above at the back (over-the-top route: from the previous joint's prong, no dip into the tube)
                cut.append(box(x0 - 1, x0 + 6.0, -7.0, 7.0, 20.0, z1 + 1))
        elif slot == 'side':
            cut.append(box(-29.5, -20.0, 0, R + 1, 16.0, 26.5))        # starts behind the +Y horn-face screw; open through the collar
                                                                            # (was closed 6 mm short of the outside until 2026-11-09)
    else:
        cut += [cyl(11.0, -20.01, -14.2, hole=True),                    # output horn clearance (to -14.2: no thin plastic ring in the case's face groove)
                cyl(P['seat'], z0 - 1, P['brgB'][1], hole=True)]
    # clamshell bolts
    rear = [(x0 + 4.5, s * 16.3) for s in (-1, 1)] if bolt_rear else []
    a = math.radians(28)
    front = [(31.5 * math.cos(a), s * 31.5 * math.sin(a)) for s in (-1, 1)]
    pts = rear + front
    for (x, y) in pts:
        if bolts == 'up':
            if sign < 0:
                cut.append(screw_cut((x, y, z0 + 3.0), (0, 0, 1), 'M3', -z0 - 3.0 + 0.2, cb=3.2))
                if reg is not None: reg.add('SHCS', 'M3', (x, y, z0 + 3.0), (0, 0, 1), 30, 'clamshell')
            else:
                cut.append(insert_cut((x, y, 0.0), (0, 0, 1), 'M3'))
                if reg is not None: reg.add('insert', 'M3', (x, y, 0.0), (0, 0, 1), None, 'clamshell')
        else:
            if sign > 0:
                # head recessed 6 mm (was 3): an M3 x 35 then fills the whole 5.7 mm insert (it reached only 3 mm)
                cut.append(screw_cut((x, y, z1 - 6.0), (0, 0, -1), 'M3', z1 - 6.0 + 0.2, cb=6.2))
                if reg is not None: reg.add('SHCS', 'M3', (x, y, z1 - 6.0), (0, 0, -1), 35, 'clamshell')
            else:
                cut.append(insert_cut((x, y, 0.0), (0, 0, -1), 'M3'))
                if reg is not None: reg.add('insert', 'M3', (x, y, 0.0), (0, 0, -1), None, 'clamshell')
    # SO-100 case screws: M2x6 self-tap, 2.0 mm wall, dia 2.0 hole, dia 4.0 head counterbore
    S = SO100
    if sign > 0:
        seat = RIM + S['cz_horn'] + S['wall']                           # head seat above the horn face
        for (x, y) in S['horn']:
            cut.append(screw_cut((x, y, seat), (0, 0, -1), 'M2', S['wall'], cb=z1 - seat + 0.5, clear=S['hole'], cb_dia=S['cb']))
            if reg is not None: reg.add('selftap', 'M2', (x, y, seat), (0, 0, -1), S['screw_len'], 'servo case (SO-100)')
    else:
        seat = -RIM - S['cz_idler'] - S['wall']
        for (x, y) in S['idler']:
            cut.append(screw_cut((x, y, seat), (0, 0, 1), 'M2', S['wall'], cb=seat - z0 + 0.5, clear=S['hole'], cb_dia=S['cb']))
            if reg is not None: reg.add('selftap', 'M2', (x, y, seat), (0, 0, 1), S['screw_len'], 'servo case (SO-100)')
    out = body - union(cut)
    if sign > 0:   # fixed separator bar over the free idler (2026-11-09): straight across the joint under the cable hole,
        hw, z0, z1 = IDLER_BAR   # anchored in the wall at both ends; the cable rides on it and can't reach the idler / screw
        out = union([out, box(-hw, hw, -13.5, 13.5, z0, z1)])
    return out


# Design B (user, 2026-11-09): the top half of the child is NOT screwed to the servo's free idler. The two 6808 bearings
# carry and centre the child; only the bottom half is screwed to the output horn and drives it. The top hub is a ring
# in the bearing (from z 28, nothing reaches down to the idler), so the servo cable goes up THROUGH THE AXIS
# (CABLE_HOLE, the 5264-3P plug fits) and only twists +-120 deg instead of sliding around the hub.
CABLE_HOLE = 6.0                    # radius
IDLER_BAR = (5.0, 22.3, 24.3)       # half width (along X), z0, z1: 0.35 mm over the idler's centre-screw space, 3.7 mm under the hub;
                                    # spans 27 mm along Y between the walls (prints as a bridge, no support)
# Cable guides on the top prong (like the SO-100's cable clips): printed arches the cable (and its plug) passes under,
# from the axis hole over the fork base and along the link to the next clamshell's notch. Opening 10 x 6 mm (5264-3P plug).
GUIDE_X = (26.0, -8.0)               # arch starts: from the axis, and from the fork base (F + value)
GUIDE_IN, GUIDE_WALL, GUIDE_LEN = (5.0, 6.0), (2.5, 2.0), 5.0   # inner half width / height, side wall / roof, length


def guide_arch(x, z):
    """One cable arch standing on a surface at height z, cable running along +X."""
    (hw, h), (w, r) = GUIDE_IN, GUIDE_WALL
    return (box(x, x + GUIDE_LEN, -hw - w, hw + w, z - 0.01, z + h + r)
            - box(x - 1, x + GUIDE_LEN + 1, -hw, hw, z - 0.1, z + h))


def fork(sign, F=58.0, reg=None, horn_screws=True, wide=None, cable=True, guides=True):
    """One half of the child side: hub in the 6808 inner race + prong + fork base, bar along +X."""
    W = P['prongW'] if wide is None else wide
    bt = P['baseT']
    if sign > 0:
        pz = P['prongT']
        parts = [cyl(P['hub'], 28.0, pz[0] + 0.01),
                 ring(P['hub'] - 0.5, 21.8, P['brgT'][1], pz[0] + 0.01),
                 cyl(P['prongR'], pz[0], pz[1]), box(0, F + 0.01, -W, W, pz[0], pz[1]),
                 box(F, F + bt, -W, W, 0.0, pz[1])]
        if cable and guides:
            parts += [guide_arch(GUIDE_X[0], pz[1]), guide_arch(F + GUIDE_X[1], pz[1])]
        s = union(parts)
        cut = [cyl(CABLE_HOLE, 27.9, pz[1] + 0.1, hole=True)] if cable else []
        for yy in (-22.0, 22.0):
            cut.append(insert_cut((F + bt / 2, yy, 0.0), (0, 0, 1), 'M3'))
            if reg is not None: reg.add('insert', 'M3', (F + bt / 2, yy, 0.0), (0, 0, 1), None, 'fork base')
        return s - union(cut)
    pz = P['prongB']
    # seats on the output horn (Z = -18.7) and is screwed to it: this half drives the next link
    parts = [cyl(9.8, -OUT_FACE - 0.75, -OUT_FACE), cyl(10.4, -20.4, -OUT_FACE - 0.7), cyl(P['hub'], pz[1] - 0.01, -20.3),
             ring(P['hub'] - 0.5, 21.8, pz[1] - 0.01, P['brgB'][0]),
             cyl(P['prongR'], pz[0], pz[1]), box(0, F + 0.01, -W, W, pz[0], pz[1]),
             box(F, F + bt, -W, W, pz[0], 0.0)]
    s = union(parts)
    cut = [cyl(CTR_D / 2, -OUT_CTR, -OUT_FACE + 0.1, hole=True)]    # output nub + centre-screw head
    if horn_screws:                                                    # M3 x 10 into the output horn; tip at its inner face
        for (x, y) in HORN_HOLES:
            seat = -(OUT_FACE - 2.8) - 10.0
            cut.append(screw_cut((x, y, seat), (0, 0, 1), 'M3', -OUT_FACE - seat, cb=seat - pz[0] + 0.5))
            if reg is not None: reg.add('SHCS', 'M3', (x, y, seat), (0, 0, 1), 10, 'horn (output)')
    for yy in (-22.0, 22.0):
        seat = pz[0] + 3.5
        cut.append(screw_cut((F + bt / 2, yy, seat), (0, 0, 1), 'M3', -seat + 0.2, cb=3.6))
        if reg is not None: reg.add('SHCS', 'M3', (F + bt / 2, yy, seat), (0, 0, 1), 35, 'fork base')
    return s - union(cut)


TUBE_BOLT_PITCH = 25.0      # max mm between tube bolt pairs (keeps the two halves clamped along the seam)


def tube(sign, xa, xb, reg=None, holes=(), H=22.0, W=25.0, t=3.0):
    """U-channel half of a hollow link from X=xa to xb, with bolt boss pairs from the fork end to the middle.
    The bolts clamp the two halves so they act as one closed box (seam shear is ~40 N; each clamped M3 holds far more)."""
    zr = (0, H) if sign > 0 else (-H, 0)
    if sign > 0:
        s = box(xa, xb, -W, W, 0, H) - box(xa - 1, xb + 1, -W + t, W - t, -1, H - t)
    else:
        s = box(xa, xb, -W, W, -H, 0) - box(xa - 1, xb + 1, -W + t, W - t, -H + t, 1)
    # The last pair stays at the middle of the tube: closer to the servo end, the next link's fork base swings
    # through the bosses at large joint angles. Extra pairs go toward the fork end, at most TUBE_BOLT_PITCH apart.
    xm = (xa + xb) / 2
    x1 = xa + 9.0                                     # first boss just past the fork-base fillet
    xs = []        # no tube bolts since 2026-11-06 (user): the bolts at both ends of the link clamp the halves
    yb = W + 3.5
    bosses = []
    for xm in xs:
        bosses += [cyl(4.5, *zr, x=xm, y=s_ * yb) for s_ in (-1, 1)]
        bosses += [box(xm - 4.5, xm + 4.5, W - 0.5, yb, *zr), box(xm - 4.5, xm + 4.5, -yb, -(W - 0.5), *zr)]
    s = union([s] + bosses)
    cut = []
    for xm in xs:
        for s_ in (-1, 1):
            if sign > 0:
                cut.append(insert_cut((xm, s_ * yb, 0.0), (0, 0, 1), 'M3'))
                if reg is not None: reg.add('insert', 'M3', (xm, s_ * yb, 0.0), (0, 0, 1), None, 'tube')
            else:
                seat = -H + 3.0
                cut.append(screw_cut((xm, s_ * yb, seat), (0, 0, 1), 'M3', -seat + 0.2, cb=3.2))
                if reg is not None: reg.add('SHCS', 'M3', (xm, s_ * yb, seat), (0, 0, 1), 25, 'tube')
    return s - union(cut) if cut else s


def _ccw(pts):
    p = np.asarray(pts, float)
    return (p if _sarea(p) > 0 else p[::-1]).tolist()


def _prism_xz(pts, y0, y1):
    """Prism from a polygon in the XZ plane, spanning Y = y0..y1."""
    # (u, v, w) -> (x = u, y = -w, z = v): a proper rotation, so the solid stays outward-facing
    s = CS([_ccw(pts)]).extrude(y1 - y0).translate((0, 0, -y1))
    return xf(s, np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]]))


def _prism_xy(pts, z0, z1):
    return CS([_ccw(pts)]).extrude(z1 - z0).translate((0, 0, z0))


def link_fillets(sign, xa, xb, H=22.0, W=25.0, r=6.0):
    """45 deg fillets in the concave corners where the tube meets the fork base (X = xa) and the servo tongue (X = xb)."""
    g = 1.0 if sign > 0 else -1.0
    zt = g * H                                        # tube top (or bottom) face
    fork_face = P['prongT'][1] if sign > 0 else P['prongB'][0]
    tongue_face = P['topT'] if sign > 0 else P['botB']
    rf = min(r, abs(fork_face) - H); rt = min(r, abs(tongue_face) - H)
    out = [_prism_xz([(xa, zt), (xa + rf, zt), (xa, zt + g * rf)], -W, W),       # on the tube, at the fork base
           _prism_xz([(xb, zt), (xb, zt + g * rt), (xb - rt, zt)], -P['WY'], P['WY'])]  # at the tongue
    zr = (0.0, H) if sign > 0 else (-H, 0.0)
    WF, WT = P['prongW'], P['WY']
    for s_ in (1, -1):                                # side corners
        tri_f = [(xa, s_ * W), (xa + (WF - W), s_ * W), (xa, s_ * WF)]   # fork base is wider than the tube
        tri_t = [(xb, s_ * WT), (xb, s_ * W), (xb + (W - WT), s_ * WT)]  # tongue is narrower than the tube
        for tri in (tri_f, tri_t):
            out.append(_prism_xy(tri, *zr))
    return union(out)


def pan_link(sign, L, F=58.0, reg=None, x0=-46.0):
    """A whole SCARA link half: fork/hub at joint A (origin), hollow tube, servo tongue for joint B at X=L."""
    rA = Reg(); rB = Reg(); rT = Reg()
    a = fork(sign, F, reg=rA)
    xa, xb = F + P['baseT'] - 0.01, L + x0 + 0.01
    t = tube(sign, xa, xb, reg=rT)
    b = xf(tongue(sign, x0=x0, bolts='up', slot='rear', reg=rB, port=True), T(L, 0, 0))
    if reg is not None:
        reg.items += rA.items + rT.items
        for it in rB.items:
            it = dict(it); it['p'] = (it['p'][0] + L, it['p'][1], it['p'][2]); reg.items.append(it)
    s = union([a, t, b, link_fillets(sign, xa, xb)])
    return s          # cable: hub slot -> arches on the prong -> over the fork base and the tube -> port at the far tongue
