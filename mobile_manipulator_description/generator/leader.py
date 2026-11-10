"""Leader arm v2 (2026-11-10): STS3215 low-gear servos as the joints AND the structure. One leader per follower arm.

User decisions (2026-11-10): STS3215 leader versions (C046 1:147 / C044 1:191; the stiffest one on the wrist pitch),
pan links 52 mm (0.4 x the follower's 130), as few printed parts as possible, no heat-set inserts, a fork (output horn
+ idler) on the shoulder and the pitch and the horn only elsewhere, clutch + lift on a USB foot pedal (nothing but the
trigger on the grip), and a HANGING stack: the shoulder at the top of a stand, each pan stage below the previous one,
the wrist lowest, so the hand hangs free in every pose like the follower's (an upward stack let the hand-down pose hit
the lower links). Same Waveshare bus and LeRobot Feetech code as the follower; joints map 1:1.

Layout (body frames, Z up; every pan servo has its output face DOWN, i.e. the servo's own frame):
  stand     two prints (too tall for the A1 in one): top = clamp block on the shoulder servo S1's rear end (4 x M2)
            + hollow column; foot = foot plate (4 x M5 holes or a C-clamp) + column + plug; 2 x M3 bolts across
  link1     fork on S1: bottom plate on the output horn (= the link), top prong on the idler, web in front of the case;
            S2 hangs under the plate's far end (4 x M2 up into its socket face)
  link2     plate under S2's horn, S3 hangs under it
  wrist     plate under S3's horn; clamp block under it holding the top (rear) end of the pitch servo S4 (shaft along
            Y, body up)
  pitch     fork on S4 (output disc +Y, idler -Y) + back plate holding the roll servo S5 (shaft along X, body down)
  grip      on S5's horn; C-channel holding the trigger servo S6 (shaft along Y), open on +Y
  trigger   lever on S6's output disc, finger bar under the grip
Fasteners: M3 into the threaded horns / idler discs, M2 x 6 self-tapping into the case pilot holes (the SO-100
screws). No inserts, no bearings.

Run:  python3 leader.py  (volumes)   /   leader_check.py (clashes, ranges)   /   leader_export.py   /   leader_render.py
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import box, cyl, union, xf, T, Rx, Ry, Rz, basis, CS
from joint import SERVO, sweep, _pocket, RIM, SO100, HORN_HOLES, case_screw, CTR_D
from hw import Reg, screw_cut

# ------------------------------------------------------------------ parameters (mm)
SCALE = 0.4
L1 = L2 = 130.0 * SCALE                 # 52
OUT_F, IDL_F = 18.7, 17.7               # output / idler disc faces from the servo centre (servo frame -Z / +Z)
PT = 8.0                                # link plate thickness (6 -> 8 on 2026-11-10: strength, see CLAUDE.md item 18)
BOSS = 2.2                              # idler-side prong boss (prong clears the case-face protrusions)
DZ = -(OUT_F + PT + RIM)                # -40.6: next pan servo centre below this one
XP = 16.0                               # pitch axis forward of the wrist-pan axis = 0.4 x the follower's 40 (was 50: the hand
                                        # then reached 1.44 x too far for the links and moved unlike the follower's)
ROOF = 2.8                              # plastic over the pitch servo's top end
ZPD = OUT_F + PT + ROOF + 35.2          # 62.7: pitch axis below the wrist-pan servo centre (S4 clamp right under the plate)
XR = 38.0                               # roll servo centre forward of the pitch axis
GRIP_L = 80.0                           # grip length from the roll horn face
S6X = 41.0                              # trigger servo centre in the grip frame
H_B = 306.0                             # shoulder servo centre above the desk (hanging hand >= ~19 mm over the desk)
COL_X = -130.0                          # stand column centre behind the shoulder axis; hand-down poses touching the stand
                                        # (605-pose grid, XP 16): 13.2 % at -60, 9.6 % at -90, 0 % at -130
LIMITS = {'J1': (-120, 120), 'J2': (-130, 130), 'J3': (-127, 127), 'pitch': (-20, 120), 'roll': (-125, 125)}
TRIG = (0.0, 25.0)                      # trigger range (deg)
PLUG = dict(x=(-18.5, -10.0), y=9.6)    # cable plugs on the socket face (servo frame): through-slot in seat plates

# servo frame -> body frame
S4_IN_WRIST = T(XP, 0, -ZPD) @ basis((0, 0, -1), (1, 0, 0), (0, -1, 0))   # shaft +-Y, output +Y, body up
S4_ROT = basis((0, 0, -1), (1, 0, 0), (0, -1, 0))
S5_IN_PITCH = T(XR, 0, 0) @ basis((0, 0, 1), (0, 1, 0), (-1, 0, 0))       # shaft X, output +X, body down
S5_IN_GRIP = T(-OUT_F, 0, 0) @ basis((0, 0, 1), (0, 1, 0), (-1, 0, 0))    # (grip frame: origin on the roll horn face)
S6_IN_GRIP = T(S6X, 0, 0) @ basis((1, 0, 0), (0, 0, 1), (0, -1, 0))       # shaft Y, output +Y, body back
S6_ROT = basis((1, 0, 0), (0, 0, 1), (0, -1, 0))

_cache = {}


def clamp_pocket():
    """Space the servo's rear end needs in a clamp block it slides into along -X (servo frame); X range -42..-18."""
    if 'clamp' not in _cache:
        M = basis((0, 0, 1), (0, 1, 0), (-1, 0, 0))               # servo X -> Z'
        sw = sweep(xf(SERVO, M), +1, -42.0, -18.0, c=0.0, cz=0.0)
        _cache['clamp'] = xf(sw, np.linalg.inv(M))
    return _cache['clamp']


def clamp_screws():
    """4 x M2 case screws (servo frame) for a clamp on the rear end: 2 on the socket face, 2 on the output face."""
    S = SO100; cut = []
    for (x, y) in S['horn']:
        cut.append(case_screw((x, y, RIM + S['wall']), (0, 0, -1), None, cb=20.0))
    for (x, y) in S['idler']:
        cut.append(case_screw((x, y, -RIM - S['cz_idler'] - S['wall']), (0, 0, 1), None, cb=20.0))
    return union(cut)


def seat_cut():
    """Cut (servo frame) for a plate on the servo's socket face (+Z): exact relief + plug slot + 4 M2 case screws."""
    if 'seat' not in _cache:
        S = SO100
        x0, x1, yy, rr = PLUG['x'][0], PLUG['x'][1], PLUG['y'], 1.5      # plug slot, corners rounded (FEA 2026-11-10)
        slot = union([box(x0 + rr, x1 - rr, -yy, yy, 14.0, 40.0), box(x0, x1, -yy + rr, yy - rr, 14.0, 40.0)] +
                     [cyl(rr, 14.0, 40.0, x, y) for x in (x0 + rr, x1 - rr) for y in (-yy + rr, yy - rr)])
        cut = [_pocket(+1), slot,
               cyl(CTR_D / 2, 14.0, 40.0, hole=True)]
        for (x, y) in [(-32.8, 10.25), (-32.8, -10.25), (-8.3, 10.25), (-8.3, -10.25)]:
            cut.append(case_screw((x, y, RIM + S['wall']), (0, 0, -1), None, cb=30.0))
        _cache['seat'] = union(cut)
    return _cache['seat']


def horn_cut(face, thick, screw_len, side=-1):
    """Cut (servo frame) for a plate screwed to a disc: side -1 = output disc (face at -OUT_F), +1 = idler (+IDL_F).
    4 x M3 on the 9.9 mm square, `screw_len` long, 2.5 mm into the disc; counterbore from the plate's outer face."""
    z_face = side * face
    seat = z_face + side * (screw_len - 2.5)
    outer = z_face + side * thick
    cut = [cyl(CTR_D / 2, -60, 60, hole=True)]                     # centre screw / nub, through
    for (x, y) in HORN_HOLES:
        cut.append(screw_cut((x, y, seat), (0, 0, -side), 'M3', screw_len, cb=abs(outer - seat) + 0.5))
    return union(cut)


# ------------------------------------------------------------------ frames
def frames(q=(0, 0, 0, 0, 0), trig=0.0):
    q1, q2, q3, qp, qr = q
    F = {'base': T(0, 0, 0)}
    F['link1'] = Rz(q1)
    F['link2'] = F['link1'] @ T(L1, 0, DZ) @ Rz(q2)
    F['wrist'] = F['link2'] @ T(L2, 0, DZ) @ Rz(q3)
    F['pitch'] = F['wrist'] @ T(XP, 0, -ZPD) @ Ry(qp)
    F['grip'] = F['pitch'] @ T(XR + OUT_F, 0, 0) @ Rx(qr)
    F['trigger'] = F['grip'] @ T(S6X, 0, 0) @ Ry(-trig)
    return F


# servos: (name, body they are fixed to, servo frame -> body frame)
SERVOS = [('S1_shoulder', 'base', T()), ('S2_elbow', 'link1', T(L1, 0, DZ)), ('S3_wrist_pan', 'link2', T(L2, 0, DZ)),
          ('S4_pitch', 'wrist', S4_IN_WRIST), ('S5_roll', 'pitch', S5_IN_PITCH), ('S6_trigger', 'grip', S6_IN_GRIP)]

# ------------------------------------------------------------------ parts
PARTS = {}
FASTENERS = []          # (kind, size, length, qty, note)


def add(name, solid, frame, color, note, qty=1):
    PARTS[name] = dict(solid=solid, frame=frame, color=color, note=note, qty=qty)


def fast(kind, size, length, qty, note):
    FASTENERS.append((kind, size, length, qty, note))


def pan_link(L, fork):
    """Plate under a pan servo's output horn (= the link) carrying the next servo under its far end. With fork=True
    (shoulder): + a prong on the idler above and a web in front of the case joining them."""
    W = 7.0 if fork else 15.0     # neck half width: narrow only on the shoulder fork (clears the stand's clamp block at
                                  # +-125 deg); the other links are full width (a 14 mm neck had a safety factor ~1.7)
    z0, z1 = -OUT_F - PT, -OUT_F
    parts = [cyl(13.0, z0, z1), box(0, 21.0, -W, W, z0, z1), box(14.0, L + 12.0, -15.0, 15.0, z0, z1)]
    cut = [horn_cut(OUT_F, PT, 8), xf(seat_cut(), T(L, 0, DZ))]
    if fork:
        t0, t1 = IDL_F + BOSS, IDL_F + BOSS + PT
        parts += [cyl(13.0, t0, t1), cyl(11.0, IDL_F, t0 + 0.01), box(0, 26.0, -W, W, t0, t1),
                  box(21.0, 26.0, -W, W, z0, t1)]
        cut.append(horn_cut(IDL_F, PT + BOSS, 8, side=+1))
    return union(parts) - union(cut)


def build():
    PARTS.clear(); FASTENERS.clear()

    # ---- stand: two prints (the A1 builds 256 mm high), joined by a plug + 2 x M3 bolts across the column
    #   stand_top : clamp block on S1's rear end (between the link-1 fork's prong and plate) + upper column
    #   stand_foot: foot plate (4 x M5 holes / clamp) + lower column + plug up into the upper column
    zf = -H_B
    zt, zb = IDL_F + BOSS - 0.3, -OUT_F + 0.3                      # block between the fork's top prong and link plate
    CX, CY, WALL = (COL_X - 13.0, COL_X + 13.0), 20.0, 3.0         # column 26 x 40, hollow, 3 mm walls
    zs = zf + 150.0                                                # split height
    inner = lambda z0, z1, e=0.0: box(CX[0] + WALL + e, CX[1] - WALL - e, -CY + WALL + e, CY - WALL - e, z0, z1)
    PLUG_L, CH = 45.0, 2.0                                         # plug length (30 -> 45) and root chamfer (FEA 2026-11-10)
    bolts = [xf(cyl(1.7, -30, 30, hole=True), T((CX[0] + CX[1]) / 2, 0, zs + z) @ Rx(90)) for z in (12.0, 34.0)]
    wP, hP = CX[1] - CX[0] - 2 * WALL - 0.4, 2 * CY - 2 * WALL - 0.4
    def frustum(e):     # chamfer ring at the plug root: plug + CH (+ e) at zs, plug (+ e) at zs + CH
        w0, h0 = wP + 2 * CH + 2 * e, hP + 2 * CH + 2 * e
        return CS.square((w0, h0), True).extrude(CH, 0, 0.0, ((wP + 2 * e) / w0, (hP + 2 * e) / h0)).translate(((CX[0] + CX[1]) / 2, 0, zs))
    top = union([box(-42.0, -21.0, -16.0, 16.0, zb, zt), box(CX[0], -21.0, -16.0, 16.0, zb, zt),   # clamp + beam back
                 box(CX[0], CX[1], -CY, CY, zs, zt)])
    top = top - union([clamp_pocket(), clamp_screws(), inner(zs - 1.0, zt - 25.0), frustum(0.2)] + bolts)
    add('stand_top', top, 'base', (0.85, 0.45, 0.15), 'stand, upper part: clamp on the shoulder servo + hollow column')
    foot = union([box(CX[0] - 40.0, 20.0, -45.0, 45.0, zf, zf + 5.0), box(CX[0], CX[1], -CY, CY, zf + 4.0, zs),
                  box(CX[0] - 40.0, CX[0] + 0.01, -3.0, 3.0, zf + 4.0, zf + 50.0), box(CX[1] - 0.01, 10.0, -3.0, 3.0, zf + 4.0, zf + 14.0),
                  box(CX[0], CX[1], -45.0, 45.0, zf + 4.0, zf + 12.0),       # ribs <= 14 mm: the hanging hand passes 19 mm up
                  inner(zs - 1.0, zs + PLUG_L, 0.2), frustum(0.0)])         # plug, 0.2 mm clearance in the upper column
    holes = [cyl(2.75, zf - 1, zf + 6, x, y, hole=True) for x in (CX[0] - 30.0, 10.0) for y in (-35.0, 35.0)]
    # 3 mm column walls; the plug's hollow starts 8 mm down, so a solid 8 mm band (was 1 mm) joins plug and column walls
    foot = foot - union([inner(zf + 7.0, zs - 8.0), inner(zs - 8.1, zs + PLUG_L + 1.0, 3.2)] + bolts + holes)
    add('stand_foot', foot, 'base', (0.85, 0.45, 0.15), 'stand, lower part: foot plate (4 x M5 / clamp) + column + plug')
    fast('selftap', 'M2', 6, 4, 'shoulder servo case (stand clamp)')
    fast('SHCS', 'M3', 50, 2, 'stand joint (across the column, nyloc nut)'); fast('nut', 'M3', None, 2, 'stand joint (nyloc)')

    add('link1', pan_link(L1, True), 'link1', (0.30, 0.55, 0.85),
        'shoulder fork (output horn + idler) = link 52 mm; the elbow servo hangs under its far end')
    fast('SHCS', 'M3', 8, 8, 'shoulder horn (4) + idler (4)'); fast('selftap', 'M2', 6, 4, 'elbow servo case')
    add('link2', pan_link(L2, False), 'link2', (0.30, 0.55, 0.85),
        'link 52 mm under the elbow horn; the wrist-pan servo hangs under its far end')
    fast('SHCS', 'M3', 8, 4, 'elbow horn'); fast('selftap', 'M2', 6, 4, 'wrist-pan servo case')

    # ---- wrist: plate under S3's horn + clamp block under it on S4's top (rear) end
    z0, z1 = -OUT_F - PT, -OUT_F
    plate = union([cyl(14.0, z0, z1), box(0, XP + 14.0, -14.0, 14.0, z0, z1)])
    blk = box(XP - 14.0, XP + 14.0, -18.2, 18.2, -ZPD + 18.0, z1)
    w = union([plate, blk]) - union([horn_cut(OUT_F, PT, 8), xf(clamp_pocket(), S4_IN_WRIST),
                                     xf(clamp_screws(), S4_IN_WRIST)])
    add('wrist', w, 'wrist', (0.93, 0.70, 0.25), 'plate under the wrist-pan horn + clamp holding the pitch servo')
    fast('SHCS', 'M3', 8, 4, 'wrist-pan horn'); fast('selftap', 'M2', 6, 4, 'pitch servo case (wrist clamp)')

    # ---- pitch fork (frame: origin on the pitch axis, X forward, Y = axis): prongs on S4's discs + back plate for S5
    xb1 = XR - RIM                                                  # back plate front face = S5's socket-face rim
    xb0 = xb1 - 4.0                                                 # 2.5 -> 4 mm (FEA 2026-11-10: weak under a side push)
    pr = []
    for sgn, face in ((+1, OUT_F), (-1, IDL_F)):
        y0, y1 = sorted((sgn * 19.2, sgn * 25.2))
        b0, b1 = sorted((sgn * face, sgn * 19.2))
        pr += [xf(cyl(13.0, y0, y1), Rx(-90)), box(0, xb1, y0, y1, -11.0, 11.0),
               box(12.0, xb1, y0, y1, -36.0, 11.0), xf(cyl(11.0, b0, b1), Rx(-90))]
    gus = [box(xb0 - 3.0, xb0 + 0.01, y0, y1, -36.0, 0.0) for (y0, y1) in ((13.5, 16.5), (-16.5, -13.5))]   # behind the plate; low
    pf = union(pr + [box(xb0, xb1, -25.2, 25.2, -36.0, 12.0)] + gus)        # half only (the upper half meets S4 at pitch -20)
    pf = pf - union([xf(horn_cut(OUT_F, 6.5, 8), S4_ROT), xf(horn_cut(IDL_F, 7.5, 8, side=+1), S4_ROT),
                     xf(seat_cut(), S5_IN_PITCH)])
    add('pitch_fork', pf, 'pitch', (0.80, 0.35, 0.35), 'pitch fork (output horn + idler) and back plate holding the roll servo')
    fast('SHCS', 'M3', 8, 8, 'pitch horn (4) + idler (4)'); fast('selftap', 'M2', 6, 4, 'roll servo case')

    # ---- grip (frame: origin on the roll horn face, X forward): back plate + C-channel around S6, open on +Y
    GY = (-17.9, 17.4)
    g = union([box(0, 4.0, GY[0], GY[1], -14.4, 14.4),                     # back plate on the roll horn
               box(0, GRIP_L, GY[0], -15.9, -14.4, 14.4),                  # -Y wall (on S6's socket face)
               box(0, GRIP_L, GY[0], GY[1], 12.4, 14.4), box(0, GRIP_L, GY[0], GY[1], -14.4, -12.4),
               box(GRIP_L - 2.0, GRIP_L, GY[0], GY[1], -14.4, 14.4)])
    g = g - union([xf(horn_cut(OUT_F, 4.0, 6), S5_IN_GRIP), xf(seat_cut(), S6_IN_GRIP)])
    add('grip', g, 'grip', (0.25, 0.25, 0.28), 'grip on the roll horn; holds the trigger servo (slides in from the +Y side)')
    fast('SHCS', 'M3', 6, 4, 'roll horn (from inside the grip)'); fast('selftap', 'M2', 6, 4, 'trigger servo case')

    # ---- trigger lever (frame: on S6's axis, X forward, Y = axis): disc on the output horn, arm down, finger bar under
    y0, y1 = OUT_F, OUT_F + 4.0
    tr = union([xf(cyl(13.0, y0, y1), Rx(-90)), box(-6.0, 6.0, y0, y1, -32.0, 0.0),
                box(-6.0, 26.0, y0, y1, -36.0, -30.0), box(8.0, 26.0, -16.0, y1, -36.0, -30.0)])
    tr = tr - xf(horn_cut(OUT_F, 4.0, 6), S6_ROT)
    add('trigger', tr, 'trigger', (0.20, 0.20, 0.22), 'trigger lever on the trigger servo horn; finger bar under the grip')
    fast('SHCS', 'M3', 6, 4, 'trigger horn')


def world(q=(0, 0, 0, 0, 0), trig=0.0, bought=True):
    """(name, solid in world, colour, body) for every part (+ servos when bought). Desk at z = -H_B."""
    F = frames(q, trig)
    out = [(nm, xf(p['solid'], F[p['frame']]), p['color'], p['frame']) for nm, p in PARTS.items()]
    if bought:
        for nm, body, M in SERVOS:
            out.append((nm, xf(SERVO, F[body] @ M), (0.12, 0.12, 0.14), body))
    return out


if __name__ == '__main__':
    build()
    tot = 0
    for nm, p in PARTS.items():
        v = p['solid'].volume() / 1000; tot += v
        print(f'{nm:12s} {v:6.1f} cm3  bbox {[round(x, 1) for x in p["solid"].bounding_box()]}')
    print(f'total {tot:.1f} cm3')
