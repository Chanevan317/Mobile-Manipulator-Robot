"""Leader arm v3 (2026-11-11): a 0.7-scale copy of the follower's kinematics, built from light leader-only parts.
v1 (AS5600 encoders) and v2 (hanging servo stack) are in git.

User decisions (2026-11-11): close to the follower ("easier teleop"), bigger is fine (0.5 .. 0.7 x), new lighter parts
(no bearings, no inserts: the servo's output horn + idler carry each joint, like the SO-100 leader), one arm module
for BOTH followers with only the base swapped (mono: one pedestal, shoulder +-180; two-arm prototype: two pedestals at
the follower's mount yaw +-135, a column stand-in between), used SEATED with TWO hands (user, after the SO-100: one
hand on the gripper, the other on a handle on the forearm, which steers the elbow, the one joint the hand alone leaves
ambiguous); the gripper is a PINCH gripper (user: thumb on one side, 2+ fingers on the other, scissor-like, no gears). Servos: STS3215 low-gear (C046
1:147 / C044 1:191, the stiffest on the pitch), same Waveshare bus + LeRobot Feetech code as the follower; joints map 1:1.

Layout (frames: Z up, arm plane z = 0 through the elbow / wrist-pan servo centres, like the follower):
  shoulder  S1 stands upright, output horn UP, in the shoulder mount (a seat on its socket face, 4 x M2); link 1 sits
            on that horn, above everything fixed, so the shoulder turns +-180 (the follower mono shoulder's range)
  links     every pan servo's output faces down (as the follower's). Link N+1 is a fork around servo N+1: bottom prong
            on the output horn, top prong on the idler, joined by a web 41..46 mm out, BEYOND the servo case (its rear
            corners are 37.3 mm from the axis) so the joint folds to ~+-150 deg; its far end clamps the next servo's
            rear end (4 x M2)
  wrist     fork around the wrist-pan servo + a clamp under it holding the pitch servo (shaft along Y, body up)
  pitch     fork on the pitch servo's discs + back plate holding the roll servo (shaft along X, body down)
  gripper   on the roll horn: base holding the gripper servo (shaft along Z, horn down) + a FIXED jaw (+Y), and a
            moving jaw on the servo horn (-Y), like pliers (one shared pivot for both jaws would let the pair swing
            unmeasured); the same oval finger loop on both jaws (thumb / 2-3 fingers, either hand); opens 0..40 deg
  handle    bar on the forearm (link 2) for the second hand
Bases (swap = 4 x M3 under the shoulder mount; the pedestal top has 8 holes every 45 deg, so the arm mounts at any
multiple of 45 deg): mono_foot + pedestal; proto: 2 x (foot half + pedestal), joined in the middle under a stand-in
for the lift column, arms at y +-105 (0.7 x 150) facing yaw +-135.

Run:  python3 leader.py  (volumes)   /   leader_check.py (clashes, ranges)   /   leader_export.py   /   leader_render.py
"""
import sys, os, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import box, cyl, union, xf, T, Rx, Ry, Rz, basis, CS
from joint import SERVO, sweep, _pocket, RIM, SO100, HORN_HOLES, case_screw, CTR_D
from hw import Reg, screw_cut, nut_cut

# ------------------------------------------------------------------ parameters (mm)
SCALE = 0.7
L1 = L2 = 130.0 * SCALE                 # 91 (the follower's 130)
OUT_F, IDL_F = 18.7, 17.7               # output / idler disc faces from the servo centre (servo frame -Z / +Z)
PT = 8.0                                # prong / plate thickness
BOSS = 2.2                              # idler-side prong boss (clears the case-face protrusions)
PR_B = (-OUT_F - PT, -OUT_F)            # bottom prong (output horn) z
PR_T = (IDL_F + BOSS, IDL_F + BOSS + PT)  # top prong (idler) z
WEB = (41.0, 46.0)                      # fork web, beyond the case's rear corners (r 37.3)
W = 10.0                                # fork arm / web / beam half width
BEAM_Z = 15.0                           # beam half height
CL = dict(x=(-37.5, -18.0), y=14.5, z=17.9)   # clamp on a servo's rear end (servo frame)
XP = 40.0 * SCALE                       # 28: pitch axis forward of the wrist-pan axis (follower 40)
ROOF = 2.8
ZPD = OUT_F + PT + ROOF + 35.2 - 2.0    # 62.7: pitch axis below the arm plane (follower 74.6 x 0.7 = 52: the servo needs room)
XR = 85.0 * SCALE                       # 59.5: roll servo centre forward of the pitch axis (follower 85)
# pinch gripper (grip frame: origin on the roll horn face, X forward along the roll axis, Z up at roll 0, Y left):
# the gripper servo S6 stands with its shaft along Z at x = PX, output horn DOWN; one jaw is fixed to the base (+Y),
# the other turns on S6's horn (-Y); both carry the same finger loop (thumb in one, 2-3 fingers in the other: either hand)
PX = 40.0
JAW = dict(x1=95.0, y_in=12.0, t=3.0, z=(-50.0, -18.7))     # blade inner face |y| 12 (24 mm apart closed), tips at x 95
LOOP = dict(xc=70.0, L=50.0, w=22.0, wall=3.0)              # oval finger loop (inside 50 x 22) on each blade's outer face
HANDLE_F = dict(x=(15.0, 95.0), r=12.0, z=50.0, posts=(32.0, None))   # forearm handle bar (second post at the beam hole)
ZC1 = -(OUT_F + PT / 2)                 # -22.7: shoulder servo centre (link 1 sits on its horn at z -4 .. 4)
H_ARM = 260.0                           # arm plane above the desk (seated; the gripper hanging straight down clears the desk)
MOUNT_R, MOUNT_Z = 38.0, ZC1 - RIM - PT  # shoulder mount disc radius / underside (-46.6)
PROTO_Y, PROTO_YAW = 150.0 * SCALE, 135.0
LIMITS = {'J1': (-180, 180), 'J2': (-130, 130), 'J3': (-127, 127), 'pitch': (-20, 120), 'roll': (-125, 125)}
LIMITS_PROTO_J1 = (-120, 120)
TRIG = (0.0, 40.0)                      # jaw opening (deg): thumb-finger span ~46 -> ~85 mm; maps to the follower's 0..90 mm
PLUG = dict(x=(-18.5, -10.0), y=9.6)

S1_IN_BASE = T(0, 0, ZC1) @ Rx(180)                                         # output up, body toward -x
S4_IN_WRIST = T(XP, 0, -ZPD) @ basis((0, 0, -1), (1, 0, 0), (0, -1, 0))   # shaft +-Y, output +Y, body up
S4_ROT = basis((0, 0, -1), (1, 0, 0), (0, -1, 0))
S5_IN_PITCH = T(XR, 0, 0) @ basis((0, 0, 1), (0, 1, 0), (-1, 0, 0))       # shaft X, output +X, body down
S5_IN_GRIP = T(-OUT_F, 0, 0) @ basis((0, 0, 1), (0, 1, 0), (-1, 0, 0))
S6_IN_GRIP = T(PX, 0, 0)                                                    # shaft Z, output DOWN, body back (-X)

_cache = {}


def clamp_pocket():
    """Space the servo's rear end needs in a clamp block it slides into along -X (servo frame)."""
    if 'clamp' not in _cache:
        M = basis((0, 0, 1), (0, 1, 0), (-1, 0, 0))
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
        x0, x1, yy, rr = PLUG['x'][0], PLUG['x'][1], PLUG['y'], 1.5
        slot = union([box(x0 + rr, x1 - rr, -yy, yy, 14.0, 40.0), box(x0, x1, -yy + rr, yy - rr, 14.0, 40.0)] +
                     [cyl(rr, 14.0, 40.0, x, y) for x in (x0 + rr, x1 - rr) for y in (-yy + rr, yy - rr)])
        cut = [_pocket(+1), slot, cyl(CTR_D / 2, 14.0, 40.0, hole=True)]
        for (x, y) in [(-32.8, 10.25), (-32.8, -10.25), (-8.3, 10.25), (-8.3, -10.25)]:
            cut.append(case_screw((x, y, RIM + S['wall']), (0, 0, -1), None, cb=30.0))
        _cache['seat'] = union(cut)
    return _cache['seat']


def horn_cut(face, thick, screw_len, side=-1):
    """Cut (servo frame) for a plate on a disc: side -1 = output (face -OUT_F), +1 = idler (+IDL_F); 4 x M3 on 9.9 mm."""
    z_face = side * face
    seat = z_face + side * (screw_len - 2.5)
    outer = z_face + side * thick
    cut = [cyl(CTR_D / 2, -60, 60, hole=True)]
    for (x, y) in HORN_HOLES:
        cut.append(screw_cut((x, y, seat), (0, 0, -side), 'M3', screw_len, cb=abs(outer - seat) + 0.5))
    return union(cut)


# ------------------------------------------------------------------ frames
def frames(q=(0, 0, 0, 0, 0), trig=0.0, root=None):
    q1, q2, q3, qp, qr = q
    F = {'base': np.eye(4) if root is None else root}
    F['link1'] = F['base'] @ Rz(q1)
    F['link2'] = F['link1'] @ T(L1, 0, 0) @ Rz(q2)
    F['wrist'] = F['link2'] @ T(L2, 0, 0) @ Rz(q3)
    F['pitch'] = F['wrist'] @ T(XP, 0, -ZPD) @ Ry(qp)
    F['grip'] = F['pitch'] @ T(XR + OUT_F, 0, 0) @ Rx(qr)
    F['jaw'] = F['grip'] @ T(PX, 0, 0) @ Rz(-trig)                     # moving jaw (-Y side) opens toward -Y
    return F


SERVOS = [('S1_shoulder', 'base', S1_IN_BASE), ('S2_elbow', 'link1', T(L1, 0, 0)), ('S3_wrist_pan', 'link2', T(L2, 0, 0)),
          ('S4_pitch', 'wrist', S4_IN_WRIST), ('S5_roll', 'pitch', S5_IN_PITCH), ('S6_gripper', 'grip', S6_IN_GRIP)]

PARTS = {}            # arm module: name -> dict(solid, frame, color, note, qty)
BASE_PARTS = {}       # bases: name -> dict(solid (base frame of ONE arm for 'pedestal'; world for the rest), variant, ...)
FASTENERS = []


def add(name, solid, frame, color, note, qty=1):
    PARTS[name] = dict(solid=solid, frame=frame, color=color, note=note, qty=qty)


def fast(kind, size, length, qty, note):
    FASTENERS.append((kind, size, length, qty, note))


# ------------------------------------------------------------------ arm parts
def fork(beam_to=None):
    """Fork around a pan servo (link frame: origin on its axis, servo = body frame, output down) and, with beam_to,
    the beam to the next servo's clamp at x = beam_to."""
    parts = [cyl(13.0, *PR_B), box(0, WEB[1], -W, W, *PR_B),                     # bottom prong on the output horn
             cyl(13.0, *PR_T), cyl(11.0, IDL_F, PR_T[0] + 0.01), box(0, WEB[1], -W, W, *PR_T),   # top prong on the idler
             box(WEB[0], WEB[1], -W, W, PR_B[0], PR_T[1])]                       # web
    cut = [horn_cut(OUT_F, PT, 8), horn_cut(IDL_F, PT + BOSS, 8, side=+1)]
    if beam_to is not None:
        parts += [box(WEB[1] - 0.01, beam_to + CL['x'][0] + 0.01, -W, W, -BEAM_Z, BEAM_Z)] + clamp_block(beam_to)
        cut += clamp_cut(beam_to)
    return parts, cut


def clamp_block(x):
    return [box(x + CL['x'][0], x + CL['x'][1], -CL['y'], CL['y'], -CL['z'], CL['z'])]


def clamp_cut(x):
    return [xf(clamp_pocket(), T(x, 0, 0)), xf(clamp_screws(), T(x, 0, 0))]


def build():
    PARTS.clear(); BASE_PARTS.clear(); FASTENERS.clear()

    # ---- shoulder mount: seat on S1's socket face (down) + low walls + disc with 4 x M3 to the pedestal (at 45 deg)
    seat = xf(box(-38.6, 13.6, -15.4, 15.4, RIM, RIM + PT), S1_IN_BASE)
    disc = cyl(MOUNT_R, MOUNT_Z, MOUNT_Z + PT)
    walls = box(-38.6, 13.6, -15.4, 15.4, MOUNT_Z, -10.0) - box(-35.6, 10.6, -12.8, 12.8, MOUNT_Z + PT - 0.01, 0)
    m = union([seat, disc, walls]) - union([xf(seat_cut(), S1_IN_BASE)] +
                                           [cyl(1.7, MOUNT_Z - 1, MOUNT_Z + PT + 1, 30 * math.cos(math.radians(a)), 30 * math.sin(math.radians(a)), hole=True)
                                            for a in (45, 135, 225, 315)])
    add('shoulder_mount', m, 'base', (0.85, 0.45, 0.15), 'holds the shoulder servo (horn up) by its socket face; '
        '4 x M3 down into the pedestal (any multiple of 45 deg)')
    fast('selftap', 'M2', 6, 4, 'shoulder servo case'); fast('SHCS', 'M3', 20, 4, 'shoulder mount to pedestal (nut in the cap underside)')
    fast('nut', 'M3', None, 4, 'shoulder mount')

    # ---- link 1: plate on S1's horn (z -4 .. 4, above everything fixed: +-180) + beam + clamp on S2's rear end
    zp = (ZC1 + OUT_F, ZC1 + OUT_F + PT)
    l1 = union([cyl(14.0, *zp), box(0, L1 + CL['x'][0] + 0.01, -W, W, zp[0], BEAM_Z)] + clamp_block(L1))
    l1 = l1 - union([xf(horn_cut(OUT_F, PT, 8), S1_IN_BASE)] + clamp_cut(L1))
    add('link1', l1, 'link1', (0.30, 0.55, 0.85), f'upper arm {L1:.0f} mm: plate on the shoulder horn, clamp on the elbow servo')
    fast('SHCS', 'M3', 8, 4, 'shoulder horn'); fast('selftap', 'M2', 6, 4, 'elbow servo case (link 1 clamp)')

    # ---- link 2: fork around S2 + beam + clamp on S3; guide-knob hole on top of the beam
    p, c = fork(beam_to=L2)
    xk = (WEB[1] + L2 + CL['x'][0]) / 2
    c += [cyl(1.7, -BEAM_Z - 1, BEAM_Z + 1, xk, 0, hole=True), cyl(3.2, -BEAM_Z - 1, -BEAM_Z + 3.0, xk, 0, hole=True)]
    add('link2', union(p) - union(c), 'link2', (0.30, 0.55, 0.85),
        f'forearm {L2:.0f} mm: fork on the elbow servo (horn + idler), clamp on the wrist-pan servo; prints on its side')
    fast('SHCS', 'M3', 8, 8, 'elbow horn (4) + idler (4)'); fast('selftap', 'M2', 6, 4, 'wrist-pan servo case (link 2 clamp)')

    # ---- forearm handle (second hand: steers the elbow): bar on two posts, one on link 2's top prong arm, one on the beam
    Hf = HANDLE_F; xa = Hf['posts'][0]
    bar = xf(cyl(Hf['r'], Hf['x'][0], Hf['x'][1]), T(0, 0, Hf['z']) @ Ry(90))
    hd = union([bar, cyl(7.0, PR_T[1] - 0.01, Hf['z'], xa, 0), cyl(7.0, BEAM_Z - 0.01, Hf['z'], xk, 0)])
    zt = Hf['z'] + Hf['r']
    hd = hd - union([screw_cut((xa, 0, 37.0), (0, 0, -1), 'M3', 37.0 - PR_T[0] + 2.6, cb=zt - 37.0 + 1),
                     screw_cut((xk, 0, 30.0), (0, 0, -1), 'M3', 30.0 + BEAM_Z, cb=zt - 30.0 + 1)])
    add('forearm_handle', hd, 'link2', (0.20, 0.20, 0.22), 'handle bar on the forearm for the second hand (steers the elbow)')
    fast('SHCS', 'M3', 20, 1, 'forearm handle, front post (nut under link 2\'s top prong)')
    fast('SHCS', 'M3', 45, 1, 'forearm handle, rear post (nut in link 2\'s beam)'); fast('nut', 'M3', None, 2, 'forearm handle')

    # ---- wrist: fork around S3 + clamp under the bottom prong holding S4's rear (top) end
    p, c = fork()
    blk = box(XP - CL['y'], XP + CL['y'], -CL['z'], CL['z'], -ZPD + 18.0, PR_B[1])        # S4 clamp (S4 frame: y_s -> x)
    p += [blk, box(0, WEB[1], -CL['z'], CL['z'], *PR_B)]
    c += [xf(clamp_pocket(), S4_IN_WRIST), xf(clamp_screws(), S4_IN_WRIST)]
    add('wrist', union(p) - union(c), 'wrist', (0.93, 0.70, 0.25),
        'fork on the wrist-pan servo (horn + idler) + clamp holding the pitch servo')
    fast('SHCS', 'M3', 8, 8, 'wrist-pan horn (4) + idler (4)'); fast('selftap', 'M2', 6, 4, 'pitch servo case (wrist clamp)')

    # ---- pitch fork (frame: origin on the pitch axis, X forward, Y = axis): prongs on S4's discs + back plate for S5
    xb1 = XR - RIM; xb0 = xb1 - 4.0
    pr = []
    for sgn, face in ((+1, OUT_F), (-1, IDL_F)):
        y0, y1 = sorted((sgn * 19.2, sgn * 25.2))
        b0, b1 = sorted((sgn * face, sgn * 19.2))
        pr += [xf(cyl(13.0, y0, y1), Rx(-90)), box(0, xb1, y0, y1, -11.0, 11.0), box(12.0, xb1, y0, y1, -36.0, 11.0),
               xf(cyl(11.0, b0, b1), Rx(-90))]
    gus = [box(xb0 - 3.0, xb0 + 0.01, y0, y1, -36.0, 0.0) for (y0, y1) in ((13.5, 16.5), (-16.5, -13.5))]
    pf = union(pr + [box(xb0, xb1, -25.2, 25.2, -36.0, 12.0)] + gus)
    pf = pf - union([xf(horn_cut(OUT_F, 6.5, 8), S4_ROT), xf(horn_cut(IDL_F, 7.5, 8, side=+1), S4_ROT), xf(seat_cut(), S5_IN_PITCH)])
    add('pitch_fork', pf, 'pitch', (0.80, 0.35, 0.35), 'pitch fork (output horn + idler) + back plate holding the roll servo')
    fast('SHCS', 'M3', 8, 8, 'pitch horn (4) + idler (4)'); fast('selftap', 'M2', 6, 4, 'roll servo case')

    # ---- pinch gripper: base (back plate on the roll horn + clamp on S6's rear end + fixed jaw) and the moving jaw
    J, Lp = JAW, LOOP
    def loop(sy, x0=0.0):
        """Oval finger loop on a blade's outer face (sy = +1: +Y side), full jaw height; in its part's frame."""
        yo = sy * (J['y_in'] + J['t']); yc = yo + sy * (Lp['wall'] + Lp['w'] / 2)
        def stadium(e, z0, z1):
            r = Lp['w'] / 2 + e; xa, xb = Lp['xc'] - x0 - Lp['L'] / 2 + Lp['w'] / 2, Lp['xc'] - x0 + Lp['L'] / 2 - Lp['w'] / 2
            return union([box(xa, xb, yc - r, yc + r, z0, z1), cyl(r, z0, z1, xa, yc), cyl(r, z0, z1, xb, yc)])
        return stadium(Lp['wall'], *J['z']) - stadium(0.0, J['z'][0] - 1, J['z'][1] + 1)
    yi, yo = J['y_in'], J['y_in'] + J['t']
    back = box(0, 4.0, -16.0, 16.0, -14.4, 14.4)
    clamp = box(PX + CL['x'][0], PX + CL['x'][1], -CL['y'], CL['y'], -CL['z'], CL['z'])
    block = box(PX + CL['x'][0], PX + CL['x'][1], -CL['y'], yo, J['z'][0], -CL['z'] + 0.01)      # under the clamp: prints without supports
    blade = box(4.0, J['x1'], yi, yo, J['z'][0], -17.0)
    stop = box(J['x1'] - 7.0, J['x1'], 1.0, yi + 0.01, J['z'][0], J['z'][0] + 8.0)               # closed stop (meets the other jaw's)
    gb = union([back, clamp, block, blade, stop, loop(+1)])
    S = SO100; cut = [xf(horn_cut(OUT_F, 4.0, 6), S5_IN_GRIP), xf(clamp_pocket(), S6_IN_GRIP), xf(clamp_screws(), S6_IN_GRIP)]
    for (x, y) in S['horn']:                                                                     # driver access to the output-face case screws
        cut.append(cyl(2.6, J['z'][0] - 1, -CL['z'] + 0.5, PX + x, y, hole=True))
    add('gripper_base', gb - union(cut), 'grip', (0.25, 0.25, 0.28),
        'pinch gripper base on the roll horn: holds the gripper servo; fixed jaw with a finger loop (+Y); prints upright')
    fast('SHCS', 'M3', 6, 4, 'roll horn (gripper base back plate)'); fast('selftap', 'M2', 6, 4, 'gripper servo case')
    # moving jaw (frame: on S6's axis): plate on the output horn, connector, blade on -Y, loop, closed stop
    mj = union([cyl(11.0, -OUT_F - PT, -OUT_F), box(-6.0, 6.0, -yo, 0, -OUT_F - PT, -OUT_F),
                box(0, J['x1'] - PX, -yo, -yi, J['z'][0], -OUT_F), loop(-1, PX),
                box(J['x1'] - PX - 7.0, J['x1'] - PX, -yi - 0.01, -1.0 - 2.0, J['z'][0], J['z'][0] + 8.0)])
    mj = mj - horn_cut(OUT_F, PT, 8)
    add('gripper_jaw', mj, 'jaw', (0.20, 0.20, 0.22), 'moving jaw on the gripper servo horn, with a finger loop (-Y); '
        'opens 0..40 deg; prints upside down (horn plate on the bed)')
    fast('SHCS', 'M3', 8, 4, 'gripper servo horn')

    build_bases()


# ------------------------------------------------------------------ bases (swap: 4 x M3 under the shoulder mount)
def pedestal():
    """Slim hollow column (r 26) from the desk to a cap (r 38) under the shoulder mount (base frame of one arm). The cap
    has 8 M3 holes every 45 deg with nut pockets open on its underside (reachable from outside)."""
    zd = -H_ARM; zt = MOUNT_Z; ct = 10.0
    s = union([cyl(MOUNT_R, zt - ct, zt), cyl(26.0, zd + 4.0, zt - ct + 0.01), cyl(52.0, zd, zd + 6.0),
               *[xf(box(0, 26.0, -2.0, 2.0, zd + 5.9, zd + 40.0), Rz(120.0 * k + 60.0)) for k in range(3)]])   # 3 ribs
    s = s - cyl(23.0, zd + 3.0, zt - ct - 0.5)
    cut = []
    for k in range(8):
        a = math.radians(45.0 * k + 45.0)
        x, y = 30 * math.cos(a), 30 * math.sin(a)
        cut += [cyl(1.7, zt - ct - 1, zt + 1, x, y, hole=True), nut_cut((x, y, zt - ct - 0.01), (0, 0, 1), 'M3', depth=4.0)]
    for k in range(3):                                                                       # 3 x M5 to the foot
        a = math.radians(120.0 * k)
        cut.append(cyl(2.75, zd - 1, zd + 7.0, 44 * math.cos(a), 44 * math.sin(a), hole=True))
    cut.append(box(-6.0, 6.0, -30.0, 0.0, zd + 3.0, zd + 15.0))                             # cable exit at the bottom
    return s - union(cut)


def build_bases():
    ped = pedestal()
    BASE_PARTS['pedestal'] = dict(solid=ped, variant='both', qty_mono=1, qty_proto=2, color=(0.55, 0.57, 0.60),
                                  note='column under the shoulder mount; 8 nut traps (arm at any multiple of 45 deg); 3 x M5 to a foot')
    zd = -H_ARM
    holes = lambda cx, cy, yaw: [cyl(2.75, zd - 7.0, zd + 1, cx + 44 * math.cos(math.radians(120 * k + yaw)),
                                     cy + 44 * math.sin(math.radians(120 * k + yaw)), hole=True) for k in range(3)]
    mono = box(-110.0, 110.0, -85.0, 85.0, zd - 6.0, zd) - union(holes(0, 0, 0) +
           [box(x - 10, x + 10, y - 4, y + 4, zd - 7, zd + 1) for x in (-95.0, 95.0) for y in (-70.0, 70.0)])
    mono = mono - union([box(x0, x1, y0, y1, zd - 7, zd + 1) for (x0, x1) in ((-85.0, -58.0), (58.0, 85.0)) for (y0, y1) in ((-60.0, -8.0), (8.0, 60.0))]
                        + [box(-45.0, 45.0, y0, y1, zd - 7, zd + 1) for (y0, y1) in ((-70.0, -58.0), (58.0, 70.0))])
    BASE_PARTS['mono_foot'] = dict(solid=mono, variant='mono', qty_mono=1, qty_proto=0, color=(0.40, 0.42, 0.45),
                                   note='foot for one leader (mono follower): C-clamp the slotted corners to the desk')
    # proto: two halves (the plate is 330 mm long), each with one pedestal seat, joined under the column stand-in
    halves = []
    for sy in (+1, -1):
        y0, y1 = sorted((sy * 4.0, sy * (PROTO_Y + 70.0)))
        h = box(-90.0, 90.0, y0, y1, zd - 6.0, zd)
        h = union([h, box(-14.0, 14.0, min(sy * 4.0, sy * 28.0), max(sy * 4.0, sy * 28.0), zd, zd + 60.0)])   # half the column stand-in (0.7 x 40 x 80)
        h = h - union(holes(0, sy * PROTO_Y, 0) + [box(x - 10, x + 10, sy * (PROTO_Y + 55.0) - 4, sy * (PROTO_Y + 55.0) + 4, zd - 7, zd + 1) for x in (-75.0, 75.0)]
                      + [xf(cyl(1.7, -20, 20, hole=True), T(x, sy * 4.0, zd + 30.0) @ Rx(90)) for x in (-7.0, 7.0)]
                      + [cyl(2.75, zd - 7, zd + 1, x, sy * 12.0, hole=True) for x in (-60.0, 60.0)])
        h = h - union([box(x0, x1, min(sy * y0, sy * y1), max(sy * y0, sy * y1), zd - 7, zd + 1)
                       for (x0, x1) in ((-75.0, -58.0), (58.0, 75.0)) for (y0, y1) in ((35.0, 85.0), (125.0, 160.0))])
        halves.append(h)
    BASE_PARTS['proto_foot_left'] = dict(solid=halves[0], variant='proto', qty_mono=0, qty_proto=1, color=(0.40, 0.42, 0.45),
                                         note='foot half for the two-arm prototype (left arm); bolts to the right half (2 x M3 through the stand-in, 2 x M5 under)')
    BASE_PARTS['proto_foot_right'] = dict(solid=halves[1], variant='proto', qty_mono=0, qty_proto=1, color=(0.40, 0.42, 0.45),
                                          note='foot half for the two-arm prototype (right arm)')


def roots(variant='mono'):
    """Base frame of each arm module in the world (desk at z = -H_ARM)."""
    if variant == 'mono':
        return [np.eye(4)]
    return [T(0, PROTO_Y, 0) @ Rz(PROTO_YAW), T(0, -PROTO_Y, 0) @ Rz(-PROTO_YAW)]


def world(q=(0, 0, 0, 0, 0), trig=0.0, bought=True, variant='mono', q2=None):
    """(name, solid, colour, body) for every part in the world, incl. the base of `variant`. For 'proto', q drives the
    left arm and q2 (default q mirrored) the right one."""
    out = []
    qs = [q] if variant == 'mono' else [q, q2 if q2 is not None else tuple(-v if i in (0, 1, 2, 4) else v for i, v in enumerate(q))]
    for ai, (R0, qa) in enumerate(zip(roots(variant), qs)):
        F = frames(qa, trig, root=R0); tag = '' if variant == 'mono' else ('L_', 'R_')[ai]
        for nm, p in PARTS.items():
            out.append((tag + nm, xf(p['solid'], F[p['frame']]), p['color'], tag + p['frame']))
        if bought:
            for nm, body, M in SERVOS:
                out.append((tag + nm, xf(SERVO, F[body] @ M), (0.12, 0.12, 0.14), tag + body))
        out.append((tag + 'pedestal', xf(BASE_PARTS['pedestal']['solid'], R0), BASE_PARTS['pedestal']['color'], tag + 'base'))
    for nm, p in BASE_PARTS.items():
        if p['variant'] == variant:
            out.append((nm, p['solid'], p['color'], 'desk'))
    return out


if __name__ == '__main__':
    build()
    tot = 0
    for nm, p in list(PARTS.items()) + list(BASE_PARTS.items()):
        v = p['solid'].volume() / 1000; tot += v
        print(f'{nm:16s} {v:6.1f} cm3  bbox {[round(x, 1) for x in p["solid"].bounding_box()]}  genus {p["solid"].genus()}')
    print(f'total {tot:.1f} cm3')
