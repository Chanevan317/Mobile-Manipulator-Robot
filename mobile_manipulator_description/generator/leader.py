"""Leader arm v3 (2026-11-11): a 0.7-scale copy of the follower's kinematics, built from light leader-only parts.
v1 (AS5600 encoders) and v2 (hanging servo stack) are in git.

User decisions (2026-11-11): close to the follower ("easier teleop"), bigger is fine (0.5 .. 0.7 x), new lighter parts
(no bearings, no inserts: the servo's output horn + idler carry each joint, like the SO-100 leader), one arm module
for BOTH followers with only the base swapped (mono: one pedestal, shoulder +-180; two-arm prototype: two pedestals at
the follower's mount yaw +-135, a column stand-in between), used SEATED with TWO hands (user, after the SO-100: one
hand on the gripper, the other on a handle on the forearm, which steers the elbow, the one joint the hand alone leaves
ambiguous; later: both hands go on the gripper, see below); the gripper is a PINCH gripper (user: thumb on one side, 2+ fingers on the other, scissor-like, no gears). Servos: STS3215 low-gear (C046
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
  gripper   on the roll horn: a frame holding the gripper servo (shaft along Z, horn down, body forward) + a FIXED jaw
            (+Y) bolted under it, and a moving jaw on the servo horn (-Y), like pliers (one shared free pivot would let
            the pair swing unmeasured); the same oval finger loop on both jaws (thumb / 2-3 fingers, either hand);
            loop centres ~40 mm apart closed, ~85 mm at 40 deg. Second hand: on the gripper too (as on the SO-100);
            the forearm handle was dropped (holding the gripper's position AND heading fixes the planar arm's shape)
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
BOSS = 8.0                              # idler-side prong boss height (2026-11-12: was 2.2, which sat on the cable sockets;
                                        # now the plug (3.5 mm) + its wires' bend (5 mm) fit under the idler prong)
BOSS_R = 9.7                            # boss radius: inside the plugs (>= 10 mm from the axis), M3 shanks at r 7
# screws = the SO-100's own set (user, 2026-11-17: "we use the same screws as the so100 arm"): M2 x 6 self-tap into the
# servo cases, M3 x 6 into the horns. SO-100 horn plates leave 3.4 mm of plastic under the head (measured from its print
# files); here 3.5 (HORN_SCREW - HORN_ENGAGE), the head sits in a dia 6.2 pocket down to that seat.
HORN_SCREW, HORN_ENGAGE = 6, 2.5        # M3 length / thread into the horn disc (2.8 mm thick)
IDL_SCREW = HORN_SCREW                  # idler horn too: the head sits 3.5 mm over the disc, deep in the tall boss
                                        # (boss section 609 -> 404 mm3, ~0.9 MPa at a 10 N push on the grip: SF ~16)
# cable plug on every servo's socket face (+Z servo frame, x PLUG['x'], |y| <= PLUG['y']): housing sticks out 3.5 mm,
# then the wires need ~5 mm before they can bend sideways (both modelled for the checks)
PLUG_ENV = dict(housing=3.5, wires=5.0)
PR_B = (-OUT_F - PT, -OUT_F)            # bottom prong (output horn) z
PR_T = (IDL_F + BOSS, IDL_F + BOSS + PT)  # top prong (idler) z
WEB = (41.0, 46.0)                      # fork web, beyond the case's rear corners (r 37.3)
W = 10.0                                # fork arm / web / beam half width
BEAM_Z = 15.0                           # beam half height
CL = dict(x=(-37.5, -18.0), y=14.5, z=17.9)   # clamp on a servo's rear end (servo frame)
CLAMP_LEAD = 0.8                        # lead-in chamfer at each sleeve's mouth
ROOF_PLAY = 0.5                         # on the side that prints as a bridge (sags into the pocket; user: part 02 never let the servo in)
CLAMP_PLAY = 0.2                        # play around the case in every clamp sleeve, each side (printed sleeves come out undersize)
XP = 40.0 * SCALE                       # 28: pitch axis forward of the wrist-pan axis (follower 40)
ROOF = 2.8
# v4: S2 and S3 are turned over (output UP, sockets DOWN) so their cable loops sit UNDER the joints: the wrist's bottom
# prong is now the raised idler prong (IDL_F + BOSS + PT = 33.7 under the arm plane, was the output prong at 26.7)
ZPD = IDL_F + BOSS + PT + ROOF + 35.2 - 2.0    # 69.7 (v3 62.7): pitch axis below the arm plane (the servo needs room)
FLIP = Rx(180)                          # S2 / S3 turned over; link 2 and the wrist fork are built upright and turned with them
XR = 85.0 * SCALE                       # 59.5: roll servo centre forward of the pitch axis (follower 85)
# pinch gripper (grip frame: origin on the roll horn face, X forward along the roll axis, Z up at roll 0, Y left):
# the gripper servo S6 lies with its shaft along Z at x = PX, output horn DOWN, its body FORWARD (so the pivot sits
# right behind the roll horn and the finger loops get a long lever); one jaw is fixed (+Y), the other turns on S6's
# horn (-Y); both carry the same finger loop (thumb in one, 2-3 fingers in the other: either hand). Closed, the blades
# almost touch (the closed stop); open 40 deg the loop centres are ~85 mm apart (2026-11-11 ergonomics recheck: the
# first layout only went 58 -> 75 mm).
PX = 15.0
JAW = dict(g=2.0, t=3.0, z=(-52.0, -18.7))                 # closed gap at the blades, blade thickness, blade z
LOOP = dict(xc=85.0, L=58.0, w=24.0, wall=3.0)              # oval finger loop (inside 58 x 24: 3 fingers or a thumb)
CLAMP_YP = 21.5                                             # the clamp's +Y wall is thick: nut traps for the fixed jaw
# v4 (2026-11-17, user: "keep all cables to 20 cm", like the SO-100): the shoulder servo S1 TURNS WITH LINK 1. Its output
# horn faces down and is screwed to a fixed hub on the pedestal; its body (sockets UP) sits in link 1's rear clamp at the
# arm plane, rear end toward the elbow. No servo cable crosses the shoulder: S1 -> S2 runs along link 1 (~9 cm) and the
# servo board rides on link 1 (a dovetail socket on a short tail behind S1), so only the board's USB + power cables
# cross the shoulder (slack loop down to the desk). v3 (S1 fixed, horn up, ~31 / 36 cm shoulder cables) is in git.
ZC1 = 0.0                               # S1 centre: in the arm plane (like S2 / S3)
H_ARM = 287.0                           # arm plane above the desk (v4 +7 with ZPD: the hanging gripper keeps ~22 mm over the desk)
MOUNT_R, MOUNT_Z = 38.0, -46.6          # hub flange radius / underside = the pedestal top (unchanged)
HUB = dict(plate=(-OUT_F - PT, -OUT_F), r_plate=14.0, tube=(11.0, 17.0), flange_t=8.0)   # horn plate z, tube radii
PROTO_Y, PROTO_YAW = 150.0 * SCALE, 135.0
PFY = 5.5                               # pitch fork: idler-side prong moved out (S4's plug + the pitch cable loop)
# pedestal yaw in the world (its 3 M5 holes must match the foot's); the shoulder mount turns on it in 45 deg steps.
# Its +X side carries the board socket and the cable window. Chosen from a 2000-pose study (leader_cables.py idea; the
# arm touches a board on any side in ~1 % of random poses, all with the hand folded down beside the pedestal): mono =
# in FRONT of the arm (the follower's hand never goes down right in front of its column / scene camera; behind it is the
# tote), proto = 180 deg (1.0 % vs 1.7 % at 0 deg; 45 deg off each arm's front).
MONO_FOOT_YAW = 0.0                     # foot M5 hole pattern (mono); proto feet: each pedestal turned with its arm (+-PROTO_YAW)
# servo bus board: the SO-101 Waveshare mounting plate (51 x 42 x 4, 18 mm dovetail peg, as on the follower) slides DOWN
# into a dovetail socket on a pad on the pedestal's +X side; the board on it is ASSUMED <= 16 mm tall
# v4.1 (2026-11-17, user tested it on the printed parts): the board is back on the PEDESTAL, right under the cap at the
# DEAD END (arm frame -X, under the stop rib, where link 1 never goes). The shoulder servo's cable runs over the top
# and down the rib's outer face onto the board. Pedestal frame (= the arm root's frame, the pedestal turns with it):
# pad face at x = face_x (looks -X, just outside the cap r 38), plate slides in SIDEWAYS (the hub ring above blocks a
# top entry), from -Y, against a closed end at +Y; top of the plate just under the cap.
BOARD = dict(face_x=-39.0, pad_in=-23.0, pad_y=27.5, top=MOUNT_Z - 10.0 - 10.5, stop_y=9.3, plate=(51.0, 42.0, 4.0), comp=16.0)
                                        # top: 10 mm under the cable guide = room for the plug on the board's top edge
# closed square loop under the rib (user: the board's cable must stay still, not follow the turning arm): the cable on
# the rib's outer face (x -54.1..-51.5) passes through it; the opening (11 x 6.5) just lets the 3-pin plug through once
CABLE_GUIDE = dict(x=(-56.0, -49.5), hw=5.5, wall=2.0, z=(MOUNT_Z - 10.0, MOUNT_Z - 0.5))
CABLE = dict(w=5.0, t=2.6)              # flat 3-wire servo cable allowance (width, thickness) for clips and corridors
# shoulder support ring + end stop (2026-11-12): the arm (~435 g, COM 170-215 mm out) hung on S1's horn alone. A fixed
# ring on the shoulder mount sits SUP_GAP under link 1's beam and carries it once it droops or is pushed down; a peg under
# link 1 (outside the ring) hits a rib on the ring at the back (-X), so J1 stops at about +-174 deg and never reaches the
# servo's +-180 wrap (the follower would swing the long way round) and the cables can't wind up.
SUP = dict(ri=44.0, ro=47.5, gap=0.3)    # ring radii (3.5 mm wall) (outside the mount walls' corners r 41.6, inside link 1's clamp x 53.5)
PEG = dict(r=(48.5, 53.6), hw=3.0, z=-24.0)      # peg under link 1's beam: radial extent, half width, bottom z
RIB = dict(ro=51.5, hw=2.5)                      # stop rib on the ring's outside at -X, from the bed up to the ring top
# v4: J2 +-120 (follower elbow +-115), roll +-90 (user: "in most scenario 90 deg is enough"; shorter cable loops)
LIMITS = {'J1': (-173, 173), 'J2': (-120, 120), 'J3': (-127, 127), 'pitch': (-20, 120), 'roll': (-90, 90)}
LIMITS_PROTO_J1 = (-120, 120)
TRIG = (0.0, 40.0)                      # jaw opening (deg): thumb-finger span ~46 -> ~85 mm; maps to the follower's 0..90 mm
PLUG = dict(x=(-18.5, -10.0), y=9.6)

S1_IN_LINK1 = T(0, 0, ZC1) @ Rz(180)                                       # output DOWN (to the hub), sockets up, rear toward +x
S4_IN_WRIST = T(XP, 0, -ZPD) @ basis((0, 0, -1), (1, 0, 0), (0, -1, 0))   # shaft +-Y, output +Y, body up
S4_ROT = basis((0, 0, -1), (1, 0, 0), (0, -1, 0))
S5_IN_PITCH = T(XR, 0, 0) @ basis((0, 0, 1), (0, 1, 0), (-1, 0, 0))       # shaft X, output +X, body down
S5_IN_GRIP = T(-OUT_F, 0, 0) @ basis((0, 0, 1), (0, 1, 0), (-1, 0, 0))
S6_IN_GRIP = T(PX, 0, 0) @ basis((-1, 0, 0), (0, -1, 0), (0, 0, 1))         # shaft Z, output DOWN, body FORWARD (+X)

_cache = {}


def clamp_pocket(roof=None):
    """Space the servo's rear end needs in a clamp block it slides into along -X (servo frame).
    Line-to-line around the case, plus the follower's SO100['cz_idler'] play (0.1 mm) over the -Z case face (2026-11-16,
    user: same fit as the follower's tongue pockets; the clamp_screws seats already assume that gap)."""
    key = ('clamp', roof)
    if key not in _cache:
        M = basis((0, 0, 1), (0, 1, 0), (-1, 0, 0))
        sw = xf(sweep(xf(SERVO, M), +1, -42.0, -18.0, c=0.0, cz=0.0), np.linalg.inv(M))
        c, r = CLAMP_PLAY, ROOF_PLAY       # 2026-11-18, user: the printed sleeves were too tight
        sh = {'+y': (c, 0), '-y': (-c, 0), '+z': (0, c), '-z': (0, -SO100['cz_idler'] - c)}
        if roof:                           # roof = the servo-frame side that is UP when the part prints
            sh[roof] = {'+y': (r, 0), '-y': (-r, 0), '+z': (0, r), '-z': (0, -max(r, SO100['cz_idler'] + c))}[roof]
        _cache[key] = union([xf(sw, T(0, dy, dz)) for dy, dz in sh.values()] + [clamp_lead_in()])
    return _cache[key]


def clamp_lead_in():
    """0.8 mm x 45 deg chamfer around the clamp sleeve's mouth (servo x -18), so the case finds its way in."""
    from lib import M as _M
    hy, zt, zb, ch = 12.35 + CLAMP_PLAY, RIM + CLAMP_PLAY, -RIM - SO100['cz_idler'] - CLAMP_PLAY, CLAMP_LEAD
    pts = [(x, y * (hy + e), z + (e if z > 0 else -e)) for x, e in ((-18.0 - ch, 0.0), (-17.9, ch + 0.1))
           for y in (-1, 1) for z in (zt, zb)]
    return _M.hull_points(pts)


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
        c = CLAMP_PLAY                     # 2026-11-18: same play as the clamps (the recess was line-to-line)
        rel = union([xf(_pocket(+1), T(dx, dy, 0)) for dx, dy in ((-c, 0), (c, 0), (0, -ROOF_PLAY), (0, c))])   # -y = up in print
        cut = [rel, slot, cyl(CTR_D / 2, 14.0, 40.0, hole=True)]
        for (x, y) in [(-32.8, 10.25), (-32.8, -10.25), (-8.3, 10.25), (-8.3, -10.25)]:
            cut.append(case_screw((x, y, RIM + S['wall']), (0, 0, -1), None, cb=30.0))
        _cache['seat'] = union(cut)
    return _cache['seat']


def horn_cut(face, thick, screw_len, side=-1):
    """Cut (servo frame) for a plate on a disc: side -1 = output (face -OUT_F), +1 = idler (+IDL_F); 4 x M3 on 9.9 mm."""
    z_face = side * face
    seat = z_face + side * (screw_len - HORN_ENGAGE)
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


SERVOS = [('S1_shoulder', 'link1', S1_IN_LINK1), ('S2_elbow', 'link1', T(L1, 0, 0) @ FLIP), ('S3_wrist_pan', 'link2', T(L2, 0, 0) @ FLIP),
          ('S4_pitch', 'wrist', S4_IN_WRIST), ('S5_roll', 'pitch', S5_IN_PITCH), ('S6_gripper', 'grip', S6_IN_GRIP)]

def clip_z(c):
    """Cable clip on a face normal to +-Y: the cable runs along Z through a tunnel (w along X, t along Y).
    c = dict(x, y_face, sy (+1: face looks +Y), z=(z0, z1))."""
    w, t, wall = CABLE['w'], CABLE['t'], 1.2
    y0 = c['y_face']; y1 = y0 + c['sy'] * (t + 2 * wall)
    blk = box(c['x'] - w / 2 - wall, c['x'] + w / 2 + wall, min(y0, y1), max(y0, y1), *c['z'])
    yt0, yt1 = y0 + c['sy'] * wall * 0.0, y0 + c['sy'] * (t + wall * 0.0)
    tun = box(c['x'] - w / 2, c['x'] + w / 2, min(yt0, yt1) - 0.01 * c['sy'], max(yt0, yt1), c['z'][0] - 1, c['z'][1] + 1)
    return blk - tun


def arch_x(x0, x1, z_face, y=0.0):
    """Cable arch on a face normal to +Z (top at z_face): tunnel along X, w along Y, t along Z."""
    w, t, wall = CABLE['w'] + 4.0, CABLE['t'] + 0.6, 1.4
    blk = box(x0, x1, y - w / 2 - wall, y + w / 2 + wall, z_face, z_face + t + wall)
    return blk - box(x0 - 1, x1 + 1, y - w / 2, y + w / 2, z_face - 0.01, z_face + t)


def clip_x(y0_, y1_, x_face, sx, z):
    """Cable clip on a face normal to +-X (at x_face): the cable runs along Z (t along X, w along Y)."""
    w, t, wall = CABLE['w'], CABLE['t'], 1.2
    x1_ = x_face + sx * (t + wall)
    blk = box(min(x_face, x1_), max(x_face, x1_), y0_ - wall, y1_ + wall, *z)
    tx1 = x_face + sx * t
    return blk - box(min(x_face, tx1) - 0.01, max(x_face, tx1) + 0.01, y0_, y1_, z[0] - 1, z[1] + 1)



WRIST_CLIP = dict(x=20.0, y_face=-CL['z'], sy=-1, z=(-39.2, -33.8))      # wrist: clip on the S4 clamp's -Y face for the drop
                                                                         # (above the pitch loop: >= 30 mm from the pitch axis)
ROLL_NOTCH = dict(hw=CABLE['w'] / 2 + 0.7, z0=12.5)                     # gripper frame back plate: cable notch (grip frame)
LINK1_CLIP = dict(x=L1 - 34.0, y_face=CL['y'], sy=+1, z=(-6.0, 1.0))     # link 1: S1 -> S2 cable down the elbow clamp's +Y face
                                                                         # (x 57: outside the support ring and the stop peg)
CABLE_EXIT = -60.0                       # elbow / wrist pan: the cable leaves the gap under the top prong at this angle

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
             cyl(13.0, *PR_T), cyl(BOSS_R, IDL_F, PR_T[0] + 0.01), box(0, WEB[1], -W, W, *PR_T),   # top prong on the idler
             box(WEB[0], WEB[1], -W, W, PR_B[0], PR_T[1])]                       # web
    cut = [horn_cut(OUT_F, PT, HORN_SCREW), horn_cut(IDL_F, PT + BOSS, IDL_SCREW, side=+1)]
    if beam_to is not None:
        parts += [box(WEB[1] - 0.01, beam_to + CL['x'][0] + 0.01, -W, W, -BEAM_Z, BEAM_Z)] + clamp_block(beam_to)
        cut += clamp_cut(beam_to)
    return parts, cut


def clamp_block(x):
    return [box(x + CL['x'][0], x + CL['x'][1], -CL['y'], CL['y'], -CL['z'], CL['z'])]


def plug_room():
    """Room for the cable plug over the servo's socket face (servo frame): the clamps on a servo's rear end reached
    0.5 mm over the sockets (their edge would block pushing the plug in)."""
    return box(PLUG['x'][0] - 0.2, PLUG['x'][1] + 0.2, -PLUG['y'] - 0.2, PLUG['y'] + 0.2, RIM - 0.01, RIM + 12.0)


def clamp_cut(x, flip=False, roof='-y'):
    M_ = T(x, 0, 0) @ (FLIP if flip else np.eye(4))
    return [xf(clamp_pocket(roof), M_), xf(clamp_screws(), M_), xf(plug_room(), M_)]


def build():
    PARTS.clear(); BASE_PARTS.clear(); FASTENERS.clear()

    # ---- shoulder hub (part 01, fixed): plate under S1's output horn (4 x M3 x 6 UP into the horn, reached through the
    # hollow tube before the hub goes on the pedestal) + tube + flange with 4 x M3 down into the pedestal cap (any multiple
    # of 45 deg) + the support ring under link 1's beam + the end-stop rib at the back (-X)
    H = HUB; ring_top = -BEAM_Z - SUP['gap']                                             # link 1's beam underside - gap
    plate = cyl(H['r_plate'], *H['plate'])
    tube = cyl(H['tube'][1], MOUNT_Z + H['flange_t'] - 0.01, H['plate'][0] + 0.01) - cyl(H['tube'][0], MOUNT_Z, H['plate'][0] + 0.02)
    flange = cyl(MOUNT_R, MOUNT_Z, MOUNT_Z + H['flange_t'])
    ring = union([cyl(SUP['ro'], MOUNT_Z, MOUNT_Z + H['flange_t']), cyl(SUP['ro'], MOUNT_Z + H['flange_t'] - 0.01, ring_top)]) \
        - cyl(SUP['ri'], MOUNT_Z + H['flange_t'], ring_top + 1)
    rib = box(-RIB['ro'], -SUP['ri'], -RIB['hw'], RIB['hw'], MOUNT_Z, ring_top)
    hub = union([plate, tube, flange, ring, rib]) - union(
        [xf(horn_cut(OUT_F, PT, HORN_SCREW), S1_IN_LINK1), cyl(H['tube'][0], MOUNT_Z - 1, H['plate'][0] + 0.01)] +
        [cyl(1.7, MOUNT_Z - 1, MOUNT_Z + H['flange_t'] + 1, 30 * math.cos(math.radians(a)), 30 * math.sin(math.radians(a)), hole=True)
         for a in (45, 135, 225, 315)])
    add('shoulder_hub', hub, 'base', (0.85, 0.45, 0.15), 'fixed hub under the shoulder servo: its output horn screws on '
        'from below (4 x M3 x 6 through the hollow tube); 4 x M3 down into the pedestal (any multiple of 45 deg); '
        'support ring under link 1 + stop rib at the back')
    fast('SHCS', 'M3', HORN_SCREW, 4, 'shoulder horn onto the hub (from below, through the tube)')
    fast('SHCS', 'M3', 20, 4, 'shoulder hub to pedestal (nut in the cap underside)'); fast('nut', 'M3', None, 4, 'shoulder hub')

    # ---- link 1 (part 02): clamp on S1's rear end + beam + clamp on S2's rear end (v4.1: no board tail any more)
    peg = box(PEG['r'][0], PEG['r'][1], -PEG['hw'], PEG['hw'], PEG['z'], -BEAM_Z + 0.01)       # end stop peg under the beam
    s1_clamp = xf(box(CL['x'][0], CL['x'][1], -CL['y'], CL['y'], -CL['z'], CL['z']), S1_IN_LINK1)   # x 18 .. 37.5
    beam = box(-CL['x'][0] - 0.01, L1 + CL['x'][0] + 0.01, -W, W, -BEAM_Z, BEAM_Z)
    # S1 -> S2 cable: S1's sockets are on top, S2's (turned over) underneath: it runs along the top and down a clip on the
    # beam's +Y face, then under the elbow clamp
    l1 = union([s1_clamp, beam, peg, clip_z(LINK1_CLIP)] + clamp_block(L1))
    s1_cut = [xf(clamp_pocket('-y'), S1_IN_LINK1), xf(clamp_screws(), S1_IN_LINK1), xf(plug_room(), S1_IN_LINK1)]
    l1 = l1 - union(s1_cut + clamp_cut(L1, flip=True))
    add('link1', l1, 'link1', (0.30, 0.55, 0.85), f'upper arm {L1:.0f} mm: clamps on the shoulder and elbow servos\' rear '
        'ends, rides on the support ring (stop peg underneath)')
    fast('selftap', 'M2', 6, 4, 'shoulder servo case (link 1 clamp)'); fast('selftap', 'M2', 6, 4, 'elbow servo case (link 1 clamp)')

    # ---- link 2: fork around S2 + beam + clamp on S3 (+ an arch on the clamp top for the S2 -> S3 cable)
    p, c = fork(beam_to=L2)
    p += [arch_x(L2 + CL['x'][0] + 0.5, L2 + CL['x'][0] + 6.5, CL['z'])]   # on the clamp face (after the flip: underneath); at its
    # far end, r >= 31 from the wrist-pan axis: closer in, the wrist's side cable met it past J3 -120
    add('link2', xf(union(p) - union(c), FLIP), 'link2', (0.30, 0.55, 0.85),
        f'forearm {L2:.0f} mm: fork on the elbow servo (horn + idler), clamp on the wrist-pan servo; prints on its side')
    fast('SHCS', 'M3', HORN_SCREW, 4, 'elbow horn'); fast('SHCS', 'M3', IDL_SCREW, 4, 'elbow idler (head deep in the boss)'); fast('selftap', 'M2', 6, 4, 'wrist-pan servo case (link 2 clamp)')

    # ---- wrist: fork around S3 + clamp under the bottom prong holding S4's rear (top) end
    p, c = fork()
    fk = xf(union(p) - union(c), FLIP)                                                   # turned over with S3: idler prong at the bottom
    prb = (-PR_T[1], -PR_T[0])                                                            # that bottom prong's z (-33.7 .. -25.7)
    blk = box(XP - CL['y'], XP + CL['y'], -CL['z'], CL['z'], -ZPD + 18.0, prb[0] + 0.01)   # S4 clamp (S4 frame: y_s -> x)
    plate = box(0, WEB[1], -CL['z'], CL['z'], *prb) - xf(horn_cut(IDL_F, PT + BOSS, IDL_SCREW, side=+1), FLIP)
    wr = union([fk, blk, plate, clip_z(WRIST_CLIP)])   # v4: S3 -> S4 cable leaves the gap under S3 at CABLE_EXIT, drops down the -Y side
    wr = wr - union([xf(clamp_pocket('-z'), S4_IN_WRIST), xf(clamp_screws(), S4_IN_WRIST), xf(plug_room(), S4_IN_WRIST)])
    add('wrist', wr, 'wrist', (0.93, 0.70, 0.25),
        'fork on the wrist-pan servo (horn + idler) + clamp holding the pitch servo')
    fast('SHCS', 'M3', HORN_SCREW, 4, 'wrist-pan horn'); fast('SHCS', 'M3', IDL_SCREW, 4, 'wrist-pan idler (head deep in the boss)'); fast('selftap', 'M2', 6, 4, 'pitch servo case (wrist clamp)')

    # ---- pitch fork (frame: origin on the pitch axis, X forward, Y = axis): prongs on S4's discs + back plate for S5
    xb1 = XR - RIM; xb0 = xb1 - 4.0
    pr = []
    # the idler side (-Y, S4's socket face) sits PFY further out so S4's plug and the pitch cable loop fit inside it
    for sgn, face, off, br in ((+1, OUT_F, 0.0, 11.0), (-1, IDL_F, PFY, BOSS_R)):
        y0, y1 = sorted((sgn * (19.2 + off), sgn * (25.2 + off)))
        b0, b1 = sorted((sgn * face, sgn * (19.2 + off)))
        pr += [xf(cyl(13.0, y0, y1), Rx(-90)), box(0, xb1, y0, y1, -11.0, 11.0), box(12.0, xb1, y0, y1, -36.0, 11.0),
               xf(cyl(br, b0, b1), Rx(-90))]
    gus = [box(xb0 - 3.0, xb0 + 0.01, y0, y1, -36.0, 0.0) for (y0, y1) in ((13.5, 16.5), (-16.5, -13.5))]
    pf = union(pr + [box(xb0, xb1, -(25.2 + PFY), 25.2, -36.0, 12.0)] + gus)
    pf = pf - union([xf(horn_cut(OUT_F, 6.5, HORN_SCREW), S4_ROT), xf(horn_cut(IDL_F, 7.5 + PFY, IDL_SCREW, side=+1), S4_ROT), xf(seat_cut(), S5_IN_PITCH)])
    add('pitch_fork', pf, 'pitch', (0.80, 0.35, 0.35), 'pitch fork (output horn + idler) + back plate holding the roll servo')
    fast('SHCS', 'M3', HORN_SCREW, 4, 'pitch horn'); fast('SHCS', 'M3', IDL_SCREW, 4, 'pitch idler (head deep in the boss)'); fast('selftap', 'M2', 6, 4, 'roll servo case')

    # ---- pinch gripper: frame (back plate on the roll horn + top plate + clamp on S6's front end), fixed jaw, moving jaw
    J, Lp = JAW, LOOP
    yi, yo = J['g'] / 2, J['g'] / 2 + J['t']                       # blade inner / outer face |y| (closed)
    def loop(sy, x0=0.0):
        """Oval finger loop on a blade's outer face (sy = +1: +Y side), full jaw height; in its part's frame."""
        yc = sy * (yo + Lp['wall'] + Lp['w'] / 2)
        def stadium(e, z0, z1):
            r = Lp['w'] / 2 + e; xa, xb = Lp['xc'] - x0 - Lp['L'] / 2 + Lp['w'] / 2, Lp['xc'] - x0 + Lp['L'] / 2 - Lp['w'] / 2
            return union([box(xa, xb, yc - r, yc + r, z0, z1), cyl(r, z0, z1, xa, yc), cyl(r, z0, z1, xb, yc)])
        return stadium(Lp['wall'], *J['z']) - stadium(0.0, J['z'][0] - 1, J['z'][1] + 1)
    x_tip = Lp['xc'] + Lp['L'] / 2 + Lp['wall']
    cx = (PX - CL['x'][1], PX - CL['x'][0])                        # clamp x (servo frame x -> -x): 33 .. 52.5
    back = box(0, 4.0, -16.0, 16.0, -14.4, 24.0)
    top = box(0, cx[1], -CL['y'], CLAMP_YP, 21.0, 24.0)
    clamp = box(cx[0], cx[1], -CL['y'], CLAMP_YP, -CL['z'], 24.0)
    fr = union([back, top, clamp, arch_x(3.0, 9.0, 24.0)])                     # arch: S5 -> S6 cable along the top
    S = SO100
    # S6 slides in SIDEWAYS from -Y (2026-11-17): the closed clamp let it move in no direction, so it could only go in before
    # the frame was on the roll horn, and then S6 covered the frame's 4 horn screws. Order now: frame onto the roll horn,
    # then S6 in from -Y, then its 3 case screws. Cuts: the -Y wall over the case height; a groove in the top wall for the
    # case's raised top panel (servo x -29.9..-18, |y| <= 9.2, 1.8 mm high); the bottom wall's -Y quarter is open for the
    # bottom panel (x -34.8..-18, |y| <= 7, 1.0 mm): a groove there would leave 0.7 mm under that case screw, so S6 keeps
    # 3 case screws (2 top + the bottom +Y one, reached through the fixed jaw)
    pk = xf(clamp_pocket('-z'), S6_IN_GRIP).bounding_box()
    side_open = union([box(pk[0] - 0.01, pk[3] + 0.01, -CL['y'] - 1.0, 0.0, -RIM - SO100['cz_idler'], RIM),
                       box(pk[0] - 0.01, PX + 29.9 + 0.3, -CL['y'] - 1.0, -9.2 + 0.2, RIM - 0.01, RIM + 1.8 + 0.2),
                       box(pk[0] - 0.01, pk[3] + 0.01, -CL['y'] - 1.0, -7.0 + 0.2, -CL['z'] - 1.0, -RIM - 0.05)])
    # v4 roll cable: from over the roll servo's top it enters the gripper through a NOTCH in the back plate's top, close to
    # the roll axis (it twists +-90 deg around S5's top there instead of wrapping a loop around the gripper's back)
    notch = box(-0.1, 4.1, -ROLL_NOTCH['hw'], ROLL_NOTCH['hw'], ROLL_NOTCH['z0'], 24.1)
    cut = [xf(horn_cut(OUT_F, 4.0, HORN_SCREW), S5_IN_GRIP), xf(clamp_pocket('-z'), S6_IN_GRIP), side_open, notch, xf(clamp_screws(), S6_IN_GRIP),
           cyl(4.0, 20.0, 25.0, PX, 0, hole=True),                                               # over the idler / centre screw
           box(PX - PLUG['x'][1] - 0.5, PX - PLUG['x'][0] + 0.5, -PLUG['y'], PLUG['y'], 15.0, 25.0)]   # plug slot (socket face up; S6's x -> -X)
    nuts = [(cx[0] + 5.0, 17.5), (cx[1] - 5.0, 17.5)]
    for (x, y) in nuts:                                                                          # fixed-jaw bolts: nut traps in the thick +Y wall
        cut += [cyl(1.7, -CL['z'] - 1, 6.0, x, y, hole=True), nut_cut((x, y, -2.0), (0, 0, 1), 'M3', depth=3.0),
                box(x - 3.0, x + 3.0, y, CLAMP_YP + 1, -2.0, 1.0)]                               # nut slides in from +Y
    add('gripper_frame', fr - union(cut), 'grip', (0.25, 0.25, 0.28),
        'pinch gripper frame on the roll horn: holds the gripper servo (horn down, body forward); prints top plate down')
    fast('SHCS', 'M3', HORN_SCREW, 4, 'roll horn (gripper frame back plate)'); fast('selftap', 'M2', 6, 3, 'gripper servo case (2 top + 1 bottom through the fixed jaw)')
    # fixed jaw (+Y): block under the clamp's thick wall + blade + loop; 2 x M3 up into the frame's nut traps
    blk = box(cx[0], cx[1], yi, CLAMP_YP, J['z'][0], -CL['z'])
    fj = union([blk, box(cx[0], x_tip, yi, yo, J['z'][0], -CL['z']), loop(+1)])
    fcut = [cyl(1.7, J['z'][0] - 1, -CL['z'] + 1, x, y, hole=True) for (x, y) in nuts]
    fcut += [cyl(3.2, J['z'][0] - 1, -CL['z'] - 6.0, x, y, hole=True) for (x, y) in nuts]       # heads recessed from below
    for (x, y) in S['idler']:                                                                    # driver access: S6's -Z face case screws (face the jaw)
        px_, py_ = PX - x, -y
        if py_ > yi: fcut.append(cyl(2.6, J['z'][0] - 1, -CL['z'] + 0.5, px_, py_, hole=True))
    add('gripper_fixed_jaw', fj - union(fcut), 'grip', (0.20, 0.20, 0.22),
        'fixed jaw with its finger loop (+Y), bolted under the gripper frame (2 x M3); prints standing on its loop')
    fast('SHCS', 'M3', 30, 2, 'fixed jaw to the gripper frame (nuts in the frame)'); fast('nut', 'M3', None, 2, 'fixed jaw')
    # moving jaw (frame: on S6's axis): plate on the output horn + blade (-Y) + loop
    # below the horn plate the blade starts at x 8.4, past the (+4.95, -4.95) horn screw's head pocket (r 3.1): it
    # covered that screw (2026-11-17); in the plate band it still runs from x 0
    mj = union([cyl(11.0, -OUT_F - PT, -OUT_F), box(-6.0, 6.0, -yo, 0, -OUT_F - PT, -OUT_F),
                box(0, x_tip - PX, -yo, -yi, -OUT_F - PT, -OUT_F), box(8.4, x_tip - PX, -yo, -yi, J['z'][0], -OUT_F - PT + 0.01),
                loop(-1, PX)])
    mj = mj - xf(horn_cut(OUT_F, PT, HORN_SCREW), basis((-1, 0, 0), (0, -1, 0), (0, 0, 1)))
    add('gripper_jaw', mj, 'jaw', (0.20, 0.20, 0.22), 'moving jaw on the gripper servo horn, with its finger loop (-Y); '
        'opens 0..40 deg; prints upside down (horn plate on the bed)')
    fast('SHCS', 'M3', HORN_SCREW, 4, 'gripper servo horn')

    build_bases()


# ------------------------------------------------------------------ bases (swap: 4 x M3 under the shoulder mount)
def pedestal():
    """Slim hollow column (r 26) from the desk to a cap (r 38) under the shoulder hub (base frame of one arm). The cap
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
    # v4.1 board socket at the dead end (-X): pad from the column out past the cap, dovetail along Y (plate slides in
    # from -Y to a closed end at +Y), plate top just under the cap; a U-slot guide under the rib holds the cable that
    # comes down the rib's outer face and drops it onto the board's top edge
    B = BOARD; zt_b = B['top']; zc = zt_b - B['plate'][1] / 2
    pad = box(B['face_x'], B['pad_in'], -B['pad_y'], B['pad_y'], zt_b - B['plate'][1] - 2.0, zt - ct + 0.01)   # up to the cap
    dv = CS([[(-1.0, -8.15), (1.0, -8.15), (2.6, -9.25), (4.0, -9.25), (4.0, 9.25), (2.6, 9.25), (1.0, 8.15), (-1.0, 8.15)]])
    # profile in (x out of the face, z); the socket runs along Y from the open end (-Y) to 2 mm short of the +Y end
    # profile u = into the pad (+X), v = Z; extruded along local z = world -Y: from the closed end y = stop_y (the
    # plate's peg then sits centred, y = 0) out through the open -Y end
    dv = xf(dv.extrude(B['pad_y'] + 1.0 + B['stop_y']).translate((0, 0, -B['stop_y'])),
            basis((1, 0, 0), (0, 0, 1), (0, -1, 0), (B['face_x'], 0, zc)))
    G = CABLE_GUIDE
    # the loop's arm runs into the cap (r 38) so it prints as one piece with the pedestal
    guide = box(G['x'][0] - G['wall'], -30.0, -G['hw'] - G['wall'], G['hw'] + G['wall'], *G['z']) \
        - box(G['x'][0], G['x'][1], -G['hw'], G['hw'], G['z'][0] - 1, G['z'][1] + 1)     # closed all round
    return (s + pad + guide) - union(cut + [dv])


def build_bases():
    ped = pedestal()
    BASE_PARTS['pedestal'] = dict(solid=ped, variant='both', qty_mono=1, qty_proto=2, color=(0.55, 0.57, 0.60),
                                  note='column under the shoulder hub; 8 nut traps; 3 x M5 to a foot; board socket + cable guide at the back under the cap')
    # one pedestal per arm module; grip = pedestal flange 6 + foot plate 6 = 12 mm, nut in a pocket under the foot
    fast('SHCS', 'M5', 12, 3, 'pedestal to foot (nut in the pocket under the foot)'); fast('nut', 'M5', None, 3, 'pedestal to foot')
    zd = -H_ARM
    def holes(cx, cy, yaw):
        """3 x M5 under a pedestal, each with a hex nut pocket (M5 nut 8 AF x 4) open on the foot's underside, so the
        foot sits flat on the desk; an M5 x 12 from the flange top (6 + 6 mm) ends flush with the bottom."""
        out = []
        for k in range(3):
            x, y = cx + 44 * math.cos(math.radians(120 * k + yaw)), cy + 44 * math.sin(math.radians(120 * k + yaw))
            out += [cyl(2.75, zd - 7.0, zd + 1, x, y, hole=True), nut_cut((x, y, zd - 6.01), (0, 0, 1), 'M5', depth=4.5, af=8.3)]
        return out
    mono = box(-110.0, 110.0, -85.0, 85.0, zd - 6.0, zd) - union(holes(0, 0, MONO_FOOT_YAW) +
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
        h = h - union(holes(0, sy * PROTO_Y, sy * PROTO_YAW) + [box(x - 10, x + 10, sy * (PROTO_Y + 55.0) - 4, sy * (PROTO_Y + 55.0) + 4, zd - 7, zd + 1) for x in (-75.0, 75.0)]
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


def ped_frame(R0, variant):
    """The pedestal under an arm root R0 (v4.1: turned with the arm, so its board socket sits under the stop rib)."""
    return R0.copy()


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
        out.append((tag + 'pedestal', xf(BASE_PARTS['pedestal']['solid'], ped_frame(R0, variant)), BASE_PARTS['pedestal']['color'], tag + 'base'))
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
