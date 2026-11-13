"""Mono-arm shoulder ring (user 2026-11-13: layout B, no longer a draft): the prototype's own arm base (01 base_top +
02 base_bottom, with its shoulder servo) bolts onto a SHELF on the ring, outside it; one link is dropped (shoulder ->
130 mm upper arm -> wrist pan). The ring only turns the whole arm around the C-Beam. Bought bearings: 2 x 6820-2RS /
61820-2RS (100 x 125 x 13). Rotor + gear + pinion in PETG. (The earlier printed-ball / collar draft is in git history.)

Units mm. Frame = CARRIAGE: origin on the column axis, x forward (working face), z = carriage z (plate top z 20).
Lift = the real C-Beam XLarge kit (cbeam_import.py): plate x -27.5..-21.5, V-wheels x -16..0.5 / |y| 38..62, top z 20.

Stack (carriage z):
    20 .. 25   disc (fixed) on the plate top; shoulder ring r 50..54 at 25..26 under bearing 1's inner ring only
    26 .. 39   bearing 1 (61820)          inner ring on the hub (r 50), outer ring in the rotor bore (r 62.5)
    39 .. 47   inner spacer (fixed) / rotor lip between the outer rings (moving)
    47 .. 60   bearing 2 (61820)
    60 .. 64   clamp (fixed): presses both inner rings + spacer down onto the disc shoulder (hub top 59.8)
    64 .. 76   cable channel: fixed drum (clamp) r 52..56, moving tray (on the rotor top) wall r 66..70
    21 .. 60   rotor (PETG): 72 T m2 gear at 22.5..30.5, shelf at 32..44 (over the pinion, top 30.5), ribs
    84         arm plane (the base's servo mid-plane); shoulder axis 161 mm out (pillar back face at r 76)
X arrangement: the rotor's lip pushes bearing 1's outer ring down (arm weight) and bearing 2's up (lift-off);
the inner rings are clamped. 0 axial play beyond the bearings' own.

    python3 mono_ring.py [out_dir]     checks + export (default ../../CAD/mono_ring)
"""
import math, os, sys, csv, collections
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import M, CS, box, cyl, ring, union, xf, T, Rx, Ry, Rz, basis, along, print_mesh
from hw import screw_cut, insert_cut, Reg, SPEC
from joint import SERVO, HORN_HOLES, OUT_FACE, RIM, CTR_D, OUT_NUB, SO100

# ---- bought bearing: 61820 (SKF / NSK size: d 100, D 125, B 13; ring lands d1 ~107.5, D1 ~117.6)
BRG = dict(d=100.0, D=125.0, B=13.0, in_land=53.75, out_land=58.8)
B1_Z, B2_Z = 26.0, 47.0                       # bearing bottoms
HUB_R = BRG['d'] / 2 - 0.05                   # slip fit for the inner rings
BORE_R = BRG['D'] / 2 + 0.05                  # slip fit for the outer rings
HUB_TOP = B2_Z + BRG['B'] - 0.2               # the clamp squeezes the inner rings, not the hub
DISC_R, DISC_Z = 61.0, (20.0, 25.0)
BEAM = (21.0, 41.0)                           # hole half sizes: beam 40 x 80 + 1 mm each side (corner wall 3.9 mm)
LIP = (B1_Z + BRG['B'], B2_Z, 57.5)           # rotor lip between the outer rings: z0, z1, inner r
CLAMP = (60.0, 64.0, 56.0)                    # z0, z1, outer r (inside the outer ring's land: touches the inner ring only)
ROT_Z0, ROT_TOP, ROT_OUT = 21.0, 60.0, 70.0
MOD, ZR, ZP = 2.0, 72, 18
GEAR_Z = (22.5, 30.5)
PIN_A = math.radians(60.0)
PIN_C = (90.0 * math.cos(PIN_A), 90.0 * math.sin(PIN_A))
SHELF_Z = (32.0, 44.0)                        # 1.5 mm over the pinion
AZ = SHELF_Z[1] + 40.0                        # arm plane: the base's mount face is 40 mm under its servo mid-plane
L1 = 161.0                                    # shoulder axis from the column axis (pillar back x -85 -> r 76)
BASE_M5 = [(-80.0, 13.5), (-80.0, -13.5), (-54.0, 13.5), (-54.0, -13.5)]     # arm.base_parts() inserts (ARM frame)
WY = 20.5                                     # base pillar half width
CH = dict(ri=56.0, ro=66.0, floor=(60.5, 62.5), top=76.0)
CABLE_HOLE_XY = (27.0, -27.0)
CABLE_A = math.atan2(CABLE_HOLE_XY[1], CABLE_HOLE_XY[0])
TRAY_SCREWS = (100.0, 180.0, 260.0)           # tray to rotor top (vertical M3), away from the shelf / cable exit
HOME_A = 180.0                                # homing magnet in the rotor bore, hall sensor in the disc rim (behind)
SDIR = -1                                   # servo body: -1 inward (toward the column, under the disc), +1 outward
def RAD(xs): return 90.0 - SDIR * xs        # radial position of a point at servo-frame x = xs
SR = sorted((RAD(-35.2), RAD(10.2)))        # radial span of the case
EAR = dict(z=(13.0, 20.0), w=18.0, r0=48.0, r1=106.0)   # inward servo: nothing fixed beyond r ~106 (the folded wrist hangs there)
SERVO_Z = EAR['z'][0] - OUT_FACE            # drive servo centre (output face = ear underside)
HUB_BOLTS = [(-33.0, sy * 27.0) for sy in (-1, 1)]   # M5 x 40 from the hub top down into the bracket (load test 2026-11-13:
# with the arm pointing back the ring tips about the bracket's rear edge, and the 2 bolts from below at x -50 were only 10 mm
# in front of it; these two sit 28 mm in front)
BRACKET = dict(x0=-61.0, x1=-28.0, leg_x0=-40.0, top_w=40.0, leg_w=56.0)   # top under the disc; 12 mm leg on the plate
CLAMP_SCREWS = [(36.0, 20.0), (36.0, -20.0), (-38.0, 12.0), (-38.0, -12.0)]   # rear pair clear of the hub bolts' counterbores
BOARD_INSERTS = [(50.0, -12.0), (46.0, -30.0), (30.0, -48.0)]

# ---- the kit's lift parts (carriage frame) for clearance checks
GANTRY = box(-27.5, -21.5, -62.5, 62.5, -105.0, 20.0)
WHEELS = union([box(-16.0, 0.5, 38.0, 62.0, -105.0, 20.0), box(-16.0, 0.5, -62.0, -38.0, -105.0, 20.0)])
GANTRY_HOLES = [(sy * 50.0, z) for sy in (-1, 1) for z in (-12.5, -72.5)]
COLUMN = box(-20.0, 20.0, -40.0, 40.0, -300.0, 600.0)

PARTS, CONTEXT = {}, {}


def add(name, solid, moving, color, reg, note, print_rot=np.eye(4), material='PLA'):
    PARTS[name] = dict(solid=solid, moving=moving, color=color, reg=reg, note=note, print_rot=print_rot, material=material)


def bearing(z0):
    """61820 as two rings (inner fixed, outer turning) + the ball / seal band between them (belongs to neither)."""
    zi = (z0, z0 + BRG['B'])
    return dict(inner=ring(BRG['d'] / 2, BRG['in_land'], *zi), outer=ring(BRG['out_land'], BRG['D'] / 2, *zi),
                band=ring(BRG['in_land'], BRG['out_land'], z0 + 0.6, z0 + BRG['B'] - 0.6))


def beam_hole(z0=-10.0, z1=200.0):
    return box(-BEAM[0], BEAM[0], -BEAM[1], BEAM[1], z0, z1)


# ------------------------------------------------------------------ fixed -------------------------------------------
def ring_base():
    """Disc on the plate top + hub (the inner rings' seat) + shoulder under bearing 1's inner ring."""
    r = Reg()
    s = union([cyl(DISC_R, *DISC_Z), ring(HUB_R - 4.0, BRG['in_land'] - 0.25, DISC_Z[1] - 0.01, B1_Z),
               cyl(HUB_R, DISC_Z[1] - 0.01, HUB_TOP)]) - beam_hole()
    cut = []
    for (x, y) in CLAMP_SCREWS:
        cut.append(insert_cut((x, y, HUB_TOP), (0, 0, -1), 'M3', length=6)); r.add('insert', 'M3', (x, y, HUB_TOP), (0, 0, -1), 6, 'clamp')
    for (x, y) in HUB_BOLTS:                 # M5 x 40 through the hub + disc, heads sunk under the clamp
        p = (x, y, HUB_TOP - 5.5)
        cut.append(screw_cut(p, (0, 0, -1), 'M5', p[2] - DISC_Z[0] + 0.5, cb=6.0)); r.add('SHCS', 'M5', p, (0, 0, -1), 40, 'hub + disc to the L bracket (from the hub top)')
    for sy in (-1, 1):                       # L bracket: 2 x M5 inserts from below
        p = (-50.0, sy * 20.0, DISC_Z[0])
        cut.append(insert_cut(p, (0, 0, 1), 'M5', length=4)); r.add('insert', 'M5', p, (0, 0, 1), 4, 'L bracket')
    u = np.array([math.cos(PIN_A), math.sin(PIN_A), 0.0]); v = np.array([-u[1], u[0], 0.0])
    for off in (-10.0, 10.0):                # drive mount: 2 x M3 down through the disc in RADIAL slots (gear backlash)
        p = tuple(u * 56.0 + v * off + np.array([0, 0, DISC_Z[1] - 2.2]))
        cut.append(union([screw_cut(tuple(np.asarray(p) + u * t), (0, 0, -1), 'M3', 3.0, cb=3.4) for t in np.linspace(-1.5, 1.5, 7)]))
        r.add('SHCS', 'M3', p, (0, 0, -1), 8, 'drive mount (slotted: slide it to set the gear backlash)')
    cut.append(cyl(3.5, DISC_Z[0] - 1, HUB_TOP + 1, *CABLE_HOLE_XY, hole=True))
    for (x, y) in BOARD_INSERTS:
        cut.append(insert_cut((x, y, DISC_Z[0]), (0, 0, 1), 'M3', length=4)); r.add('insert', 'M3', (x, y, DISC_Z[0]), (0, 0, 1), 4, 'board mount')
    a = math.radians(HOME_A); hx, hy = DISC_R * math.cos(a), DISC_R * math.sin(a)
    cut.append(xf(box(-4.0, 0.1, -2.3, 2.3, 21.0, DISC_Z[1] - 0.8), T(hx, hy, 0) @ Rz(HOME_A)))
    cut.append(cyl(1.8, DISC_Z[0] - 1, 22.0, (DISC_R - 3.0) * math.cos(a), (DISC_R - 3.0) * math.sin(a), hole=True))
    r.add('part', 'hall sensor', (hx, hy, 23.0), (-1, 0, 0), None, 'homing: SS49E / A3144 in the disc rim')
    add('ring_base', s - union(cut), False, (0.55, 0.57, 0.60), r,
        'disc on the gantry plate + hub (seat of both 61820 inner rings) + shoulder under bearing 1; prints disc down')


def spacer():
    s = ring(HUB_R + 0.15, BRG['in_land'] - 0.25, LIP[0], LIP[1])
    add('inner_spacer', s, False, (0.55, 0.57, 0.60), Reg(), 'between the two inner rings (same height as the rotor lip); prints flat')


def clamp_ring():
    r = Reg(); z0, z1, ro = CLAMP
    s = cyl(ro, z0, z1)
    boss = box(27.0, 49.0, -12.0, 12.0, z1 - 0.01, z1 + 12.0)                      # scene-camera post socket
    drum = union([ring(CH['ri'] - 4.0, CH['ri'], z1 - 0.01, CH['top']), ring(CH['ri'] - 0.01, CH['ri'] + 3.0, CH['top'] - 2.0, CH['top'])])
    s = union([s, boss, drum]) - beam_hole() - box(30.8, 45.2, -8.2, 8.2, z1 + 2.0, z1 + 13.0)
    cut = [insert_cut((49.0, 0.0, z1 + 7.0), (-1, 0, 0), 'M3', length=4),
           cyl(3.5, z0 - 1, z1 + 1, *CABLE_HOLE_XY, hole=True),
           xf(box(30.0, CH['ri'] + 1, -4.0, 4.0, z1 - 0.01, z1 + 9.0), Rz(math.degrees(CABLE_A)))]
    r.add('insert', 'M3', (49.0, 0.0, z1 + 7.0), (-1, 0, 0), 4, 'camera post set screw')
    for (x, y) in CLAMP_SCREWS:
        p = (x, y, z1)
        cut.append(screw_cut(p, (0, 0, -1), 'M3', (z1 - z0) + 0.2 + 6.0, cb=0.0)); r.add('SHCS', 'M3', p, (0, 0, -1), 10, 'clamp to hub (squeezes the inner rings)')
    add('clamp_ring', s - union(cut), False, (0.55, 0.57, 0.60), r,
        'clamps both inner rings + spacer; fixed inner wall of the cable channel; camera post socket; prints flat')


def gantry_bracket():
    r = Reg(); B = BRACKET
    top = box(B['x0'], B['x1'], -B['top_w'], B['top_w'], 12.0, DISC_Z[0])
    leg = box(B['leg_x0'], B['x1'], -B['leg_w'], B['leg_w'], -80.0, DISC_Z[0])          # on the plate's back face (-27.5): 0.5 mm, clear of the wheel bolt heads (-27.7)
    gus = union([xf(_tri(B['x0'], B['leg_x0'], -40.0, 12.0, t=4.0), T(0, y, 0)) for y in (-B['top_w'], B['top_w'] - 4.0)])
    s = union([top, leg, gus]); cut = []
    for sy in (-1, 1):
        p = (-50.0, sy * 20.0, 12.0)
        cut.append(screw_cut(p, (0, 0, 1), 'M5', 8.0 + 4.0, cb=6.0)); r.add('SHCS', 'M5', p, (0, 0, 1), 10, 'bracket to disc (from below)')
    for (x, y) in HUB_BOLTS:
        cut.append(insert_cut((x, y, DISC_Z[0]), (0, 0, -1), 'M5', length=8)); r.add('insert', 'M5', (x, y, DISC_Z[0]), (0, 0, -1), 8, 'hub bolts')
    for (y, z) in GANTRY_HOLES:
        p = (B['leg_x0'], y, z)
        cut.append(screw_cut(p, (1, 0, 0), 'M5', (B['x1'] - B['leg_x0']) + 6.0 + 4.0)); r.add('SHCS+nyloc', 'M5', p, (1, 0, 0), 25, 'bracket to gantry plate (its 5.2 mm holes at y +-50)')
    add('gantry_bracket', s - union(cut), False, (0.55, 0.57, 0.60), r,
        'L bracket: the gantry plate (rear face, kit holes at y +-50 / z -12.5, -72.5) to the disc + hub; 12 mm leg; prints on its side', Rx(90))


def _tri(x0, x1, z0, z1, t=2.0):
    return CS([[(x0, z1), (x1, z0), (x1, z1)]]).extrude(t).transform(np.array([[1, 0, 0, 0], [0, 0, -1, t], [0, 1, 0, 0]], float))


# ------------------------------------------------------------------ fixed: drive mount, strap, pinion ----------
def servo_frame():
    """STS3215 frame: output face up at the ear underside, shaft at PIN_C, body inward (SDIR -1) or outward."""
    u = np.array([math.cos(PIN_A), math.sin(PIN_A), 0.0])
    xs, zs = SDIR * -u, np.array([0.0, 0.0, -1.0])
    return basis(xs, np.cross(zs, xs), zs, (PIN_C[0], PIN_C[1], SERVO_Z))


def drive_mount():
    r = Reg(); Fs = servo_frame()
    u = np.array([math.cos(PIN_A), math.sin(PIN_A), 0.0]); v = np.array([-u[1], u[0], 0.0])
    R = np.eye(4); R[:3, 0], R[:3, 1] = u, v                   # radial frame
    z0, z1 = EAR['z']
    case_top = SERVO_Z + RIM                                     # output-face rim (carriage z)
    ear = box(EAR['r0'], EAR['r1'], -EAR['w'], EAR['w'], z0, z1)
    # cradle walls around the case (case: servo x -35.2 .. 10.2 -> radial 90 - 10.2 .. 90 + 35.2, y +-12.4)
    W = 2.4
    cr = box(SR[0] - W, SR[1] + W, -12.4 - W, 12.4 + W, SERVO_Z - RIM - 0.01, z1)
    pocket = box(SR[0] - 0.2, SR[1] + 0.2, -12.6, 12.6, SERVO_Z - 40, case_top + 0.1)
    zb = SERVO_Z - RIM                                           # case bottom (idler-face rim)
    rs = RAD(-29.0)                                              # strap at the idler-face case screws
    bosses = [box(rs - 5, rs + 5, sy * 14.0, sy * 21.0, zb, zb + 8.0) if sy > 0 else box(rs - 5, rs + 5, -21.0, -14.0, zb, zb + 8.0)
              for sy in (1, -1)]
    s = xf(union([ear, cr] + bosses), R) - xf(pocket, R)
    cut = [cyl(11.5, z0 - 5, z1 + 1, PIN_C[0], PIN_C[1], hole=True)]                  # pinion hub + horn
    for (xc, yc) in SO100['horn']:                                                    # 2 case screws through the output face
        pc = Fs @ np.array([xc, yc, -RIM - 2.0, 1.0])
        cut.append(screw_cut(tuple(pc[:3]), (0, 0, -1), 'M2', 2.0 + 4.0, cb=z1 - pc[2] + 1, clear=2.0, cb_dia=4.4))
        r.add('self-tap', 'M2', tuple(pc[:3]), (0, 0, -1), 6, 'servo case (output face)')
    for off in (-10.0, 10.0):
        p = tuple(u * 56.0 + v * off + np.array([0, 0, z1]))
        cut.append(insert_cut(p, (0, 0, -1), 'M3', length=6)); r.add('insert', 'M3', p, (0, 0, -1), 6, 'drive mount to ring base')
    for sy in (-1, 1):                                                                # strap inserts in the cradle walls
        p = tuple(R @ np.array([rs, sy * 17.8, zb, 1.0]))[:3]
        cut.append(insert_cut(p, (0, 0, 1), 'M2', length=4)); r.add('insert', 'M2', p, (0, 0, 1), 4, 'strap')
    s = s - union(cut)
    add('drive_mount', s, False, (0.55, 0.57, 0.60), r,
        'ear under the ring base + cradle for the ring-drive STS3215 (output up); servo goes in from below; '
        'prints ear down', Rx(180))
    # strap across the case bottom (idler face) at the rear, 2 x M2 case screws + 2 x M2 into the cradle walls
    sr = Reg(); zb = SERVO_Z - RIM
    rs = RAD(-29.0)
    strap = box(rs - 5, rs + 5, -21.0, 21.0, zb - 3.0, zb)
    sc = []
    for (xc, yc) in SO100['idler']:
        pc = Fs @ np.array([xc, yc, RIM + 2.0, 1.0])
        sc.append(screw_cut(tuple(pc[:3]), (0, 0, 1), 'M2', 2.0 + 4.0, cb=1.0, clear=2.0, cb_dia=4.4))
        sr.add('self-tap', 'M2', tuple(pc[:3]), (0, 0, 1), 6, 'servo case (idler face)')
    for sy in (-1, 1):
        p = tuple(R @ np.array([rs, sy * 17.8, zb - 3.0, 1.0]))[:3]
        sc.append(screw_cut(p, (0, 0, 1), 'M2', 3.0 + 3.0, cb=0.0)); sr.add('SHCS', 'M2', p, (0, 0, 1), 6, 'strap to cradle')
    add('drive_strap', xf(strap, R) - union(sc), False, (0.55, 0.57, 0.60), sr, 'holds the servo in the cradle; prints flat')


def pinion():
    r = Reg(); cx, cy = PIN_C; hf = EAR['z'][0]                 # horn face
    g = gear2d(ZP, MOD).extrude(GEAR_Z[1] - GEAR_Z[0]).translate((cx, cy, GEAR_Z[0]))
    s = union([g, cyl(10.6, hf, GEAR_Z[0] + 0.01, cx, cy)])
    cut = [cyl(CTR_D / 2, hf - 0.1, hf + (OUT_NUB + 3.5 - OUT_FACE), cx, cy, hole=True)]
    seat = hf + 6.9
    for (a, b) in HORN_HOLES:
        cut.append(screw_cut((cx + a, cy + b, seat), (0, 0, -1), 'M3', 6.9, cb=GEAR_Z[1] - seat + 0.5))
        r.add('SHCS', 'M3', (cx + a, cy + b, seat), (0, 0, -1), 10, 'pinion to servo horn')
    add('ring_pinion', s - union(cut), False, (0.85, 0.85, 0.88), r, f'PETG: {ZP} T m{MOD:g} pinion on the drive servo horn; '
        'prints gear down', Rx(180), 'PETG')


# The Waveshare bus-servo board on the user's SO-101 mounting plate (51 x 42 x 4, 18 mm dovetail peg on its back),
# hanging under the ring base, front-right of the beam: the free quadrant (drive front-left, gantry behind, wheels at
# the sides), right beside the hub hole the servo bus goes up through, near the drive servo and the hall sensor.
# Power + USB come up the lift in a drag chain to here.
BOARD = dict(face_x=44.0, yc=-36.0, zc=-9.0, w=51.0, h=42.0, t=4.0, depth=15.0)   # plate on the face x = face_x


def board_mount():
    r = Reg(); B = BOARD; X = B['face_x']
    y0, y1 = -62.0, -8.0
    flange = box(24.0, 58.0, y0, y1, 15.0, DISC_Z[0])
    wall = box(X - 7.0, X, y0, y1, B['zc'] - B['h'] / 2 - 2.0, 15.01)
    gus = union([xf(_tri(24.0, X - 7.0, B['zc'] - 12.0, 15.0, t=3.0), T(0, y, 0)) for y in (y0, y1 - 3.0)])
    s = union([flange, wall, gus])
    zc = B['zc']
    dv = CS([[(-1.0, zc - 8.15), (1.0, zc - 8.15), (2.6, zc - 9.25), (4.0, zc - 9.25),
              (4.0, zc + 9.25), (2.6, zc + 9.25), (1.0, zc + 8.15), (-1.0, zc + 8.15)]])
    y_stop = B['yc'] + 9.25                                     # the peg (centred on the plate) stops here
    L = y_stop - (y0 - 1.0)
    dv = xf(dv.extrude(L).translate((0, 0, y0 - 1.0)), basis((-1, 0, 0), (0, 0, 1), (0, 1, 0), (X, 0, 0)))
    cut = [dv, cyl(3.6, 14.0, DISC_Z[0] + 1, *CABLE_HOLE_XY, hole=True)]              # dovetail; cables from the hub hole
    for (x, y) in BOARD_INSERTS:
        p = (x, y, 15.0 + 1.5)
        cut.append(screw_cut(p, (0, 0, 1), 'M3', DISC_Z[0] - p[2] + 6.0, cb=3.0)); r.add('SHCS', 'M3', p, (0, 0, 1), 10, 'board mount to ring base')
    for y in (y0 + 6.0, y1 - 6.0):                                                    # cable-tie slots for the drag-chain cables
        cut.append(box(X - 8.0, X + 1.0, y - 1.5, y + 1.5, B['zc'] - B['h'] / 2 + 2.0, B['zc'] - B['h'] / 2 + 7.0))
    add('board_mount', s - union(cut), False, (0.55, 0.57, 0.60), r,
        'hangs under the ring base (front right): dovetail socket for the SO-101 Waveshare plate (slides in from the '
        'right side), cable-tie slots for the power + USB from the lift; prints on its back (wall face down)', Ry(-90))
    plate = box(X, X + B['t'], B['yc'] - B['w'] / 2, B['yc'] + B['w'] / 2, zc - B['h'] / 2, zc + B['h'] / 2)
    board = box(X + B['t'], X + B['t'] + B['depth'], B['yc'] - 20.0, B['yc'] + 20.0, zc - 17.0, zc + 17.0)
    CONTEXT['servo_board'] = union([plate, board])              # plate + board with its plugs (envelope)



# ------------------------------------------------------------------ moving (PETG) -----------------------------------
def shelf_shape():
    """Shelf outline (ring frame, heading +x): under the base pillar and round under its tongue."""
    sh = CS.circle(40.0, 96).translate((L1, 0.0))
    sh = sh + CS([[(ROT_OUT - 4.0, -WY - 6.5), (L1 - 20.0, -WY - 6.5), (L1 - 20.0, WY + 6.5), (ROT_OUT - 4.0, WY + 6.5)]])
    return sh.extrude(SHELF_Z[1] - SHELF_Z[0]).translate((0, 0, SHELF_Z[0]))


def rotor():
    r = Reg()
    teeth = gear2d(ZR, MOD).extrude(GEAR_Z[1] - GEAR_Z[0]).translate((0, 0, GEAR_Z[0]))
    body = union([teeth, cyl(ROT_OUT, GEAR_Z[1] - 0.01, ROT_TOP), cyl(MOD * ZR / 2 - 1.25 * MOD, ROT_Z0, GEAR_Z[0] + 0.01)])
    ribs = []
    for sy in (-1, 1):                       # two ribs on the shelf beside the base pillar (|y| 21..27), rotor to r 125
        rib = CS([[(ROT_OUT - 6.0, SHELF_Z[1] - 0.01), (L1 - 36.0, SHELF_Z[1] - 0.01), (ROT_OUT + 1.0, 80.0), (ROT_OUT + 1.0, ROT_TOP + 0.3), (ROT_OUT - 6.0, ROT_TOP + 0.3)]]).extrude(6.0)   # clear of the tray above the rotor top
        rib = rib.transform(np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0]], float))          # XY polygon -> XZ, extruded along +y
        ribs.append(xf(rib, T(0, (WY + 0.5) if sy > 0 else -(WY + 6.5), 0)))
    s = union([body, shelf_shape()] + ribs)
    s = s - cyl(BORE_R, ROT_Z0 - 1, LIP[0], hole=True) - cyl(LIP[2], LIP[0] - 0.01, LIP[1] + 0.01, hole=True) - cyl(BORE_R, LIP[1], ROT_TOP + 30, hole=True)
    cut = []
    for (x, y) in BASE_M5:                   # the base's 4 M5 inserts: screws up through the shelf, heads sunk 5.5 mm
        p = (L1 + x, y, SHELF_Z[0] + 5.5)
        cut.append(screw_cut(p, (0, 0, 1), 'M5', SHELF_Z[1] - p[2], cb=6.0)); r.add('SHCS', 'M5', p, (0, 0, 1), 14, 'arm base (01 + 02) to the shelf')
    for a in TRAY_SCREWS:
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))]) * ((BORE_R + ROT_OUT) / 2)
        cut.append(insert_cut((u[0], u[1], ROT_TOP), (0, 0, -1), 'M3', length=4)); r.add('insert', 'M3', (u[0], u[1], ROT_TOP), (0, 0, -1), 4, 'cable tray')
    a = math.radians(HOME_A)
    cut.append(xf(along(cyl(2.6, -0.1, 3.2, hole=True), 'x'), T(BORE_R * math.cos(a), BORE_R * math.sin(a), 23.0) @ Rz(HOME_A)))
    r.add('part', 'magnet 5 x 3', (BORE_R * math.cos(a), BORE_R * math.sin(a), 23.0), (math.cos(a), math.sin(a), 0), None, 'homing magnet (glue in)')
    add('rotor', s - union(cut), True, (0.20, 0.55, 0.62), r,
        f'PETG: both 61820 outer rings (lip between them) + {ZR} T m{MOD:g} gear + shelf with ribs for the arm base; '
        'prints upright (gear down), tree supports under the shelf, 5 walls', np.eye(4), 'PETG')


def gear2d(z, mod):
    from arm import involute_gear
    return involute_gear(z, mod)


def cable_tray():
    """Moving outer wall + floor of the cable channel, screwed down on the rotor top; the cables leave it at the arm
    heading (+x), over the wall toward the base's rear cable slot."""
    r = Reg(); f0, f1 = CH['floor']
    s = union([ring(CH['ri'] + 1.0, ROT_OUT, f0, f1), ring(CH['ro'], ROT_OUT, f0, CH['top']),
               ring(CH['ro'] - 4.0, CH['ro'] + 0.01, CH['top'] - 2.0, CH['top'])])
    s = s - box(CH['ro'] - 6, ROT_OUT + 1, -6.0, 6.0, f1, CH['top'] - 2.0)                                # exit toward the arm
    cut = []
    for a in TRAY_SCREWS:
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))]) * ((BORE_R + ROT_OUT) / 2)
        cut.append(screw_cut((u[0], u[1], f1), (0, 0, -1), 'M3', f1 - ROT_TOP + 4.0, cb=0.0)); r.add('SHCS', 'M3', (u[0], u[1], f1), (0, 0, -1), 6, 'cable tray to rotor')
    s = s - union(cut)
    add('cable_tray', s, True, (0.20, 0.55, 0.62), r, 'moving outer wall + floor of the cable channel (reversing loop); prints floor down')


# ------------------------------------------------------------------ the arm on the shelf --------------------------
def arm_world(ring_deg=0.0, q1=0.0, q2=0.0, pitch=0.0, roll=0.0, opening=60.0, parts=True, servos=True):
    """The prototype arm minus its forearm, on the shelf: base + upper arm (shoulder q1) + wrist (pan q2 at 130 mm) +
    pitch / roll / gripper; list of (name, solid, color) in the carriage frame."""
    import build, arm
    L = build.L1
    F = build.frames((q1, 0.0, q2), pitch, roll, opening)
    S = Rz(q1) @ T(-L, 0, 0) @ Rz(-q1)                       # drop the forearm: everything past the elbow moves back 130
    G = Rz(ring_deg) @ T(L1, 0, AZ)
    out = []
    key = {}
    if parts:
        build.build_parts(F)
        for nm, p in arm.PARTS.items():
            if nm.startswith('forearm'): continue
            fr = p['frame'] if 'frame' in p else np.eye(4)
            past = not (nm.startswith('base') or nm.startswith('upper_arm'))
            out.append((nm, xf(p['solid'], G @ (S if past else np.eye(4)) @ fr), p['color']))
    if servos:
        for b in build.bought(F):
            if 'elbow' in b['name'] or '_J2_' in b['name']: continue          # the forearm's joint is gone
            past = not ('shoulder' in b['name'] or '_J1_' in b['name'])
            out.append((b['name'], xf(b['solid'], G @ (S if past else np.eye(4))), b.get('color', (0.12, 0.12, 0.14))))
    return out


def build_all():
    PARTS.clear(); CONTEXT.clear()
    ring_base(); spacer(); clamp_ring(); gantry_bracket(); drive_mount(); pinion(); board_mount()
    CONTEXT['drive_servo'] = xf(SERVO, servo_frame())
    rotor(); cable_tray()
    CONTEXT['bearing_1'] = bearing(B1_Z); CONTEXT['bearing_2'] = bearing(B2_Z)


# ------------------------------------------------------------------ checks ----------------------------------------
def check(n_ring=12):
    import build, arm
    res, info = [], []
    fixed = {k: p['solid'] for k, p in PARTS.items() if not p['moving']}
    fixed['servo_board'] = CONTEXT['servo_board']; fixed['drive_servo'] = CONTEXT['drive_servo']
    env = {'gantry plate': GANTRY, 'V-wheels': WHEELS, 'beam': COLUMN}
    b1, b2 = CONTEXT['bearing_1'], CONTEXT['bearing_2']
    inner = union([b1['inner'], b2['inner']]); outer = union([b1['outer'], b2['outer']]); band = union([b1['band'], b2['band']])
    # 1) fixed parts against each other and the lift (the pinion meshes with the ring by design; servo / mount / pinion touch by design)
    names = list(fixed)
    skip = {frozenset(p) for p in [('drive_mount', 'drive_servo'), ('drive_strap', 'drive_servo'), ('ring_pinion', 'drive_servo'),
                                   ('board_mount', 'servo_board'), ('drive_mount', 'drive_strap')]}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if frozenset((a, b)) in skip: continue
            v = (fixed[a] ^ fixed[b]).volume()
            if v > 0.5: res.append(f'fixed {a} x {b}: {v:.1f} mm3')
        for e, es in env.items():
            v = (fixed[a] ^ es).volume()
            if v > 0.5: res.append(f'fixed {a} x {e}: {v:.1f} mm3')
    # 2) bearings: fixed parts may touch only the inner rings, turning parts only the outer rings; nothing in the ball band
    moving = {k: p['solid'] for k, p in PARTS.items() if p['moving']}
    for k, s in fixed.items():
        for nm, ref in (('outer rings', outer), ('ball band', band)):
            v = (s ^ ref).volume()
            if v > 0.5: res.append(f'fixed {k} x bearing {nm}: {v:.1f} mm3')
        v = (s ^ inner).volume()
        if v > 0.5: res.append(f'fixed {k} overlaps the inner rings: {v:.1f} mm3 (seat too tight)')
    for k, s in moving.items():
        for nm, ref in (('inner rings', inner), ('ball band', band)):
            v = (s ^ ref).volume()
            if v > 0.5: res.append(f'moving {k} x bearing {nm}: {v:.1f} mm3')
        v = (s ^ outer).volume()
        if v > 0.5: res.append(f'moving {k} overlaps the outer rings: {v:.1f} mm3')
    # 3) everything that turns with the ring (rotor, tray, the arm's base) against everything fixed, all the way round
    base = union([s for nm, s, c in arm_world(0, 0, 0, servos=True) if nm.startswith('base') or 'shoulder' in nm or '_J1_' in nm])
    turn = union(list(moving.values()) + [base])
    allfixed = union([s for k, s in fixed.items() if k not in ('ring_pinion', 'drive_servo', 'servo_board')] + list(env.values()))
    pin_body = PARTS['ring_pinion']['solid'] - cyl(MOD * ZP / 2 + MOD + 0.01, GEAR_Z[0] - 0.01, GEAR_Z[1] + 1, *PIN_C)
    allfixed = union([allfixed, pin_body, CONTEXT['drive_servo'], CONTEXT['servo_board']])
    worst = {}
    for ang in range(0, 360, 10):
        v = (xf(turn, Rz(ang)) ^ allfixed).volume()
        if v > 0.5: worst[ang] = round(v, 1)
    if worst: res.append(f'rotor / tray / arm base hit fixed parts at ring angles {worst}')
    v = (base ^ union(list(moving.values()))).volume()
    if v > 0.5: res.append(f'arm base x rotor / shelf / tray: {v:.1f} mm3')
    # gaps that matter
    gap_pin = SHELF_Z[0] - GEAR_Z[1]
    # 4) the arm (upper arm + wrist + hand) moving: against the rotor group (any ring angle: same relative pose) and
    #    against the fixed ring parts + lift at every ring angle; hand hanging (pitch 90) and level (0)
    arm_hits, fixed_hits, col_hits = collections.Counter(), collections.Counter(), collections.Counter()
    near = union([s for k, s in fixed.items() if k not in ('gantry_bracket',)] + [GANTRY, WHEELS, CONTEXT['servo_board']])
    rot_grp = union(list(moving.values()))
    n = 0
    for q1 in range(-115, 116, 23):
        for q2 in range(-122, 123, 61):
            for pitch in (0, 90):
                n += 1
                w = arm_world(0, q1, q2, pitch, servos=True)
                mv = union([s for nm, s, c in w if not (nm.startswith('base') or 'shoulder' in nm)])
                if (mv ^ rot_grp).volume() > 0.5: arm_hits[(q1, q2, pitch)] += 1
                for ra in range(0, 360, 360 // n_ring):
                    m2 = xf(mv, Rz(ra))
                    if (m2 ^ near).volume() > 0.5: fixed_hits[(q1, q2, pitch)] += 1
                    if (m2 ^ COLUMN).volume() > 0.5: col_hits[(q1, q2, pitch)] += 1
    info.append(f'arm sweep: {n} arm poses x {n_ring} ring angles; arm vs rotor / shelf / tray: {len(arm_hits)} poses; '
                f'vs fixed ring parts / gantry / board: {len(fixed_hits)} poses; vs the beam: {len(col_hits)} poses')
    if arm_hits: info.append(f'  arm folded back onto the rotor / shelf / tray (planning limit, not a part clash): {sorted(arm_hits)[:10]}')
    if fixed_hits: info.append(f'  arm vs fixed ring parts (poses: ring angles): {dict(list(sorted(fixed_hits.items()))[:12])}')
    if col_hits: info.append(f'  arm vs beam (poses: ring angles): {dict(list(sorted(col_hits.items()))[:12])}')
    cd = math.hypot(*PIN_C)
    info.append(f'gear {ZR} / {ZP} T m{MOD:g}, centre distance {cd:.2f} (ideal {MOD * (ZR + ZP) / 2:.2f}); shelf {gap_pin:.1f} mm over the pinion')
    rng = math.degrees(2 * math.pi * CH['ro'] / (CH['ri'] + CH['ro']))
    info.append(f'cable channel {CH["ro"] - CH["ri"]:.0f} x {CH["top"] - 2 - CH["floor"][1]:.1f} mm, reversing loop +-{rng:.0f} deg')
    info.append(f'arm plane carriage z {AZ:.1f}; shoulder axis {L1:.0f} mm from the column; bearings at z {B1_Z:.0f} / {B2_Z:.0f} '
                f'({B2_Z - B1_Z:.0f} mm apart); hub corner wall {HUB_R - math.hypot(*BEAM):.1f} mm')
    return res, info


# ------------------------------------------------------------------ export ----------------------------------------
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
        num = f'M{i:02d}_{nm}'
        _write(os.path.join(out, 'assembly_stl', num + '.stl'), p['solid'])
        s = xf(p['solid'], p['print_rot']); bb = s.bounding_box()
        s = s.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2])); bb = s.bounding_box()
        _write(os.path.join(out, 'print_ready_stl', num + '.stl'), s)
        rows.append(dict(file=num + '.stl', part=nm, material=p['material'], turns_with_arm='yes' if p['moving'] else 'no',
                         size_mm=f'{bb[3]-bb[0]:.1f} x {bb[4]-bb[1]:.1f} x {bb[5]-bb[2]:.1f}',
                         volume_cm3=round(p['solid'].volume() / 1000, 1), note=p['note']))
        for it in p['reg'].items:
            fast[(it['kind'], it['size'], it['length'], it['note'])] += 1
    fast[('bearing', '6820-2RS = 61820-2RS (100 x 125 x 13)', '', 'ring: 2 (chosen: BM3 chrome steel 6820 2RS, pack of 5, Moglix)')] = 2
    ctx = os.path.join(out, 'assembly_stl', 'context')
    _write(os.path.join(ctx, 'bearings_61820.stl'), union([union([b['inner'], b['outer'], b['band']]) for b in (CONTEXT['bearing_1'], CONTEXT['bearing_2'])]))
    _write(os.path.join(ctx, 'drive_servo_STS3215.stl'), CONTEXT['drive_servo'])
    _write(os.path.join(ctx, 'servo_board_on_SO101_plate.stl'), CONTEXT['servo_board'])
    _write(os.path.join(ctx, 'lift_kit_gantry_wheels_beam.stl'), union([GANTRY, WHEELS, COLUMN ^ box(-30, 30, -50, 50, -110, 140)]))
    _write(os.path.join(ctx, 'arm_base_01_02_with_shoulder_servo.stl'),
           union([s for nm, s, c in arm_world(0, 0, 0) if nm.startswith('base') or 'shoulder' in nm or '_J1_' in nm]))
    with open(os.path.join(out, 'printed_parts.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    with open(os.path.join(out, 'fasteners.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['kind', 'size', 'length_mm', 'qty', 'where'])
        for (k, s, L, n), q in sorted(fast.items(), key=lambda t: (str(t[0][1]), t[0][0], str(t[0][2]))):
            w.writerow([k, s, L, q, n])
    render(os.path.join(out, 'preview_mono_ring.png'))
    print(f'exported {len(rows)} parts to {out}')


def render(path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    def tris(items, cut=None):
        T_, C_ = [], []
        for s, c in items:
            if cut is not None: s = s ^ cut
            m = s.to_mesh(); t = np.asarray(m.vert_properties)[:, :3][np.asarray(m.tri_verts)]
            if len(t): T_.append(t); C_.append(np.tile(np.r_[c[:3]], (len(t), 1)))
        return np.vstack(T_), np.vstack(C_)

    def draw(ax, items, el, az, cut=None, title='', lim=None, zoom=1.0):
        T_, C_ = tris(items, cut)
        N = np.cross(T_[:, 1] - T_[:, 0], T_[:, 2] - T_[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
        Ld = np.array([math.cos(math.radians(el + 25)) * math.cos(math.radians(az + 30)),
                       math.cos(math.radians(el + 25)) * math.sin(math.radians(az + 30)), math.sin(math.radians(el + 25))])
        sh = 0.35 + 0.65 * np.abs(N @ Ld)
        ax.add_collection3d(Poly3DCollection(T_, facecolors=np.c_[C_ * sh[:, None], np.ones(len(sh))], edgecolor='none'))
        lo, hi = (T_.reshape(-1, 3).min(0), T_.reshape(-1, 3).max(0)) if lim is None else lim
        c = (lo + hi) / 2; r = (hi - lo).max() / 2 / zoom
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off(); ax.set_title(title, fontsize=13)
        ax.set_facecolor((0.906, 0.922, 0.941))

    BRGC = (0.98, 0.80, 0.10)                                   # the two 61820s: bright yellow, to see them
    ring_items = [(p['solid'], p['color']) for p in PARTS.values()]
    ring_items += [(union([b['inner'], b['outer'], b['band']]), BRGC) for b in (CONTEXT['bearing_1'], CONTEXT['bearing_2'])]
    ring_items += [(CONTEXT['drive_servo'], (0.12, 0.12, 0.14)), (CONTEXT['servo_board'], (0.10, 0.35, 0.20))]
    lift = [(GANTRY, (0.80, 0.80, 0.82)), (WHEELS, (0.35, 0.35, 0.38)), (COLUMN ^ box(-30, 30, -50, 50, -110, 170), (0.30, 0.33, 0.37))]
    armw = [(s, c) for nm, s, c in arm_world(0, 35, -60, 30)]
    fig = plt.figure(figsize=(21, 13), facecolor=(0.906, 0.922, 0.941))
    ax = fig.add_subplot(2, 2, 1, projection='3d'); draw(ax, ring_items + lift + armw, 24, -60, title='layout B: arm base on the ring shelf (shoulder 161 mm out)', zoom=1.25)
    ax = fig.add_subplot(2, 2, 2, projection='3d'); draw(ax, ring_items + lift + [(s, c) for nm, s, c in arm_world(0, 0, 0, 0, servos=True)
                                                                              if nm.startswith('base') or 'shoulder' in nm or '_J1_' in nm], 30, 130, title='from behind: ring, shelf, base 01 + 02', zoom=1.2)
    sec = box(-300, 300, 0, 300, -200, 300)
    ax = fig.add_subplot(2, 2, 3, projection='3d')
    draw(ax, ring_items + lift, 0, -90, cut=sec,
         title='cut through the ring: the two 61820 bearings (yellow) on the hub, spacer / lip between them, clamp on top',
         lim=(np.array([20, 0, 5]), np.array([90, 0, 85])))
    ax = fig.add_subplot(2, 2, 4, projection='3d')
    expl = []
    for k, p in PARTS.items():
        dz = {'ring_base': -60, 'gantry_bracket': -60, 'drive_mount': -60, 'drive_strap': -60, 'ring_pinion': -60, 'board_mount': -60,
              'inner_spacer': 40, 'clamp_ring': 110, 'rotor': 0, 'cable_tray': 80}.get(k, 0)
        expl.append((xf(p['solid'], T(0, 0, dz)), p['color']))
    expl += [(xf(union([b['inner'], b['outer'], b['band']]), T(0, 0, dz)), BRGC) for b, dz in ((CONTEXT['bearing_1'], -25), (CONTEXT['bearing_2'], 60))]
    draw(ax, expl, 18, -60, title='exploded: disc + hub, bearing 1, rotor, spacer, bearing 2, tray, clamp', zoom=1.3)
    plt.subplots_adjust(left=0, right=1, bottom=0, top=0.96, wspace=0, hspace=0.06); plt.savefig(path, dpi=90, facecolor=fig.get_facecolor()); plt.close(fig)


if __name__ == '__main__':
    import mono_ring as RB
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = args[0] if args else os.path.join('..', '..', 'CAD', 'mono_ring')
    RB.build_all()
    if '--no_check' not in sys.argv:
        res, info = RB.check()
        for line in info: print(line)
        print('problems:', res if res else 'none')
    if '--no_export' not in sys.argv: RB.export(out)
