"""DRAFT printable parts for the mono-arm shoulder ring (mono-arm experiment, user 2026-11-11: "keep it a draft ...
mechanism matters most"). The arm's shoulder turns around the C-Beam lift column on a PRINTED two-row ball bearing
(6 mm steel balls; user 2026-11-11: no bought bearing),
driven by an STS3215 through a 72 / 18 tooth gear (4:1). Units mm. Frame = CARRIAGE: origin on the column axis,
x forward (working face), z = carriage z (the arm plane is at z AZ = 65.5, as in the mono URDF).

Lift = the real C-Beam XLarge kit (2026-11-13, from its STEP: cbeam_import.py): gantry plate 125 x 125 x 6, 1.5 mm
behind the beam's open face (x -27.5..-21.5), top at z 20; V-wheels x -16..0.5, |y| 38..62; the lead-screw nut block
between plate and beam (z -62.5..-42.5). The bracket uses 4 of the plate's own 5.2 mm holes (y +-50, z -12.5 / -72.5:
beside the beam and between the wheels, so the nylocs fit in front of the plate).

Stack (carriage z):
    20 .. 28   ring base disc (rests on the gantry plate's top edge), hub up to 60 = inner race (r 54.5)
    35, 55     ball rows (57 x 6 mm each, pitch r 56); rotor = outer race (r 57.5) in 3 rings split at 35 / 55
    60 .. 64   top cap (camera post socket, inner wall of the cable channel up to 76)
    22.5 .. 30.5  gear teeth on the rotor (m2, 72 T) and the pinion (18 T) on the drive servo at r 90, 60 deg
    31.5 .. 46    rotor lug: a saddle under the upper-arm tube (its underside 43.5, curving down to 38.5 at the elbow end)
Why so low: the forearm's bottom fork prong is 33.6 mm under the arm plane (z 31.9) and sweeps r 92..168 around the
column whenever the elbow passes; anything fixed out there must stay below it (pinion top 30.5).

    python3 mono_ring.py [out_dir]      checks + export (default ../../CAD/mono_ring_draft)
"""
import math, os, sys, csv
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import M, CS, box, cyl, ring, union, xf, T, Rx, Ry, Rz, basis, mesh_arrays, along
from hw import screw_cut, insert_cut, Reg, place, SPEC
from joint import SERVO, HORN_HOLES, OUT_FACE, RIM, CTR_D, OUT_NUB, case_screw, SO100

AZ = 65.5                                   # arm plane (carriage z)
# PRINTED two-row ball bearing (user 2026-11-11: no bought bearing; 6 mm steel balls are fine). Inner race = the hub
# (two grooves), outer race = the rotor, printed as 3 rings split through the groove centres; balls go in row by row
# while stacking the rings. The rings are modelled PRELOAD apart at each split: 8 x M3 pull them together and squeeze
# the balls (3-point contact: takes axial load both ways and the arm's tipping moment). Grooves 6.2 for 6.0 balls.
BALL = dict(d=6.0, pitch_r=56.0, groove_r=3.1, rows=(35.0, 55.0), n=57)       # full complement minus one ball per row
HUB_R = 54.5                                # hub race surface (groove 1.6 deep)
ROT_IN = 57.5                               # rotor race surface (groove 1.6 deep); 3 mm radial gap bridged by the balls
PRELOAD = 0.15                              # gap at each rotor split, closed by the stack bolts
STACK = dict(r=64.0, angles=[22.5 + 45.0 * k for k in range(8)])
HUB_TOP = 60.0
CLAMP = (60.0, 64.0)                        # top cap (fixed): camera post socket, cable channel's inner wall
BEAM = (22.0, 42.0)                         # beam hole half sizes (beam 40 x 80 + 2 mm)
DISC_R, DISC_Z = 64.0, (20.0, 28.0)
# gear: m2, ring 72 T on the rotor, pinion 18 T: 4:1, centre distance 90
MOD, ZR, ZP = 2.0, 72, 18
GEAR_Z = (22.5, 30.5)
PIN_A = math.radians(60.0)                  # drive position (front-left): clear of the gantry plate and its wheels
PIN_C = (90.0 * math.cos(PIN_A), 90.0 * math.sin(PIN_A))
ROT_IN_LOW = 65.0                           # rotor inner radius below the races (clears the disc r 64)
ROT_OUT = 69.0                              # rotor body outer radius above the gear
# ARM ROOT COLLAR (2026-11-12, user: "the arm is bound to fall"): the draft held the arm on a small lug under the tube
# with 4 x M3 in ONE row 6.5 mm from the edge it tips over: the arm (1.76 kg, COM 266 mm out) + a 0.25 kg product put
# ~4.3 N m on it at rest (~650 N pull on those screws; about twice that moving). Now the upper arm's bottom half has a
# COLLAR all the way round the rotor (r 69.4..75, from just above the gear to the arm's split line), screwed radially into
# the rotor's middle ring at 6 places: the tipping load spreads over the ring's 140 mm diameter.
COLLAR = dict(ri=ROT_OUT + 0.4, ro=75.0, z0=31.6, screw_z=43.0, screw_a=(45.0, 90.0, 135.0, 225.0, 270.0, 315.0))
TAB_Z = 52.0                                # cable-tray tab screws (radial, into the middle ring); the draft had the
                                            # screw at z 55 and the insert at z 49 (they missed)
TUBE_X0 = 70.0                              # the mono upper arm starts here (the prototype's shoulder fork is cut off)
TUBE_Z = AZ - 22.0                          # tube underside (carriage z 43.5)
SDIR = -1                                   # servo body: -1 inward (toward the column, under the disc), +1 outward
def RAD(xs): return 90.0 - SDIR * xs        # radial position of a point at servo-frame x = xs
SR = sorted((RAD(-35.2), RAD(10.2)))        # radial span of the case
EAR = dict(z=(13.0, 20.0), w=18.0, r0=48.0, r1=106.0)   # inward servo: nothing fixed beyond r ~106 (the folded wrist hangs there)
SERVO_Z = EAR['z'][0] - OUT_FACE            # drive servo centre (output face = ear underside)
# cable crossing (reversing loop): fixed inner wall (drum on the clamp ring) r 52..56, moving outer wall (tray on the
# rotor) r 66..69; the cables lie on the tray floor, run along the drum, U-turn, and come back along the outer wall.
# The U-turn travels at half the ring's speed: range +-(2 pi r_o / (r_i + r_o)) = +-195 deg. Two cables stacked:
# servo bus (bottom) + wrist-camera USB (top).
CH = dict(ri=56.0, ro=66.0, floor=(60.0, 62.0), top=76.0)
CABLE_HOLE_XY = (27.0, -27.0)               # fixed cable: up through the hub wall from under the disc (front right of the beam)
CABLE_A = math.atan2(CABLE_HOLE_XY[1], CABLE_HOLE_XY[0])
TRAY_TABS = (90.0, 180.0, 270.0)             # tray tabs, away from the arm tube (+-20 deg at the rotor)
ROT_OUT = 69.0                              # (also set below) rotor body outer radius above the gear
HOME_A = 0.0                                # homing: magnet in the rotor's inner wall at the arm heading, hall sensor in the disc rim

# ---- estimated lift hardware (carriage frame) for clearance checks ----
GANTRY = box(-27.5, -21.5, -62.5, 62.5, -105.0, 20.0)       # the kit's plate (cbeam/carriage.stl)
WHEELS = union([box(-16.0, 0.5, 38.0, 62.0, -105.0, 20.0), box(-16.0, 0.5, -62.0, -38.0, -105.0, 20.0)])   # wheels + their nuts
GANTRY_HOLES = [(sy * 50.0, z) for sy in (-1, 1) for z in (-12.5, -72.5)]   # plate holes (y, z) the bracket bolts through
COLUMN = box(-20.0, 20.0, -40.0, 40.0, -300.0, 600.0)

PARTS = {}                                  # name -> dict(solid (carriage frame), moving, color, reg, note, print_rot)


def add(name, solid, moving, color, reg, note, print_rot=np.eye(4)):
    PARTS[name] = dict(solid=solid, moving=moving, color=color, reg=reg, note=note, print_rot=print_rot)


def gear2d(z, mod):
    from arm import involute_gear
    return involute_gear(z, mod)


def slot_cut(p, d, size, shank, cb, along, half):
    """Clearance slot (screw_cut swept +-half mm along `along`)."""
    a = np.asarray(along, float)
    return union([screw_cut(tuple(np.asarray(p) + a * t), d, size, shank, cb=cb) for t in np.linspace(-half, half, 7)])


# ------------------------------------------------------------------ fixed: ring base (disc + hub) -----------------
def ring_base():
    r = Reg()
    s = union([cyl(DISC_R, *DISC_Z), cyl(HUB_R, DISC_Z[1], HUB_TOP)])                  # hub = inner race
    s = s - box(-BEAM[0], BEAM[0], -BEAM[1], BEAM[1], 0, 100) - grooves()
    cut = []
    for sx in (-1, 1):                       # clamp ring: 4 x M3 inserts in the hub top
        for sy in (-1, 1):
            p = (sx * 38.0, sy * 26.0, HUB_TOP)
            cut.append(insert_cut(p, (0, 0, -1), 'M3', length=6)); r.add('insert', 'M3', p, (0, 0, -1), 6, 'clamp ring')
    for sy in (-1, 1):                       # gantry bracket: 2 x M5 inserts from below
        p = (-55.0, sy * 20.0, DISC_Z[0])
        cut.append(insert_cut(p, (0, 0, 1), 'M5', length=6)); r.add('insert', 'M5', p, (0, 0, 1), 6, 'gantry bracket')
    u = np.array([math.cos(PIN_A), math.sin(PIN_A), 0.0]); v = np.array([-u[1], u[0], 0.0])
    for off in (-10.0, 10.0):                # drive mount: 2 x M3 down through the disc in RADIAL slots (gear backlash)
        p = tuple(u * 56.0 + v * off + np.array([0, 0, DISC_Z[1] - 3.2]))
        cut.append(slot_cut(p, (0, 0, -1), 'M3', 8.0, 3.4, u, 1.5))
        r.add('SHCS', 'M3', p, (0, 0, -1), 12, 'drive mount (slotted: slide it to set the gear backlash)')
    cut.append(cyl(3.5, DISC_Z[0] - 1, HUB_TOP + 1, *CABLE_HOLE_XY, hole=True))         # cables (bus + USB) up the hub wall
    for (x, y) in BOARD_INSERTS:             # board mount: 3 x M3 inserts from below
        cut.append(insert_cut((x, y, DISC_Z[0]), (0, 0, 1), 'M3', length=6)); r.add('insert', 'M3', (x, y, DISC_Z[0]), (0, 0, 1), 6, 'board mount')
    # hall sensor (TO-92 / SS49E, 4.2 x 3.2 x 1.6 body) in the disc rim, facing the rotor's magnet 1 mm away
    hx = DISC_R * math.cos(HOME_A); hy = DISC_R * math.sin(HOME_A)
    cut.append(xf(box(-4.0, 0.1, -2.3, 2.3, 22.0, 27.0), T(hx, hy, 0) @ Rz(math.degrees(HOME_A))))
    cut.append(cyl(1.8, DISC_Z[0] - 1, 23.0, (DISC_R - 3.0) * math.cos(HOME_A), (DISC_R - 3.0) * math.sin(HOME_A), hole=True))
    r.add('part', 'hall sensor', (hx, hy, 24.5), (-1, 0, 0), None, 'homing: SS49E / A3144 in the disc rim')
    s = s - union(cut)
    add('ring_base', s, False, (0.55, 0.57, 0.60), r,
        'disc on the gantry plate top + hub = inner race (2 ball grooves); beam hole 44 x 84; prints disc down; '
        '6 walls (the grooves are solid plastic)', np.eye(4))


def clamp_ring():
    r = Reg(); z0, z1 = CLAMP
    s = cyl(ROT_IN - 1.5, z0, z1) - box(-BEAM[0], BEAM[0], -BEAM[1], BEAM[1], 0, 100)
    # scene-camera post socket (post 14 x 16 at x 31..45 as in the URDF), M3 set screw from the front
    boss = box(27.0, 49.0, -12.0, 12.0, z1, z1 + 12.0)
    drum = union([ring(CH['ri'] - 4.0, CH['ri'], z1 - 0.01, CH['top']), ring(CH['ri'] - 0.01, CH['ri'] + 3.0, CH['top'] - 2.0, CH['top'])])
    s = union([s, boss, drum]) - box(30.8, 45.2, -8.2, 8.2, z1 + 2.0, z1 + 13.0)
    cut = [insert_cut((49.0, 0.0, z1 + 7.0), (-1, 0, 0), 'M3', length=4),
           cyl(3.5, z0 - 1, z1 + 1, *CABLE_HOLE_XY, hole=True),
           xf(box(30.0, CH['ri'] + 1, -4.0, 4.0, z1 - 0.01, z1 + 9.0), Rz(math.degrees(CABLE_A)))]     # cables out through the drum
    r.add('insert', 'M3', (49.0, 0.0, z1 + 7.0), (-1, 0, 0), 4, 'camera post set screw')
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = (sx * 38.0, sy * 26.0, z1 - 1.6)
            cut.append(screw_cut(p, (0, 0, -1), 'M3', 1.6 + 0.4 + 6.0, cb=4.0)); r.add('SHCS', 'M3', p, (0, 0, -1), 8, 'clamp ring')
    s = s - union(cut)
    add('clamp_ring', s, False, (0.55, 0.57, 0.60), r,
        'top cap on the hub: fixed inner wall of the cable channel, camera post socket; prints flat', np.eye(4))


# ------------------------------------------------------------------ moving: rotor + gear + lug ------------------
def grooves():
    """The two ball grooves (tori), cut into both races."""
    return union([M.revolve(CS.circle(BALL['groove_r'], 48).translate((BALL['pitch_r'], z)), 160) for z in BALL['rows']])


def balls():
    out = []
    for z in BALL['rows']:
        for k in range(BALL['n']):
            a = 2 * math.pi * k / (BALL['n'] + 1)
            out.append(M.sphere(BALL['d'] / 2, 16).translate((BALL['pitch_r'] * math.cos(a), BALL['pitch_r'] * math.sin(a), z)))
    return union(out)


def rotor(arm_bottom):
    """Outer race + 72 T gear + lug, printed as 3 rings (bottom: gear, magnet; middle: lug, tray inserts; top)."""
    teeth = gear2d(ZR, MOD).extrude(GEAR_Z[1] - GEAR_Z[0]).translate((0, 0, GEAR_Z[0]))
    body = union([teeth, cyl(ROT_OUT, GEAR_Z[1] - 0.01, HUB_TOP)])
    s = body - cyl(ROT_IN_LOW, GEAR_Z[0] - 1, DISC_Z[1] + 1.0, hole=True) - cyl(ROT_IN, DISC_Z[1] + 0.99, HUB_TOP + 1, hole=True) - grooves()
    za, zb = BALL['rows']; g = PRELOAD / 2
    bot = s ^ box(-200, 200, -200, 200, 0, za - g)
    mid = s ^ box(-200, 200, -200, 200, za + g, zb - g)
    top = s ^ box(-200, 200, -200, 200, zb + g, 100)
    rb, rm, rt = Reg(), Reg(), Reg(); cb, cm, ct = [], [], []
    for a in COLLAR['screw_a']:              # the upper arm's collar screws radially into the middle ring
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0])
        p = tuple(u * ROT_OUT + np.array([0, 0, COLLAR['screw_z']]))
        cm.append(insert_cut(p, -u, 'M3', length=6)); rm.add('insert', 'M3', p, tuple(-u), 6, 'upper arm collar')
    cb.append(xf(along(cyl(2.6, -0.1, 3.2, hole=True), 'x'), T(ROT_IN_LOW * math.cos(HOME_A), ROT_IN_LOW * math.sin(HOME_A), 25.5) @ Rz(math.degrees(HOME_A))))
    rb.add('part', 'magnet 5 x 3', (ROT_IN_LOW, 0, 25.5), (1, 0, 0), None, 'homing magnet (glue in)')
    for a in TRAY_TABS:                      # cable tray: radial M3 into the middle ring
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0])
        p = tuple(u * ROT_OUT + np.array([0, 0, TAB_Z]))
        cm.append(insert_cut(p, -u, 'M3', length=4)); rm.add('insert', 'M3', p, tuple(-u), 4, 'cable tray')
    for a in STACK['angles']:                # stack bolts: top + middle -> inserts in the bottom ring (preload)
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0]) * STACK['r']
        seat = HUB_TOP - 3.2
        ct.append(screw_cut((u[0], u[1], seat), (0, 0, -1), 'M3', seat - (za - g) + 0.5, cb=4.0))
        cm.append(cyl(1.7, za - 1, zb + 1, u[0], u[1], hole=True))
        cb.append(insert_cut((u[0], u[1], za - g), (0, 0, -1), 'M3', length=4))
        rt.add('SHCS', 'M3', (u[0], u[1], seat), (0, 0, -1), 25, 'rotor stack (tighten evenly: preload)')
        rb.add('insert', 'M3', (u[0], u[1], za - g), (0, 0, -1), 4, 'rotor stack')
    add('rotor_bottom', bot - union(cb), True, (0.85, 0.45, 0.15), rb,
        f'outer race (lower half of row A) + {ZR} T m{MOD:g} gear + homing magnet; prints gear down, 6 walls', np.eye(4))
    add('rotor_middle', mid - union(cm), True, (0.90, 0.52, 0.20), rm,
        'outer race (upper half of row A, lower half of row B) + inserts for the upper arm collar and the cable tray; '
        'prints upside down (its top face on the bed), 6 walls', Rx(180))
    add('rotor_top', top - union(ct), True, (0.85, 0.45, 0.15), rt, 'outer race (upper half of row B); prints upside down '
        '(groove up), 6 walls', Rx(180))


def collar(reg, note):
    """The arm-root collar round the rotor (r COLLAR ri..ro, from just above the gear to the arm's split line) and its
    cuts: 6 radial M3 into the rotor's middle ring, pockets + screwdriver holes for the cable tray's 3 tabs."""
    C = COLLAR
    col = ring(C['ri'], C['ro'], C['z0'], AZ)
    cut = []
    for a in TRAY_TABS:
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0])
        cut.append(xf(box(C['ri'] - 1.0, ROT_OUT + 3.6, -6.6, 6.6, TAB_Z - 4.6, AZ + 1), Rz(a)))    # tab pocket
        cut.append(xf(along(cyl(3.3, 0, 20.0), 'x'), T(u[0] * (ROT_OUT + 2.0), u[1] * (ROT_OUT + 2.0), TAB_Z) @ Rz(a)))        # driver access
    for a in C['screw_a']:
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0])
        p = tuple(u * (C['ro'] - 2.0) + np.array([0, 0, C['screw_z']]))
        cut.append(screw_cut(p, -u, 'M3', (C['ro'] - 2.0 - ROT_OUT) + 6.0, cb=3.0))
        reg.add('SHCS', 'M3', p, tuple(-u), int(round(C['ro'] - 2.0 - ROT_OUT + 6.0)), note)   # M3 x 10
    return col, cut


def mono_upper_arm():
    """The prototype's upper-arm halves with the shoulder fork cut off at TUBE_X0 (the elbow end is unchanged)."""
    import build, arm
    if not arm.PARTS:
        build.build_parts(build.frames((0, 0, 0), 0, 0, 60.0))
    keep = box(TUBE_X0, 500, -300, 300, -300, 300)
    out = {}
    for half in ('upper_arm_top', 'upper_arm_bottom'):
        s = arm.PARTS[half]['solid'] ^ keep                     # link frame: origin on the shoulder axis, z 0 = arm plane
        out[half] = xf(s, T(0, 0, AZ))
    # the cut-off fork held the halves together at this end: a bolt pair through both halves at x 76 (as the old tube bolts)
    xb, yb = 76.0, 25.0 + 3.5
    rt = Reg(); ct = []
    for sy in (-1, 1):
        y = sy * yb; web = box(xb - 4.5, xb + 4.5, min(sy * 24.5, y), max(sy * 24.5, y), 0, 1)
        out['upper_arm_top'] = union([out['upper_arm_top'], cyl(4.5, AZ, AZ + 22.0, xb, y), box(xb - 4.5, xb + 4.5, min(sy * 24.5, y), max(sy * 24.5, y), AZ, AZ + 22.0)])
        out['upper_arm_bottom'] = union([out['upper_arm_bottom'], cyl(4.5, TUBE_Z, AZ, xb, y),
                                         box(xb - 4.5, xb + 4.5, min(sy * 24.5, y), max(sy * 24.5, y), TUBE_Z, AZ)])
        ct.append(insert_cut((xb, y, AZ), (0, 0, 1), 'M3', length=8)); rt.add('insert', 'M3', (xb, y, AZ), (0, 0, 1), 8, 'upper arm halves (rotor end)')
    r = Reg(); cut = []
    for sy in (-1, 1):
        seat = TUBE_Z + 3.0
        cut.append(screw_cut((xb, sy * yb, seat), (0, 0, 1), 'M3', AZ - seat + 0.2, cb=3.2))
        r.add('SHCS', 'M3', (xb, sy * yb, seat), (0, 0, 1), 25, 'upper arm halves (rotor end)')
    out['upper_arm_top'] = out['upper_arm_top'] - union(ct)
    tube_in = box(TUBE_X0 - 8.0, TUBE_X0 + 20.0, -22.0, 22.0, TUBE_Z + 3.0, AZ + 1)               # the tube's inside
    col, ccut = collar(r, 'upper arm collar to rotor (radial)')
    out['upper_arm_bottom'] = union([out['upper_arm_bottom'], col]) - union(ccut + [tube_in])
    add('mono_upper_arm_bottom', out['upper_arm_bottom'] - union(cut), True, (0.33, 0.56, 0.86), r,
        'prototype upper_arm_bottom without its shoulder fork (x >= 70) + a collar round the rotor (6 radial M3); prints split face down', Rx(180))
    add('mono_upper_arm_top', out['upper_arm_top'], True, (0.25, 0.48, 0.80), rt,
        'prototype upper_arm_top without its shoulder fork (x >= 70); prints split face down', np.eye(4))
    return out['upper_arm_bottom']


def cable_tray():
    """Moving outer wall + floor of the cable channel, on the rotor top; the cables leave it at the arm heading,
    straight into the upper-arm tube's open end (x 70)."""
    r = Reg(); f0, f1 = CH['floor']
    s = union([ring(CH['ri'] + 1.0, ROT_OUT, f0 + 0.2, f1),                                 # floor (0.2 over the bearing)
               ring(CH['ro'], ROT_OUT, f0 + 0.2, CH['top']),                                 # outer wall
               ring(CH['ro'] - 4.0, CH['ro'] + 0.01, CH['top'] - 2.0, CH['top'])])           # lip (keeps the loop in)
    s = s - box(ROT_OUT - 6, ROT_OUT + 1, -6.0, 6.0, f1, CH['top'] - 2.0)                    # exit toward the arm
    tabs, cut = [], []
    for a in TRAY_TABS:
        R_ = Rz(a)
        tabs.append(xf(box(ROT_OUT - 0.01, ROT_OUT + 3.0, -6.0, 6.0, TAB_Z - 4.0, f1), R_))
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0])
        p = tuple(u * (ROT_OUT + 3.0) + np.array([0, 0, TAB_Z]))
        cut.append(screw_cut(p, -u, 'M3', 3.0 + 4.0)); r.add('SHCS', 'M3', p, tuple(-u), 8, 'cable tray to rotor')
    s = union([s] + tabs) - union(cut)
    add('cable_tray', s, True, (0.85, 0.45, 0.15), r,
        'moving outer wall + floor of the cable channel (reversing loop, +-195 deg); prints floor down', np.eye(4))


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
    add('ring_pinion', s - union(cut), False, (0.85, 0.85, 0.88), r, f'{ZP} T m{MOD:g} pinion on the drive servo horn; '
        'prints gear down', Rx(180))


# ------------------------------------------------------------------ fixed: servo board mount --------------------
# The Waveshare bus-servo board on the user's SO-101 mounting plate (51 x 42 x 4, 18 mm dovetail peg on its back),
# hanging under the ring base, front-right of the beam: the free quadrant (drive front-left, gantry behind, wheels at
# the sides), right beside the hub hole the servo bus goes up through, near the drive servo and the hall sensor.
# Power + USB come up the lift in a drag chain to here.
BOARD = dict(face_x=44.0, yc=-36.0, zc=-9.0, w=51.0, h=42.0, t=4.0, depth=15.0)   # plate on the face x = face_x
BOARD_INSERTS = [(50.0, -12.0), (46.0, -32.0), (32.0, -50.0)]                       # M3 from below into the disc
CONTEXT = {}


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


# ------------------------------------------------------------------ fixed: bracket to the gantry plate ----------
def gantry_bracket():
    r = Reg()
    top = box(-64.0, -28.0, -30.0, 30.0, 12.0, DISC_Z[0])
    leg = box(-35.5, -28.0, -56.0, 56.0, -80.0, DISC_Z[0])          # on the plate's back face (-27.5): 0.5 mm, clear of the wheel bolt heads (-27.7)
    gus = union([xf(_tri(-64.0, -35.5, -40.0, 12.0, t=3.0), T(0, y, 0)) for y in (-30.0, 27.0)])   # at the ends: one on the bed
    s = union([top, leg, gus])
    cut = []
    for sy in (-1, 1):
        p = (-55.0, sy * 20.0, 12.0)
        cut.append(screw_cut(p, (0, 0, 1), 'M5', 8.0 + 6.0, cb=6.0)); r.add('SHCS', 'M5', p, (0, 0, 1), 12, 'bracket to ring base')
    for (y, z) in GANTRY_HOLES:                                       # the plate's own 5.2 mm holes, nyloc in front of the plate
        p = (-35.5, y, z)
        cut.append(screw_cut(p, (1, 0, 0), 'M5', 7.5 + 6.0 + 4.0)); r.add('SHCS+nyloc', 'M5', p, (1, 0, 0), 20, 'bracket to gantry plate (its 5.2 mm holes at y +-50)')
    add('gantry_bracket', s - union(cut), False, (0.55, 0.57, 0.60), r,
        'L bracket from the gantry plate (rear face, the kit plate holes at y +-50 / z -12.5, -72.5) to the ring base; prints on its side', Rx(90))


def _tri(x0, x1, z0, z1, t=2.0):
    """Gusset in the XZ plane (x0, z1) - (x1, z1) - (x1, z0), thickness t along +y."""
    from lib import CS
    return CS([[(x0, z1), (x1, z0), (x1, z1)]]).extrude(t).transform(np.array([[1, 0, 0, 0], [0, 0, -1, t], [0, 1, 0, 0]], float))


def build_all():
    PARTS.clear()
    ab = mono_upper_arm()
    ring_base(); clamp_ring(); rotor(ab); cable_tray(); drive_mount(); pinion(); gantry_bracket(); board_mount()
    return TUBE_Z


# ------------------------------------------------------------------ checks --------------------------------------
def forearm_at(shoulder_deg, elbow_deg):
    """Forearm halves (link frame from arm.PARTS) at the elbow, in the carriage frame."""
    import arm
    E = Rz(shoulder_deg) @ T(130.0, 0, AZ) @ Rz(elbow_deg)
    return union([xf(arm.PARTS[h]['solid'], E) for h in ('forearm_top', 'forearm_bottom')])


def check():
    fixed = {k: p['solid'] for k, p in PARTS.items() if not p['moving']}
    fixed.update(CONTEXT)                    # the servo board envelope
    nopin = {k: v for k, v in fixed.items() if k != 'ring_pinion'}     # its teeth mesh with the ring by design
    moving = union([p['solid'] for p in PARTS.values() if p['moving']])
    brg = balls()
    env = {'gantry plate': GANTRY, 'V-wheels': WHEELS, 'beam': COLUMN}
    res = []
    # 1) parts against each other at rest
    names = list(fixed)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            v = (fixed[a] ^ fixed[b]).volume()
            if v > 0.5: res.append(f'fixed {a} x {b}: {v:.1f} mm3')
        for e, es in env.items():
            v = (fixed[a] ^ es).volume()
            if v > 0.5: res.append(f'fixed {a} x {e}: {v:.1f} mm3')
    # 2) the turning parts against everything fixed, all the way round
    worst = {}
    allfixed = union(list(nopin.values()) + list(env.values()))
    pin_body = PARTS['ring_pinion']['solid'] - cyl(MOD * ZP / 2 + MOD + 0.01, GEAR_Z[0] - 0.01, GEAR_Z[1] + 1, *PIN_C)
    allfixed = union([allfixed, pin_body])
    for ang in range(0, 360, 10):
        mv = xf(moving, Rz(ang))
        v = (mv ^ allfixed).volume()
        if v > 0.5: worst[ang] = round(v, 1)
    if worst: res.append(f'rotor / upper arm hit fixed parts at {worst}')
    # 3) the forearm (fork prong) passing over the drive, elbow folded either way
    hits = {}
    drive = union([fixed['drive_mount'], fixed['ring_pinion'], fixed['clamp_ring'], fixed['ring_base'], fixed['board_mount'], fixed['servo_board']])
    tray = PARTS['cable_tray']['solid']
    for el in range(-117, 118, 9):
        v = (forearm_at(0, el) ^ tray).volume()
        if v > 0.5: hits[('tray', el)] = round(v, 1)
    for sh in range(0, 360, 10):
        for el in (-117, -90, -45, 0, 45, 90, 117):
            v = (forearm_at(sh, el) ^ drive).volume()
            if v > 0.5: hits[(sh, el)] = round(v, 1)
    # 6) the cable channel: free width / height, and the reversing-loop range
    ch_w = CH['ro'] - CH['ri']; ch_h = CH['top'] - 2.0 - CH['floor'][1]
    rng = math.degrees(2 * math.pi * CH['ro'] / (CH['ri'] + CH['ro']))
    res_ch = f'cable channel {ch_w:.0f} mm wide x {ch_h:.0f} mm tall, reversing-loop range +-{rng:.0f} deg, cable in the loop ~{(CH["ro"] * math.pi + 40) / 1000:.2f} m each'
    print(res_ch)
    if hits: res.append(f'forearm hits the fixed ring parts at {dict(list(hits.items())[:8])} ({len(hits)} poses)')
    # 7) the arm's collar: clear of the rotor rings, the tray, the balls and the top half; the forearm folding back
    #    over it; each radial collar screw lands in its rotor insert
    ab = PARTS['mono_upper_arm_bottom']['solid']
    for k in ('rotor_bottom', 'rotor_middle', 'rotor_top', 'cable_tray', 'mono_upper_arm_top', 'clamp_ring', 'ring_base'):
        v = (ab ^ PARTS[k]['solid']).volume()
        if v > 0.5: res.append(f'upper arm collar x {k}: {v:.1f} mm3')
    v = (ab ^ brg).volume()
    if v > 0.5: res.append(f'upper arm collar x balls: {v:.1f} mm3')
    fh = [el for el in range(-117, 118, 9) if (forearm_at(0, el) ^ ab).volume() > 0.5]
    if fh: res.append(f'forearm hits the upper arm / collar at elbow {fh}')
    mid = PARTS['rotor_middle']['solid']
    for a in COLLAR['screw_a']:
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))]) * (ROT_OUT - 3.0)
        if (cyl(0.5, COLLAR['screw_z'] - 0.1, COLLAR['screw_z'] + 0.1, u[0], u[1]) ^ mid).volume() > 0.01:
            res.append(f'collar screw at {a} deg misses its insert')
    # 4) the balls sit in their grooves (no overlap with the hub / rotor / cap)
    for k in ('ring_base', 'rotor_bottom', 'rotor_middle', 'rotor_top', 'clamp_ring'):
        v = (PARTS[k]['solid'] ^ brg).volume()
        if v > 0.5: res.append(f'balls x {k}: {v:.1f} mm3')
    # each ball touches (within 0.15 mm) the hub groove and the rotor groove: radial play of the race
    play = 2 * (BALL['groove_r'] - BALL['d'] / 2)
    print(f'ball races: {BALL["n"]} x 2 balls dia {BALL["d"]:g} on r {BALL["pitch_r"]:g}, rows {BALL["rows"]}, groove play {play:.2f} mm '
          f'before preload ({PRELOAD} mm gap per split)')
    # 5) gear geometry
    cd = math.hypot(*PIN_C); res_g = f'gear: {ZR} / {ZP} T m{MOD:g}, centre distance {cd:.2f} (ideal {MOD * (ZR + ZP) / 2:.2f}), ratio {ZR / ZP:.1f}:1'
    pin_out = cd + MOD * ZP / 2 + MOD
    return res, res_g, pin_out


if __name__ == '__main__':
    import mono_ring as MR                     # one module instance (the export imports it too)
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join('..', '..', 'CAD', 'mono_ring_draft')
    tz = MR.build_all()
    print(f'upper-arm tube underside at carriage z {tz:.2f}')
    res, g, pin_out = MR.check()
    print(g); print(f'pinion reaches r {pin_out:.1f}; forearm prong sweeps r 92..168 above z {AZ - 33.6:.1f}; pinion top {GEAR_Z[1]}')
    print('problems:', res if res else 'none')
    from mono_ring_export import export
    export(out)
