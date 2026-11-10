"""SCARA arm (one arm) - printable parts, purchased-part models and an assembly with every fastener placed.
Frames (all Z-up, mm):
  ARM  : origin on the shoulder axis at the shoulder servo's mid-plane; X forward along the arm at zero pose.
         The carriage mount plate's top face is Z = -40.
  link frames rotate about Z at J1 (0,0,0), J2 (180,0,0), J3 (330,0,0).
  T    : tilt frame, origin on the pitch axis at W(40, 0, PZ); pitch rotates about Y.
  R    : roll frame, origin on the roll axis at T(85, 0, 0); roll rotates about X.
"""
import math
import numpy as np
from lib import M, box, cyl, cone, ring, union, xf, T, Rx, Ry, Rz, basis, mesh_arrays, CS
from hw import Reg, screw_cut, insert_cut, nut_cut, place, bearing_6808, mgn9_rail, mgn9h_block, SPEC
import joint
from joint import P, tongue, fork, pan_link, sweep, SERVO, HORN_HOLES, SO100, RIM, case_screw, CTR_D, CTR_TOP, OUT_FACE, OUT_CTR

L1, L2 = 130.0, 130.0      # 2026-11-09 (user): was 180 / 150; 130 / 130 lets the stock 20 cm servo cables reach
X0 = -46.0
PZ = -(-P['prongB'][0]) - 41.0          # pitch axis below J3: bottom prong underside (-33.6) minus housing height 41
F_PAN = 58.0
F_TILT = 46.0
ROLL_X = 85.0                           # roll axis origin forward of the pitch axis
G = 50.0                                # gripper servo shaft forward of the roll origin
RAIL_Z = 32.0
PIN_R = 12.0                            # pinion pitch radius (24 T, module 1)

# joint-frame -> part-frame maps
PITCH_J = basis((0, 0, -1), (1, 0, 0), (0, -1, 0))          # pitch servo frame in T
PITCH_C = basis((1, 0, 0), (0, 0, 1), (0, -1, 0))           # pitch child (tilt fork) modelling frame in T
# Roll and gripper servos are turned over (2026-11-06): their driven OUTPUT face (servo -Z, horn face at 18.7) points
# forward to the yoke / pinion, and the cable-socket face (+Z, the free idler) points back. The servo origin sits 1 mm
# behind ROLL_X / G so the output horn face lands where the horn face was before (17.7 ahead of ROLL_X / G).
ROLL_J = basis((0, 0, -1), (0, -1, 0), (-1, 0, 0), (ROLL_X - (OUT_FACE - 17.7), 0, 0))   # roll servo in T
GRIP_J = basis((0, 0, -1), (0, -1, 0), (-1, 0, 0), (G - (OUT_FACE - 17.7), 0, 0))        # gripper servo in R
SOCK_ROLL = ROLL_X - (OUT_FACE - 17.7) - RIM     # T x of the roll servo's socket-face rim (on the tilt-fork base)
SOCK_GRIP = G - (OUT_FACE - 17.7) - RIM          # R x of the gripper servo's socket-face rim (on the yoke back wall)

PARTS = {}      # name -> dict(solid, frame, color, qty, note, reg)


def add(name, solid, frame, color, reg=None, qty=1, note='', printed=True):
    PARTS[name] = dict(solid=solid, frame=frame, color=color, reg=reg or Reg(), qty=qty, note=note, printed=printed)


def reg_xf(reg, mat):
    out = Reg()
    for it in reg.items:
        it = dict(it)
        p = mat @ np.r_[it['p'], 1.0]; d = mat[:3, :3] @ np.asarray(it['d'])
        it['p'] = tuple(p[:3]); it['d'] = tuple(d); out.items.append(it)
    return out


# ------------------------------------------------------------------ base (shoulder) ----
BOARD_ZC = -17.0      # Waveshare plate centre on the pillar back face (plate 51 x 42 spans z -38..+4, y +-25.5)


def base_parts():
    for sign, nm in ((1, 'base_top'), (-1, 'base_bottom')):
        r = Reg()
        t = tongue(sign, x0=X0, bolts='down', slot='rear', reg=r)
        z0, z1 = (0.0, P['topT']) if sign > 0 else (-40.0, 0.0)
        pil = box(-85.0, X0 + 0.01, -P['WY'], P['WY'], z0, z1)
        s = union([t, pil])
        cut = []
        if sign > 0:
            cut.append(box(-86.0, -34.0, -7.0, 7.0, 8.0, 26.5))              # cable slot out of the back (1.25 mm web beside the M2s)
        else:
            cut.append(box(-86.0, -60.0, -8.0, 8.0, -1.0, 0.01))            # cable groove continues
            # dovetail socket for the SO-101 Waveshare mounting plate on the pillar's back face, right under the cable
            # slot (the cables come out of the slot and plug into the board). Its 18 mm square peg tapers 15.9 -> 18.1 mm
            # away from the plate; the socket is open toward +Y, the plate slides in sideways (no disassembly).
            zc = BOARD_ZC
            dv = CS([[(-1.0, zc - 8.15), (1.0, zc - 8.15), (2.6, zc - 9.25), (4.0, zc - 9.25),
                      (4.0, zc + 9.25), (2.6, zc + 9.25), (1.0, zc + 8.15), (-1.0, zc + 8.15)]])
            dv = xf(dv.extrude(30.75).translate((0, 0, -21.5)), basis((1, 0, 0), (0, 0, 1), (0, -1, 0), (-85.0, 0, 0)))
            cut.append(dv)
        for y in (-14.5, 14.5):                                              # two more clamshell bolts in the pillar (+-14.5: 2 mm from the board dovetail)
            if sign > 0:
                cut.append(screw_cut((-74.0, y, z1 - 6.0), (0, 0, -1), 'M3', z1 - 6.0 + 0.2, cb=6.2))   # 6 mm recess: full insert
                r.add('SHCS', 'M3', (-74.0, y, z1 - 6.0), (0, 0, -1), 35, 'clamshell')
            else:
                cut.append(insert_cut((-74.0, y, 0.0), (0, 0, -1), 'M3', length=8))
                r.add('insert', 'M3', (-74.0, y, 0.0), (0, 0, -1), 8, 'clamshell')
        if sign < 0:                                                         # M5 heat-set inserts for the mount plate
            for (x, y) in ((-80.0, 13.5), (-80.0, -13.5), (-54.0, 13.5), (-54.0, -13.5)):
                cut.append(insert_cut((x, y, -40.0), (0, 0, 1), 'M5', length=8))
                r.add('insert', 'M5', (x, y, -40.0), (0, 0, 1), 8, 'mount plate')
                r.add('SHCS', 'M5', (x, y, -48.0), (0, 0, 1), 16, 'from under the 8 mm mount plate')
        s = s - union(cut)
        add(nm, s, np.eye(4), (0.93, 0.55, 0.20) if sign > 0 else (0.96, 0.66, 0.30), r,
            note='shoulder servo clamshell + pillar; prints split-face down')


def link_parts(name, L, frame):
    for sign, sfx in ((1, '_top'), (-1, '_bottom')):
        r = Reg()
        s = pan_link(sign, L, F=F_PAN, reg=r, x0=X0)
        add(name + sfx, s, frame, (0.25, 0.48, 0.80) if sign > 0 else (0.33, 0.56, 0.86), r,
            note=f'link {L:.0f} mm: hub/fork at the near joint, servo clamshell at the far joint; prints split-face down')


# ------------------------------------------------------------------ wrist bracket (J3 child) + pitch housing ----
WRIST_CABLE_Y = -24.0           # wrist pan -> pitch cable down the wrist bracket's front face


def wall_clip(x_face, y, z, out=1):
    """Cable clip on a face normal to +X at x_face, cable running along Z at y, between z[0] and z[1]."""
    (hw, h), (w, r) = joint.GUIDE_IN, joint.GUIDE_WALL
    return (box(x_face - 0.01, x_face + h + r, y - hw - w, y + hw + w, z[0], z[1])
            - box(x_face - 1, x_face + h, y - hw, y + hw, z[0] - 1, z[1] + 1))


def wrist_parts(frame):
    pin = [(26.0, -22.0), (26.0, 22.0), (54.0, -22.0), (54.0, 22.0)]      # pitch housing screws, W frame
    for sign, nm in ((1, 'wrist_top'), (-1, 'wrist_bottom')):
        r = Reg()
        s = fork(sign, F=F_PAN, reg=r)
        # cable clips on the fork base's front face (wrist pan -> pitch cable: from the prong-top arches down the
        # front, offset to -y so the camera clears it at full tilt, into the pitch servo's chamber through its front)
        if sign < 0:   # both on the bottom half: a clip on the top half meets the camera at -20 deg pitch
            s = union([s, wall_clip(F_PAN + P['baseT'], WRIST_CABLE_Y, (-8.0, -3.0))])   # (a lower one meets the roll clamp at -20 deg)
        if sign < 0:
            cut = []
            for (x, y) in pin:
                seat = P['prongB'][1] - 3.2
                cut.append(screw_cut((x, y, seat), (0, 0, -1), 'M3', seat - P['prongB'][0] + 0.2, cb=3.3))
                r.add('SHCS', 'M3', (x, y, seat), (0, 0, -1), 8, 'pitch housing')
            s = s - union(cut)
        add(nm, s, frame, (0.20, 0.60, 0.55) if sign > 0 else (0.27, 0.67, 0.60), r,
            note='wrist-pan fork; the pitch housing hangs under the bottom prong')
    tw = frame @ T(40.0, 0.0, PZ)
    for sign, nm in ((1, 'pitch_housing_horn'), (-1, 'pitch_housing_idler')):
        r = Reg()
        t = tongue(sign, x0=-41.0, bolts='up', slot='side', reg=r, bolt_rear=False, ins_len=6)
        s = xf(t, PITCH_J)                                             # into T frame
        rr = reg_xf(r, PITCH_J)
        cut = []
        for (x, y) in pin:                                             # inserts in the top face (W frame -> T frame)
            xt, yt = x - 40.0, y
            if (yt < 0) == (sign > 0):                                 # horn side is -Y
                cut.append(insert_cut((xt, yt, 41.0), (0, 0, -1), 'M3', depth=6.0, length=4))
                rr.add('insert', 'M3', (xt, yt, 41.0), (0, 0, -1), 4, 'to wrist bracket')
        s = s - union(cut)
        add(nm, s, tw, (0.80, 0.35, 0.35) if sign > 0 else (0.86, 0.45, 0.42), rr,
            note='pitch-servo clamshell (STS3215, SO-100 socket); split plane is vertical')


# ------------------------------------------------------------------ tilt fork with the roll-servo clamp ----
ROLL_SERVO_T = xf(SERVO, ROLL_J)


def tilt_parts(frame):
    tw = frame
    # clamp around the top 12 mm of the roll servo (outside the yoke's 22 mm swing radius)
    clamp = box(F_TILT, 102.0, -18.6, 18.6, -14.4, 42.0)
    rot = Rx(90)                                                        # T -> sweep frame (Y_T becomes Z)
    for sign, nm in ((1, 'tilt_fork_horn'), (-1, 'tilt_fork_idler')):
        r = Reg()
        f = fork(sign, F=F_TILT, reg=r)
        s = xf(f, PITCH_C); rr = reg_xf(r, PITCH_C)
        half_y = (-30.0, 0.0) if sign > 0 else (0.0, 30.0)               # horn side half is -Y_T
        c = clamp ^ box(40, 110, half_y[0], half_y[1], -20, 50)
        if sign > 0:   # cable clip on the clamp's -Y side (roll -> gripper cable, from the clamp-top channel to the roll gap)
            c = union([c, box(84.0, 89.0, -18.6 - 8.0, -18.59, 20.0, 32.0) - box(83.0, 90.0, -18.6 - 6.0, -18.5, 22.0, 30.0)])
        # camera fork cheek, grown out of this half's side of the clamp (no separate camera post since 2026-11-09)
        ys = -1 if sign > 0 else 1
        cy0, cy1 = sorted((ys * CHEEK[0], ys * CHEEK[1])); ry0, ry1 = sorted((ys * 18.59, ys * (CHEEK[0] + 0.01)))
        c = union([c, box(76.0, 94.0, cy0, cy1, 34.0, CAM_PIVOT[2] + 8.0), box(76.0, 94.0, ry0, ry1, 34.0, 42.0)])
        exit_sign = +1 if sign > 0 else -1                              # this half pulls away along -/+Y; servo exits toward +/-Y in sweep frame
        # in sweep frame Z = Y_T; the -Y half is removed toward -Y, i.e. servo exits toward +Z relative to it
        sw = sweep(xf(ROLL_SERVO_T, rot), +1 if sign > 0 else -1, -20.0 if sign > 0 else 0.0, 0.0 if sign > 0 else 20.0)
        pocket = xf(sw, Rx(-90))
        s = union([s, c]) - pocket
        recut = []
        for it in rr.items:                      # the clamp block refilled the fork-base bolt holes: cut them again
            if it['note'] == 'fork base':
                if it['kind'] == 'SHCS':
                    recut.append(screw_cut(it['p'], it['d'], 'M3', 36.0, cb=4.0))
                else:
                    recut.append(insert_cut(it['p'], it['d'], 'M3', length=it['length']))
        s = s - union(recut)
        # cable channel from the sockets (now at the back, T x 64.6..68.1, z ~14) up through the top of the clamp
        cut = [box(63.5, SOCK_ROLL + 0.1, -8.8, 8.8, 11.5, 17.0), box(63.5, SOCK_ROLL + 0.1, -6.0, 6.0, 11.5, 42.5)]
        # SO-100 case screws into the roll servo's socket-face holes (-32.8 / -8.3, +-10.25) -> T, from behind through
        # the fork base. The socket face sits flush on the fork base (line-to-line pocket), 2.0 mm wall under the head.
        for yy in (-10.25, 10.25):
            if (yy < 0) == (sign > 0):
                for zz in (32.8, 8.3):
                    seat = SOCK_ROLL - SO100['cz_horn'] - SO100['wall']
                    cut.append(case_screw((seat, yy, zz), (1, 0, 0), rr, 'roll servo case (SO-100)', cb=seat - F_TILT + 0.5))
        # lateral clamp bolts (heads on the idler/+Y side, inserts in the horn/-Y side)
        for (x, z) in ((75.0, 38.7), (96.0, 38.7), (56.0, 24.0), (61.0, -9.0)):     # (56, 24): clear of the cable channel
            if sign < 0:
                cut.append(screw_cut((x, 18.6 - 3.0, z), (0, -1, 0), 'M3', 15.8, cb=3.2))
                rr.add('SHCS', 'M3', (x, 18.6 - 3.0, z), (0, -1, 0), 20, 'roll clamp')
            else:
                cut.append(insert_cut((x, 0.0, z), (0, -1, 0), 'M3', length=4))
                rr.add('insert', 'M3', (x, 0.0, z), (0, -1, 0), 4, 'roll clamp')
        # camera tilt bolt through the cheek (one M3 x 55 through both cheeks and the cradle tube, nyloc nut)
        cut.append(xf(cyl(1.7, -30, 30, hole=True), T(CAM_PIVOT[0], 0, CAM_PIVOT[2]) @ Rx(90)))
        if sign < 0:
            rr.add('SHCS', 'M3', (CAM_PIVOT[0], CHEEK[1], CAM_PIVOT[2]), (0, -1, 0), 55, 'camera tilt bolt (both cheeks + cradle tube, nyloc nut)')
            rr.add('nut', 'M3', (CAM_PIVOT[0], -CHEEK[1], CAM_PIVOT[2]), (0, -1, 0), None, 'camera tilt bolt (nyloc)')
        s = s - union(cut)
        add(nm, s, tw, (0.55, 0.42, 0.75) if sign > 0 else (0.62, 0.50, 0.80), rr,
            note='tilt fork half: hub/prong on the pitch servo + half of the roll-servo clamp; split plane is vertical')


# ------------------------------------------------------------------ wrist camera: OV 32 x 32 board on a tilting cradle ----
# User's camera (2026-11-09): 32 x 32 mm OV module, 4 mounting holes on a 28 x 28 mm square. ONE screw (user): an M3 bolt
# through both cheeks (grown out of the two tilt-fork halves) and through a tube on the back of the cradle; loosen it to tilt, tighten
# it and the cheeks clamp the tube ends (friction holds the tilt). Tilt CAM_TILT_RANGE (0 = straight ahead along the
# gripper, + = down). High enough to look over the gripper rail to the fingertips.
CAM_PIVOT = (85.0, 0.0, 110.0)  # pivot axis (along Y) in T
CAM_P = 16.0                    # pivot (the tube behind the back plate) this far behind the board's front face
CAM_TILT = 35.0                 # default tilt (deg down), used for the URDF camera frame
CAM_TILT_RANGE = (0.0, 60.0)
CAM_C = (CAM_PIVOT[0] + CAM_P * math.cos(math.radians(CAM_TILT)), 0.0,
         CAM_PIVOT[2] - CAM_P * math.sin(math.radians(CAM_TILT)))   # camera reference: front face of the board, in T
CHEEK = (20.6, 25.6)            # fork cheek |y| (the tube spans |y| <= 20.4: 0.2 mm each side, closed by the bolt)


def cam_frame(tilt=CAM_TILT):
    """Camera frame (x = optical axis, y lateral, z up on the board) in T for a given tilt."""
    return T(*CAM_PIVOT) @ Ry(tilt) @ T(CAM_P, 0, 0)


def camera_board():
    """Approximate envelope of the OV 32 x 32 module in the camera frame: board, lens, parts on the back."""
    return union([box(-1.6, 0.0, -16, 16, -16, 16), xf(cyl(8.0, 0.0, 16.0), basis((0, 0, -1), (0, 1, 0), (1, 0, 0))),
                  box(-7.8, -1.6, -11, 11, -11, 11)])


def camera_parts(frame):
    """The cradle only: the fork cheeks are part of the tilt-fork halves (tilt_parts)."""
    # cradle, in the camera frame: back plate with a window, 4 standoffs on the 28 x 28 pattern (M2 self-tapping),
    # side walls, and the pivot tube behind the plate that the tilt bolt passes through
    rc = Reg()
    xp = -CAM_P
    body = union([box(-12.0, -9.0, -18.0, 18.0, -18.0, 18.0)]
                  + [xf(cyl(2.8, -9.01, -1.6), T(0, sy * 14.0, sz * 14.0) @ Ry(90)) for sy in (-1, 1) for sz in (-1, 1)]
                  + [box(-12.0, -1.6, lo, hi, -18.0, 18.0) for lo, hi in ((17.0, 20.4), (-20.4, -17.0))]
                  + [xf(cyl(4.6, -20.4, 20.4), T(xp, 0, 0) @ Rx(-90)), box(xp, -11.99, -20.4, 20.4, -3.0, 3.0)])
    hcut = [box(-13.0, -8.0, -11.0, 11.0, -11.0, 11.0),                                     # window for the back parts / plug
            xf(cyl(1.7, -21, 21, hole=True), T(xp, 0, 0) @ Rx(-90))]                        # tilt bolt
    for sy in (-1, 1):
        for sz in (-1, 1):
            hcut.append(xf(cyl(0.8, -13.0, -1.0, hole=True), T(0, sy * 14.0, sz * 14.0) @ Ry(90)))
            rc.add('selftap', 'M2', (0.0, sy * 14.0, sz * 14.0), (-1, 0, 0), 8, 'camera board (from the front, into the standoff)')
    body = body - union(hcut)
    add('camera_cradle', xf(body, cam_frame()), frame, (0.35, 0.35, 0.40), reg_xf(rc, cam_frame()),
        note='holds the 32 x 32 OV camera board (4 x M2 on 28 x 28); tilts 0..60 deg on one M3 bolt')


# ------------------------------------------------------------------ roll yoke + gripper housing ----
GRIP_SERVO_R = xf(SERVO, GRIP_J)
FP_Y = 16.1                     # front-plate screw / insert |y| (was 15.6: 1 mm insert wall)


def yoke_part(frame):
    r = Reg()
    W = 18.6
    front = union([cyl(22.0, 28.0, 34.0) if False else xf(cyl(22.0, 28.0, 34.0), np.eye(4)), ])
    # build in R frame (x forward). helper: disc on X axis
    def xdisc(rad, x0, x1, y=0.0, z=0.0, hole=False):
        return xf(cyl(rad, x0, x1, hole=hole), T(0, y, z) @ Ry(90) @ Rx(0)) if False else xf(cyl(rad, x0, x1, hole=hole), basis((0, 0, -1), (0, 1, 0), (1, 0, 0), (0, y, z)))
    parts = [
        xdisc(9.8, 17.7, 18.45), xdisc(11.0, 18.4, 28.05),                           # pilot + boss on the roll horn
        xdisc(22.0, 28.0, 34.0), box(28.0, 34.0, -W, W, -14.4, 39.4),               # front plate = housing back wall
        box(34.0 - 0.01, 66.0, 12.6, W, -14.4, 39.4), box(34.0 - 0.01, 66.0, -W, -12.6, -14.4, 39.4),   # side walls
        box(34.0 - 0.01, 66.0, -W, W, 35.4, 39.4), box(34.0 - 0.01, 66.0, -W, W, -14.4, -10.4),          # top / bottom walls
    ]
    s = union(parts)
    cut = []
    cut.append(xdisc(CTR_D / 2, 17.6, 17.7 + (OUT_CTR - OUT_FACE), hole=True))       # roll output nub + centre screw head
    cut.append(box(27.0, SOCK_GRIP + 0.1, -8.8, 8.8, 11.5, 17.0))                   # gripper cable channel through the back wall
    # gripper servo pocket: servo slides out forward
    sw = sweep(xf(GRIP_SERVO_R, Ry(-90)), +1, 28.5, 70.0)
    cut.append(xf(sw, Ry(90)))
    # roll horn screws (from inside the housing, before the gripper servo goes in)
    for (a, b) in HORN_HOLES:                       # horn frame (x_s, y_s) -> R: z = -x_s, y = y_s
        y, z = b, -a
        seat = 25.6
        cut.append(screw_cut((seat, y, z), (-1, 0, 0), 'M3', seat - 17.7, cb=34.0 - seat + 0.6))
        r.add('SHCS', 'M3', (seat, y, z), (-1, 0, 0), 10, 'roll horn')
    # SO-100 case screws into the gripper servo's socket-face holes (-32.8, +-10.25): servo (x, y, z) -> R (.., -y, -x).
    # Driven from behind through the back wall; reach them with the roll turned so the hole clears the roll servo.
    for (xs, ys) in SO100['horn']:
        seat = SOCK_GRIP - SO100['cz_horn'] - SO100['wall']
        cut.append(case_screw((seat, -ys, -xs), (1, 0, 0), r, 'gripper servo case (SO-100)', cb=seat - 28.0 + 0.5))
    # front-plate inserts in the housing walls
    # front-plate inserts: 0.5 mm further out than the wall centre + a 1.2 mm bump outside, so >= 1.5 mm of wall all round
    s = union([s] + [box(58.0, 66.0, min(sy * 18.59, sy * 19.8), max(sy * 18.59, sy * 19.8), z - 4.0, z + 4.0)
                     for sy in (-1, 1) for z in (-8.0, 30.0)])
    for y in (-FP_Y, FP_Y):
        for z in (-8.0, 30.0):
            cut.append(insert_cut((66.0, y, z), (-1, 0, 0), 'M3', length=4))
            r.add('insert', 'M3', (66.0, y, z), (-1, 0, 0), 4, 'gripper front plate')
    s = s - union(cut)
    add('roll_yoke_gripper_housing', s, frame, (0.92, 0.78, 0.25), r,
        note='bolts to the roll servo horn, holds the gripper servo; servo slides in from the front')


def front_plate_part(frame):
    r = Reg()
    s = union([box(66.0, 74.0, -18.6, 18.6, -14.4, 39.4), box(66.0, 74.0, -85.0, 85.0, RAIL_Z - 8.0, RAIL_Z + 8.0)])
    cut = [xf(cyl(11.2, 65, 75, hole=True), basis((0, 0, -1), (0, 1, 0), (1, 0, 0))),
           ]                                                                          # (no connector window: the sockets face back now)
    s = union([s] + [box(66.0, 74.0, min(sy * 18.59, sy * 19.8), max(sy * 18.59, sy * 19.8), z - 4.0, z + 4.0)
                     for sy in (-1, 1) for z in (-8.0, 30.0)])         # matching bumps around the screw holes
    for y in (-FP_Y, FP_Y):
        for z in (-8.0, 30.0):
            cut.append(screw_cut((71.0, y, z), (-1, 0, 0), 'M3', 5.2, cb=3.1))
            r.add('SHCS', 'M3', (71.0, y, z), (-1, 0, 0), 10, 'front plate to housing')
    _, ys = mgn9_rail(170.0)
    for y in ys:
        if abs(y) < 30:
            continue
        cut.append(screw_cut((74.0, y, RAIL_Z), (-1, 0, 0), 'M3', 8.5))
        cut.append(nut_cut((66.0, y, RAIL_Z), (1, 0, 0)))
        r.add('SHCS', 'M3', (77.0, y, RAIL_Z), (-1, 0, 0), 12, 'MGN9 rail')
        r.add('nut', 'M3', (66.0, y, RAIL_Z), (1, 0, 0), None, 'MGN9 rail (nut trap)')
    cut.append(xf(sweep(xf(GRIP_SERVO_R, Ry(-90)), -1, 60.0, 75.0), Ry(90)))       # horn-side plateau, horn, connectors
    s = s - union(cut)
    add('gripper_front_plate', s, frame, (0.30, 0.30, 0.34), r, note='closes the gripper housing and carries the 170 mm MGN9 rail')


# ------------------------------------------------------------------ pinion, racks, finger carriers, fingers ----
def involute_gear(z, mod, pa=20.0, n=6):
    """2D spur gear outline (CrossSection) with true involute flanks."""
    rp = z * mod / 2; rb = rp * math.cos(math.radians(pa)); ra = rp + mod; rf = rp - 1.25 * mod
    def inv(a): return math.tan(a) - a
    tooth_half = math.pi / (2 * z) + inv(math.radians(pa))       # half angular thickness at base circle
    pts = []
    for k in range(z):
        c = 2 * math.pi * k / z
        flank = []
        for i in range(n + 1):
            r_ = rb + (ra - rb) * i / n if rb > rf else rf + (ra - rf) * i / n
            r_ = max(r_, rb)
            a = math.acos(rb / r_); th = tooth_half - inv(a)
            flank.append((r_, th))
        left = [(r_ * math.cos(c - th), r_ * math.sin(c - th)) for r_, th in flank]
        right = [(r_ * math.cos(c + th), r_ * math.sin(c + th)) for r_, th in reversed(flank)]
        a0 = c - tooth_half - 0.0; a1 = c + tooth_half
        pts += [(rf * math.cos(c - math.pi / z + 0.0001), rf * math.sin(c - math.pi / z + 0.0001)),
                (rf * math.cos(c - tooth_half), rf * math.sin(c - tooth_half))] + left + right + \
               [(rf * math.cos(c + tooth_half), rf * math.sin(c + tooth_half))]
    return CS([pts])


def pinion_part(frame):
    r = Reg()
    gear = involute_gear(24, 1.0).extrude(8.0)                   # z 0..8
    g = union([gear.translate((0, 0, 74.5)), cyl(10.6, G + 17.7, 74.6), cyl(9.8, G + 17.7, G + 18.4)])
    cut = [cyl(CTR_D / 2, G + 17.6, G + 17.7 + (OUT_CTR - OUT_FACE), hole=True)]   # gripper output nub + centre screw head
    for (a, b) in HORN_HOLES:
        seat = G + 25.6
        cut.append(screw_cut((a, b, seat), (0, 0, -1), 'M3', seat - (G + 17.7), cb=82.5 - seat + 0.5))
        r.add('SHCS', 'M3', (a, b, seat), (0, 0, -1), 10, 'pinion to gripper horn')
    g = g - union(cut)
    # modelled along Z; place: Z -> R +X (shaft forward), x_s -> -z
    pf = basis((0, 0, -1), (0, 1, 0), (1, 0, 0))
    add('pinion_24T_m1', g, frame @ pf, (0.85, 0.85, 0.88), r, note='24 T module 1 spur pinion on the gripper servo horn')


def carrier_part(side, frame, opening=60.0):
    """side=+1: finger on +Y (driven by the bottom rack); side=-1: mirror (top rack). Modelled at opening 0."""
    r = Reg()
    yP = 7.0                                                     # finger centre at zero opening
    zr = -1 if side > 0 else 1                                   # rack below (+Y finger) / above (-Y finger)
    plate = union([box(84.0, 92.0, yP - 7.0, yP + 7.0, -20.0, 42.0), box(84.0, 92.0, yP - 7.0, yP + 33.5, 22.0, 42.0)])
    # rack: teeth toward the pinion, pitch line at |z| = PIN_R
    rk = rack_cs(yP - 60.0, yP + 5.0)                            # in (y, z) with teeth pointing -z, pitch line at z = 0
    rk3 = rk.extrude(8.0)                                        # (y, z, x) -> map to R
    rk3 = xf(rk3, basis((0, 1, 0), (0, 0, 1), (1, 0, 0), (74.5, 0, PIN_R)))   # teeth point down from z=PIN_R
    if zr < 0:
        rk3 = xf(rk3, np.diag([1.0, 1.0, -1.0, 1.0]))            # mirror to the underside of the pinion
    bridge = box(82.4, 84.01, yP - 7.0, yP + 5.0, *((PIN_R + 1.25, PIN_R + 8.25) if zr > 0 else (-PIN_R - 8.25, -PIN_R - 1.25)))
    s = union([plate, rk3, bridge])
    cut = []
    for dy in (-8.0, 8.0):
        for dz in (-7.5, 7.5):
            p = (92.0 - 3.0, yP + 13.5 + dy, RAIL_Z + dz)
            cut.append(screw_cut((89.0, yP + 13.5 + dy, RAIL_Z + dz), (-1, 0, 0), 'M3', 5.2, cb=3.1))
            r.add('SHCS', 'M3', (89.0, yP + 13.5 + dy, RAIL_Z + dz), (-1, 0, 0), 8, 'carrier to MGN9H')
    for z in (-10.0, 10.0):
        cut.append(screw_cut((87.0, yP, z), (1, 0, 0), 'M3', 5.2, cb=3.1))
        r.add('SHCS', 'M3', (87.0, yP, z), (1, 0, 0), 10, 'finger')
    s = s - union(cut)
    if side < 0:
        mir = np.diag([1.0, -1.0, 1.0, 1.0]); mir2 = np.diag([1.0, 1.0, -1.0, 1.0])
        s = xf(s, mir)
        # the -Y finger's rack must be on top: mirror the rack position in z too, but keep the plate
        r = reg_xf(r, mir)
    return s, r


def rack_cs(y0, y1, mod=1.0, pa=20.0):
    """Rack outline in (u=y, v=z) with teeth hanging toward -v; pitch line v=0; body up to v=+8.25."""
    pitch = math.pi * mod; ha = mod; hf = 1.25 * mod
    t = math.tan(math.radians(pa))
    pts = [(y1, 8.25), (y0, 8.25)]
    u = y0 - (y0 % pitch)
    seq = []
    while u < y1 + pitch:
        # one tooth centred at u: thickness pitch/2 at the pitch line
        seq += [(u - pitch / 4 - ha * t, hf * 0 + 1.25 * mod), (u - pitch / 4 + ha * t * 0 - 0 + (ha) * t * 0, 1.25 * mod)]
        u += pitch
    # build explicitly
    pts = []
    u = math.floor(y0 / pitch) * pitch + pitch / 2
    prof = []
    while u <= y1 + pitch:
        a = u - pitch / 4                                       # tooth flank at pitch line
        b = u + pitch / 4
        prof += [(a - hf * t, hf), (a + ha * t, -ha), (b - ha * t, -ha), (b + hf * t, hf)]
        u += pitch
    prof = [(min(max(p[0], y0), y1), p[1]) for p in prof]
    poly = [(y0, 8.25), (y0, hf)] + prof + [(y1, hf), (y1, 8.25)]
    # convert to tooth-down convention: teeth reach v=-ha, root v=+hf
    cs = CS([poly[::-1]]) if CS([poly]).area() < 0 else CS([poly])
    return cs


def finger_part(frame_fn):
    r = Reg()
    # wedge finger, inner (grip) face at y = 0, body toward +y, root x 92 .. tip 142
    prof = CS([[(92.0, -20.0), (142.0, -6.0), (142.0, 6.0), (92.0, 20.0)]])
    w = prof.extrude(14.0)                                         # (x, z, y)
    w = xf(w, basis((1, 0, 0), (0, 0, 1), (0, 1, 0)))
    w = w - box(108.0, 140.0, -0.01, 1.6, -9.0, 9.0)               # pocket for a 1.5 mm rubber grip pad
    cut = []
    for z in (-10.0, 10.0):
        cut.append(insert_cut((92.0, 7.0, z), (1, 0, 0), 'M3', length=6))
        r.add('insert', 'M3', (92.0, 7.0, z), (1, 0, 0), 6, 'finger')
    return w - union(cut), r
