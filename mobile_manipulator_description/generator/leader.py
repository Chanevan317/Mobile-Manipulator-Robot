"""Mini leader arm (2026-11-09): a 0.4-scale copy of the SCARA follower for teleoperation, one per follower arm.

Same joints in the same order, angles 1:1: shoulder pan, elbow pan, wrist pan (Z), wrist pitch (Y), wrist roll (X),
plus a gripper trigger. Every joint has an AS5600 magnetic encoder (Robokits RKI-6142 module, ~23 x 23 mm board, magnet
Ø6 x 2.5 mm diametric, included). An ESP32-S3 in the base reads the analog outputs. Board size, chip height and the
magnet are parameters (BOARD, MAG): measure a real module and update them before printing.

Fork joint (pan joints and pitch), local frame: axis = +Z, origin at the middle of the fixed hub.
  fixed hub     Ø28 x 12 (z -6..6) on the parent part; M5 bolt through it (head below, nyloc nut on top), fixed
  moving fork   prongs above / below the hub (|z| 6.5..12.5) on the child part, each with a 685ZZ bearing (5 x 11 x 5)
                on the bolt; 1 mm bosses / spacers clamp the inner races to the hub
  encoder       magnet in a printed cap pressed on the fixed nut; AS5600 board in a housing on the moving upper prong,
                chip facing down 1.5 mm over the magnet; lid with a wire slot
  hold          M3 set screw down through the upper prong onto a Ø6 felt / PTFE pad on the hub's top face
Inline joint (roll), local frame: axis = +X, x = 0 at the front face of the fixed bearing tube.
  the grip's M5 bolt runs back through two 685ZZ in the tube; nut + magnet cap at the back turn with the grip; the board
  sits fixed behind them in a slot that opens to +Y (slide in, clip shut); hold pad from an M3 set screw in the tube.

Run:  python3 leader.py      (volumes)    /   leader_check.py (clashes, ranges, export, renders)
"""
import sys, os, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import box, cyl, union, xf, T, Rx, Ry, Rz

# ------------------------------------------------------------------ parameters (mm)
SCALE = 0.4
L1, L2 = 130.0 * SCALE, 130.0 * SCALE          # 52, 52
BOARD = dict(w=23.0, t=1.6, clear=0.25, chip_h=1.5, ledge=1.2)   # AS5600 module
MAG = dict(d=6.0, h=2.5, gap=1.5)              # diametric magnet Ø6 x 2.5, 1.5 mm air gap to the chip
BRG = dict(od=11.0, w=5.0)                     # 685ZZ (5 x 11 x 5)
HUB_R, HUB_H = 14.0, 6.0
PRONG = (6.5, 12.5)
NUT = dict(af=8.0, h=5.0)                      # M5 nyloc
HEAD = dict(d=8.5, h=5.0)                      # M5 socket head
WALL = 2.0
LIMITS = {'J1': (-115, 115), 'J2': (-125, 125), 'J3': (-122, 122), 'pitch': (-15, 115), 'roll': (-120, 120)}
PITCH_X = 52.0                                 # pitch axis forward of J3 (44 let the pitch lid touch the forearm at J3 +120)
ROLL_X = 66.0                                  # roll tube front face forward of the pitch axis
GRIP = dict(x0=10.0, x1=100.0, w=30.0, h=34.0, wall=2.5) # grip box in the grip frame (x from the roll tube front face)
TRIG_PIVOT = (80.0, 0.0, -12.0)                # trigger pivot (axis Y) in the grip frame

# fork-joint stack along the axis
Z_NUT = PRONG[1]                               # nut on a 1 mm spacer above the upper bearing (6.5..11.5)
Z_CAP0 = Z_NUT + 0.1
Z_MAG1 = Z_NUT + NUT['h'] + 0.8 + 1.0 + MAG['h']
Z_CHIP = Z_MAG1 + MAG['gap']
Z_BOARD0 = Z_CHIP + BOARD['chip_h']
Z_BOARD1 = Z_BOARD0 + BOARD['t']
Z_LID1 = Z_BOARD1 + 2.0
HOUSING = BOARD['w'] + 2 * BOARD['clear'] + 2 * WALL
C_IN = BOARD['w'] / 2 + BOARD['clear']


def m5(lo, hi):
    return cyl(2.65, lo, hi, hole=True)


# ------------------------------------------------------------------ fork joint pieces (local frame, axis +Z)
def hub():
    s = union([cyl(HUB_R, -HUB_H, HUB_H), cyl(3.5, HUB_H - 0.01, PRONG[0]), cyl(3.5, -PRONG[0], -HUB_H + 0.01)])
    return s - m5(-20, 20)


def fork(reach, W=10.0, r_web=17.0):
    """Moving prongs, the web joining them at r >= 17, and the board housing on top. Child side; reach along +X."""
    plates = []
    for z0, z1 in ((PRONG[0], PRONG[1]), (-PRONG[1], -PRONG[0])):
        plates += [cyl(HUB_R, z0, z1), box(0, reach, -W, W, z0, z1)]
    web = box(r_web, reach, -W, W, -PRONG[1], PRONG[1])
    h = HOUSING / 2
    housing = box(-h, h, -h, h, PRONG[1] - 0.01, Z_BOARD1)
    s = union(plates + [web, housing])
    cut = [cyl(BRG['od'] / 2, PRONG[0] - 0.01, PRONG[0] + BRG['w'], hole=True),
           cyl(BRG['od'] / 2, -PRONG[0] - BRG['w'], -PRONG[0] + 0.01, hole=True),
           cyl(4.6, PRONG[0] + BRG['w'] - 0.01, PRONG[1] + 0.1, hole=True),
           cyl(4.6, -PRONG[1] - 0.1, -PRONG[0] - BRG['w'] + 0.01, hole=True),
           cyl(8.0, PRONG[1] - 0.02, Z_BOARD0 + 0.01, hole=True),                                   # magnet-cap cavity
           box(-C_IN, C_IN, -C_IN, C_IN, Z_BOARD0, Z_BOARD1 + 0.01),                                # board on its ledge
           box(-C_IN + BOARD['ledge'], C_IN - BOARD['ledge'], -C_IN + BOARD['ledge'], C_IN - BOARD['ledge'], Z_CHIP - 0.5, Z_BOARD0 + 0.01),
           cyl(1.25, PRONG[0] - 0.1, Z_BOARD1 + 0.1, y=-10.5, hole=True),                           # hold set screw
           cyl(3.1, PRONG[0] - 0.1, PRONG[0] + 2.0, y=-10.5, hole=True)]                            # pad pocket
    for sx, sy in ((1, 1), (-1, -1)):                                                                # lid screws M2
        cut.append(cyl(0.8, Z_BOARD1 - 4.0, Z_BOARD1 + 0.1, x=sx * (h - 1.1), y=sy * (h - 1.1), hole=True))
    return s - union(cut)


def lid():
    h = HOUSING / 2
    s = box(-h, h, -h, h, Z_BOARD1, Z_LID1) - box(h - 7.0, h + 1, -7.0, 7.0, Z_BOARD1 - 1, Z_LID1 + 1)
    for sx, sy in ((1, 1), (-1, -1)):
        s = s - cyl(1.1, Z_BOARD1 - 1, Z_LID1 + 1, x=sx * (h - 1.1), y=sy * (h - 1.1), hole=True)
    return s


def magnet_cap():
    r_hex = NUT['af'] / 2 / math.cos(math.pi / 6) + 0.15
    return cyl(7.0, Z_CAP0, Z_MAG1) - union([cyl(r_hex, Z_CAP0 - 0.1, Z_CAP0 + NUT['h'] + 0.6, n=6),
                                            cyl(MAG['d'] / 2 + 0.1, Z_MAG1 - MAG['h'], Z_MAG1 + 0.1, hole=True)])


def fork_hw():
    """(hardware fixed to the hub, magnet, board) in the local frame."""
    hw = union([cyl(BRG['od'] / 2 - 0.05, PRONG[0] + 0.05, PRONG[0] + BRG['w'] - 0.05),
                cyl(BRG['od'] / 2 - 0.05, -PRONG[0] - BRG['w'] + 0.05, -PRONG[0] - 0.05),
                cyl(2.4, -PRONG[1] - 1, Z_NUT + NUT['h']), cyl(HEAD['d'] / 2, -PRONG[1] - 1 - HEAD['h'], -PRONG[1] - 1),
                cyl(NUT['af'] / 2 / math.cos(math.pi / 6), Z_NUT, Z_NUT + NUT['h'], n=6)])
    mag = cyl(MAG['d'] / 2, Z_MAG1 - MAG['h'], Z_MAG1)
    brd = union([box(-BOARD['w'] / 2, BOARD['w'] / 2, -BOARD['w'] / 2, BOARD['w'] / 2, Z_BOARD0, Z_BOARD1),
                 box(-2.5, 2.5, -2.5, 2.5, Z_CHIP + 0.05, Z_BOARD0)])
    return hw, mag, brd


# ------------------------------------------------------------------ inline joint (roll): local frame, axis +X
TUBE_R = 9.5
X_B1 = -BRG['w']                        # front bearing  -5..0
X_B2 = (-16.0, -11.0)                   # rear bearing
X_NUT = (-22.0, -17.0)                  # nut on a 1 mm spacer behind the rear bearing (turns with the grip)
X_MAG0 = X_NUT[0] - 3.9 - 1.0 - MAG['h']          # magnet back face (cap clears the bolt tip, 1 mm floor)
X_CHIP = X_MAG0 - MAG['gap']
X_BOARD = (X_CHIP - BOARD['chip_h'] - BOARD['t'], X_CHIP - BOARD['chip_h'])


def roll_tube():
    """Fixed bearing tube + board housing (parent side, pitch link). Slot opens to +Y for the board."""
    h = HOUSING / 2
    s = union([xf(cyl(TUBE_R, X_NUT[1] + 0.01, 0.0), Ry(90)),
               box(X_BOARD[0] - WALL, X_NUT[1] + 0.02, -h, h, -h, h)])
    cut = [xf(cyl(BRG['od'] / 2, X_B1 - 0.01, 0.1, hole=True), Ry(90)),
           xf(cyl(BRG['od'] / 2, X_B2[0] - 0.1, X_B2[1] + 0.01, hole=True), Ry(90)),
           xf(cyl(4.4, X_B1 - 6.0, X_B1 + 0.02, hole=True), Ry(90)),                     # spacer room between races
           xf(cyl(8.0, X_CHIP + 0.01, X_B2[0] + 0.02, hole=True), Ry(90)),               # nut + magnet cap turn here
           box(X_BOARD[0], X_BOARD[1] + 0.01, -C_IN, h + 1, -C_IN, C_IN),                 # board slot, open to +Y
           box(X_BOARD[1] - 0.01, X_CHIP + 0.5, -C_IN + BOARD['ledge'], h + 1, -C_IN + BOARD['ledge'], C_IN - BOARD['ledge']),   # window chip-side
           cyl(1.25, -TUBE_R - 1, 0, x=-8.5, hole=True),                                   # hold set screw from below...
           ]
    cut[-1] = xf(cyl(1.25, 0, TUBE_R + 1, hole=True), T(-8.5, 0, 0) @ Rx(180))             # (radial, onto the spacer)
    return s - union(cut)


def roll_clip():
    """Closes the board slot on +Y (2 x M2)."""
    h = HOUSING / 2
    s = box(X_BOARD[0] - WALL, X_NUT[1] - 0.5, h, h + 2.0, -h, h) - box(X_BOARD[0] - 3, X_BOARD[1] + 3, h - 1, h + 3, -6.0, 6.0)
    return s


def roll_cap():
    """Pressed on the grip's nut (turns with the grip); magnet in its back face."""
    r_hex = NUT['af'] / 2 / math.cos(math.pi / 6) + 0.15
    c = cyl(7.0, 0.0, X_NUT[1] - X_MAG0 - 0.1) - union([cyl(r_hex, -0.1, NUT['h'] + 3.9, n=6),
                                                        cyl(MAG['d'] / 2 + 0.1, X_NUT[1] - X_MAG0 - MAG['h'] - 0.1, 99, hole=True)])
    return xf(c, T(X_NUT[1] - 0.1, 0, 0) @ Ry(-90))          # local +z -> -X


def roll_hw():
    hw = union([xf(cyl(BRG['od'] / 2 - 0.05, X_B1 + 0.05, -0.05), Ry(90)), xf(cyl(BRG['od'] / 2 - 0.05, X_B2[0] + 0.05, X_B2[1] - 0.05), Ry(90)),
                xf(cyl(2.4, X_NUT[0] - 3.5, 5.0), Ry(90)), xf(cyl(NUT['af'] / 2 / math.cos(math.pi / 6), X_NUT[0], X_NUT[1], n=6), Ry(90))])
    mag = xf(cyl(MAG['d'] / 2, X_MAG0, X_MAG0 + MAG['h']), Ry(90))
    brd = union([box(X_BOARD[0], X_BOARD[1], -BOARD['w'] / 2, BOARD['w'] / 2, -BOARD['w'] / 2, BOARD['w'] / 2),
                 box(X_CHIP - 0.0, X_BOARD[1], -2.5, 2.5, -2.5, 2.5)])
    return hw, mag, brd


# NB Ry(90) maps local +z to +x, so cyl(r, a, b) under Ry(90) spans x in [a, b] (cylinders above are built that way).


# ------------------------------------------------------------------ assembly frames
def frames(q=(0, 0, 0, 0, 0), trig=0.0):
    a1, a2, a3, p, r = q
    F = {'base': np.eye(4)}
    F['upper'] = Rz(a1)
    F['fore'] = F['upper'] @ T(L1, 0, 0) @ Rz(a2)
    F['wrist'] = F['fore'] @ T(L2, 0, 0) @ Rz(a3)
    F['pchild'] = F['wrist'] @ T(PITCH_X, 0, 0) @ Ry(p)              # + p = grip tip down
    F['grip'] = F['pchild'] @ T(ROLL_X, 0, 0) @ Rx(r)
    F['trig'] = F['grip'] @ T(*TRIG_PIVOT) @ Ry(-trig)
    # joint frames: (parent-side frame, child-side frame) in local joint coordinates (fork joints: axis local Z)
    F['J1p'], F['J1c'] = F['base'], F['upper']
    F['J2p'], F['J2c'] = F['upper'] @ T(L1, 0, 0), F['fore']
    F['J3p'], F['J3c'] = F['fore'] @ T(L2, 0, 0), F['wrist']
    F['PJp'], F['PJc'] = F['wrist'] @ T(PITCH_X, 0, 0) @ Rx(-90), F['pchild'] @ Rx(-90)
    F['RJp'], F['RJc'] = F['pchild'] @ T(ROLL_X, 0, 0), F['grip']
    return F


# ------------------------------------------------------------------ parts
PARTS = {}


def add(name, solid, frame, color, note, qty=1):
    PARTS[name] = dict(solid=solid, frame=frame, color=color, note=note, qty=qty)


def build():
    PARTS.clear()
    # base: J1 hub, arm back to a pillar, plate (clamp behind the desk edge), ESP32-S3 bay with a USB slot
    pz = -60.0
    # short neck at the hub (the J1 fork's web sweeps r 17..26 up to +-145 deg), wide pillar from r 30 back
    base = union([hub(), box(-31.0, -10.0, -8.0, 8.0, -HUB_H, HUB_H), box(-74.0, -30.0, -16.0, 16.0, pz, HUB_H),
                  box(-150.0, -30.0, -45.0, 45.0, pz - 6.0, pz)])
    bay = box(-146.0, -96.0, -23.0, 23.0, pz - 0.01, pz + 14.0) - box(-144.0, -98.0, -21.0, 21.0, pz + 1.5, pz + 15.0)
    base = union([base, bay]) - union([m5(-20, 20), box(-147.0, -143.0, -6.0, 6.0, pz + 2.0, pz + 12.0)])
    add('base', base, 'base', (0.85, 0.45, 0.15), 'desk-edge base: J1 hub, pillar, plate, ESP32-S3 bay (USB at the back)')
    for nm, L, fr in (('upper_link', L1, 'upper'), ('fore_link', L2, 'fore')):
        # full-height box section between the forks; thin (|z| <= 6) only within 17 mm of the next hub, where the
        # next fork's prongs pass above and below
        s = union([fork(26.0, W=7.0), box(17.0, L - 17.0, -7.0, 7.0, -PRONG[1], PRONG[1]), box(L - 17.5, L, -7.0, 7.0, -HUB_H, HUB_H),
                   xf(hub(), T(L, 0, 0))]) - xf(m5(-20, 20), T(L, 0, 0))
        add(nm, s, fr, (0.30, 0.55, 0.85), f'pan link {L:.0f} mm: encoder fork at the near joint, hub of the next joint')
    # wrist: J3 fork + arm to the pitch hub (axis Y)
    w = union([fork(26.0, W=7.0), box(17.0, PITCH_X - 17.0, -7.0, 7.0, -PRONG[1], PRONG[1]),
               box(20.0, PITCH_X, -HUB_H, HUB_H, -8.0, 8.0) - xf(cyl(HUB_R + 0.5, -30, 30), T(PITCH_X, 0, 0) @ Rx(-90)),
               xf(hub(), T(PITCH_X, 0, 0) @ Rx(-90))])
    w = w - xf(m5(-20, 20), T(PITCH_X, 0, 0) @ Rx(-90))
    add('wrist', w, 'wrist', (0.20, 0.60, 0.55), 'wrist-pan fork + pitch hub (axis Y)')
    # pitch link: pitch fork (frame PJc = pchild @ Rx(-90)) + arm to the roll tube's back
    h = HOUSING / 2
    pf = xf(fork(28.0, W=8.0, r_web=20.0), Rx(-90))      # narrow web: clears the wrist arm at pitch +120
    back = X_BOARD[0] - WALL + ROLL_X
    pl = union([pf, box(22.0, back + 0.5, -10.0, 10.0, -8.0, 8.0), xf(roll_tube(), T(ROLL_X, 0, 0))])
    add('pitch_link', pl, 'pchild', (0.55, 0.42, 0.75), 'pitch fork + roll bearing tube with the roll encoder bay')
    add('roll_clip', xf(roll_clip(), T(ROLL_X, 0, 0)), 'pchild', (0.35, 0.35, 0.40), 'closes the roll board slot (2 x M2)')
    # grip (frame 'grip', x = 0 at the roll tube's front face): flange + boss on the bolt, hollow grip box, trigger bay
    g0, g1, gw, gh = GRIP['x0'], GRIP['x1'], GRIP['w'] / 2, GRIP['h'] / 2
    flange = union([xf(cyl(HUB_R, 0.5, 4.5), Ry(90)), xf(cyl(3.5, 0.0, 0.51), Ry(90)), box(4.0, g0 + 0.5, -10.0, 10.0, -10.0, 10.0)])
    gt = GRIP['wall']
    shell = box(g0, g1, -gw, gw, -gh, gh) - box(g0 + gt, g1 - gt, -gw + gt, gw - gt, -gh + gt, gh - gt)
    grip = union([flange, shell])
    tx, ty, tz = TRIG_PIVOT
    cut = [xf(m5(-1, 4.6), Ry(90)), xf(cyl(HEAD['d'] / 2 + 0.3, 4.6, g0 + WALL + 0.1, hole=True), Ry(90)),   # bolt head from inside
           box(tx - 30.0, tx + 7.0, -5.6, 5.6, -gh - 1, tz + 7.0),                                           # trigger slot
           xf(cyl(1.6, -gw - 1, gw + 1, hole=True), T(tx, 0, tz) @ Rx(90)),                                   # trigger pivot M3
           cyl(8.0, gh - 3, gh + 1, x=g0 + 20.0, hole=True),                                                  # thumbstick Ø16
           cyl(3.6, gh - 3, gh + 1, x=g1 - 22.0, hole=True),                                                  # clutch button Ø7
           box(g1 - 3, g1 + 1, -5.0, 5.0, -4.0, 4.0)]                                                        # wire exit at the front end
    grip = grip - union(cut)
    # trigger encoder bay inside the +Y wall: board in the XZ plane, chip facing -Y onto the trigger hub's magnet face
    yc = 5.6 + MAG['gap']                         # chip face (hub face at y 5.5, magnet flush with it)
    yb0, yb1 = yc + BOARD['chip_h'], yc + BOARD['chip_h'] + BOARD['t']
    bay = box(tx - C_IN - 1.5, tx + C_IN + 1.5, yb0 - 1.5, gw - WALL + 0.01, max(tz - C_IN - 1.5, -gh + 0.01), gh - WALL + 0.01)
    bay = bay - union([box(tx - C_IN, tx + C_IN, yb0, yb1, tz - C_IN, gh), box(tx - C_IN + 1.2, tx + C_IN - 1.2, yb0 - 2, yb0 + 0.01, tz - C_IN + 1.2, tz + C_IN - 1.2)])
    add('grip', union([grip, bay]), 'grip', (0.92, 0.78, 0.25),
        'hand grip: roll flange on the M5, hollow grip, trigger slot + encoder bay, thumbstick (lift) and clutch-button holes')
    add('roll_cap', roll_cap(), 'RJc', (0.95, 0.95, 0.95), 'roll magnet cap (pressed on the grip bolt nut)')
    trig = union([xf(cyl(6.0, -5.5, 5.5), Rx(-90)), box(-30.0, 2.0, -5.5, 5.5, -24.0, -3.0)])
    trig = trig - union([xf(cyl(1.6, -7, 7, hole=True), Rx(-90)), xf(cyl(MAG['d'] / 2 + 0.1, 5.5 - MAG['h'], 6.0, hole=True), Rx(-90))])
    add('trigger', trig, 'trig', (0.15, 0.15, 0.17), 'gripper trigger: M3 pivot, magnet in the +Y hub face')
    # inner-race spacers: 1 mm rings under each bolt head / nut (fork joints) and behind the rear roll bearing, and the
    # 6 mm tube between the two roll bearings (rings small enough to clear the outer races)
    ring = cyl(3.7, 0.0, 1.0) - cyl(2.65, -1, 2, hole=True)
    for k in range(9):
        add(f'spacer_{k}', xf(ring, T(0, 0, -200.0 - 3 * k)), 'base', (0.95, 0.95, 0.95), 'inner-race spacer ring Ø7.4 / Ø5.3 x 1 mm')
    add('roll_spacer', xf(cyl(3.7, 0.0, 6.0) - cyl(2.65, -1, 7, hole=True), T(0, 0, -260.0)), 'base', (0.95, 0.95, 0.95),
        'spacer tube between the two roll bearings, Ø7.4 / Ø5.3 x 6 mm')
    for j in ('J1', 'J2', 'J3', 'PJ'):
        add(f'lid_{j}', lid(), f'{j}c', (0.35, 0.35, 0.40), 'encoder board lid (2 x M2 self-tapping)')
        add(f'magnet_cap_{j}', magnet_cap(), f'{j}p', (0.95, 0.95, 0.95), 'magnet cap, pressed on the fixed nyloc nut')


def world(q=(0, 0, 0, 0, 0), trig=0.0, bought=True):
    """Every part placed at joint angles q = (J1, J2, J3, pitch, roll) in degrees: [(name, solid, color, body)].
    body = the rigid body it moves with (for clash checks)."""
    F = frames(q, trig)
    body = {'base': 'base', 'upper': 'upper', 'fore': 'fore', 'wrist': 'wrist', 'pchild': 'pchild', 'grip': 'grip', 'trig': 'trig',
            'J1c': 'upper', 'J2c': 'fore', 'J3c': 'wrist', 'PJc': 'pchild', 'J1p': 'base', 'J2p': 'upper', 'J3p': 'fore',
            'PJp': 'wrist', 'RJc': 'grip', 'RJp': 'pchild'}
    out = [(nm, xf(p['solid'], F[p['frame']]), p['color'], body[p['frame']]) for nm, p in PARTS.items() if 'spacer' not in nm]
    if bought:
        for j in ('J1', 'J2', 'J3', 'PJ'):
            hw, mag, brd = fork_hw()
            out += [(f'hw_{j}', xf(hw, F[j + 'p']), (0.6, 0.6, 0.62), body[j + 'p']),
                    (f'magnet_{j}', xf(mag, F[j + 'p']), (0.75, 0.1, 0.1), body[j + 'p']),
                    (f'board_{j}', xf(brd, F[j + 'c']), (0.1, 0.45, 0.2), body[j + 'c'])]
        hw, mag, brd = roll_hw()
        out += [('hw_RJ', xf(hw, F['RJc']), (0.6, 0.6, 0.62), 'grip'), ('magnet_RJ', xf(mag, F['RJc']), (0.75, 0.1, 0.1), 'grip'),
                ('board_RJ', xf(brd, F['RJp']), (0.1, 0.45, 0.2), 'pchild')]
        tx, ty, tz = TRIG_PIVOT
        yc = 5.6 + MAG['gap']
        tb = box(tx - BOARD['w'] / 2, tx + BOARD['w'] / 2, yc + BOARD['chip_h'], yc + BOARD['chip_h'] + BOARD['t'], tz - BOARD['w'] / 2, tz + BOARD['w'] / 2)
        out += [('board_trig', xf(tb, F['grip']), (0.1, 0.45, 0.2), 'grip'),
                ('magnet_trig', xf(xf(cyl(MAG['d'] / 2, 5.5 - MAG['h'], 5.5), Rx(-90)), F['trig']), (0.75, 0.1, 0.1), 'trig')]
    return out


if __name__ == '__main__':
    build()
    tot = 0
    for nm, p in PARTS.items():
        v = p['solid'].volume() / 1000; tot += v
        print(f'{nm:16s} {v:6.1f} cm3')
    print('total printed volume', round(tot, 1), 'cm3')
