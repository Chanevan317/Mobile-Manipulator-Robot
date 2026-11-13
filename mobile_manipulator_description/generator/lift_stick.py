"""HANDHELD case for the lift joystick (user, 2026-11-11; on the table or in the hand): a KY-023 dual-axis thumbstick module (only its front-back axis is used:
push forward = lift up, pull back = down, let go = stop) + a Raspberry Pi Pico as the USB device. Both bought.

The Pico's dimensions are its datasheet's (exact). The KY-023's are ASSUMED from listings (they disagree: 34 x 26,
34 x 32, 40 x 27 mm; holes "M3.5, 27 / 20 mm"): MEASURE the real module and change KY below. The lid (with the
front-back slot around the stick's shaft) is the part that depends on them most; it's a separate, quick reprint.

Frame: mm, z up from the case bottom, x = toward the tip (= push = up), y to the left.
    python3 lift_stick.py [out_dir]       (default ../../CAD/lift_stick)
"""
import math, os, sys, csv
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import box, cyl, union, xf, T, Rx, Ry, Rz, print_mesh

# CONTROLLER GRIP (user, 2026-11-12: "ergonomic shape, easy to hold like a controller", two front buttons for the index
# and middle finger as placeholders, the Pico removable to solder wires and slid back in; replaces the 2026-11-11 wedge).
# One-hand grip in the spirit of a game-controller nunchuk-style grip (our own shape): a rounded body, palm along the back
# and the side, a taller HEAD at the front with the thumbstick on its flat top (thumb) and two buttons stacked on its
# front face (index on the upper one, middle finger on the lower one). Flat bottom: it can also lie on the table.
# x = toward the front (= push = lift UP), y to the left, z up from the bottom. Hull of spheres (all edges rounded).
# Raspberry Pi Pico (datasheet): 51 x 21 x 1.0 mm, 4 x dia 2.1 holes on 47.0 x 11.4 (2 mm from each short end),
# micro-USB on one short end. With OR without headers (2026-11-12, user: "enough space for headers if the board has";
# wires are soldered on the top side only): it SLIDES in from the front, USB end first, on a centre ridge (between the
# two pin rows), held sideways by two walls and from above by lips over its edges, until it stops against two blocks
# behind it with its socket in the USB opening; two fins from the top shell stop it sliding forward. Under both pin
# rows a channel stays free down to the floor for a header's plastic strip (2.5 mm) and pins (6.5 mm). Optional:
# 2 x M2 self-tap from above through its front holes into the ridge.
PICO = dict(L=51.0, W=21.0, t=1.0, hole=2.1, hx=47.0 / 2, hy=11.4 / 2, usb=(8.0, 3.2), x=3.2 + 25.5)
HDR = dict(row=8.89, half=1.27, plastic=2.5, pins=6.5, top=3.0)   # Pico pin rows at y +-8.89; a header under it (Pico H)
PICO_Z = 13.3                                       # board underside: a header's pins (9 mm) clear the floor, also in the
                                                    # rounded inside corner at the USB end (12.0 touched it there)
GROOVE = dict(play=0.15, lip=0.6, ridge=6.9)        # play round the board, lip over its edges, centre ridge half width
USB_OPEN = (13.5, 9.0)                              # opening up to the socket: a micro-USB (grip <= 11.5 x 8) OR a USB-C plug
                                                    # (grip <= 12.6 x 7.6) for a same-size board with USB-C (2026-11-12, user)
# KY-023 (ASSUMED, measure!): board L x W x t, hole dia and spacing, stick centre (from the board centre), dome top
# above the board's top face, shaft dia, the stock cap's underside above the board. Hangs from the top shell (4 posts).
# Joystick (2026-11-12, user: the Prakti PS2 dual-axis module, ASIN B0BTXGX8R9; its listing: 40 x 26 x 20 = the standard
# KY-023 board + its bent header pins, height without the cap). Dimensions from the one DIMENSIONED drawing found, the
# same standard board (Aus Electronics Direct TA0051): board 34.0 x 26.5 x 1.6, 4 x dia 3.0 holes on 26.2 x 19.9 (their
# centre 1.0 mm from the board centre, away from the pins), the header bent flat, 4.9 mm past the pin end; stick axis
# 0.9 mm toward the pins and 0.85 to one side of the hole pattern's centre; pivot 6.2 above the board; the moving dome
# dia 26.2 with its widest circle 3.3 above the pivot (its top ~12.5 above the pivot: ASSUMED from the drawing's look);
# cap dia 20, underside 23.0 / top 27.5 above the board; travel +-33 deg every way. 'x' = the hole pattern's centre.
# Held with M2.5 self-tap (the holes are 3.0: an M3 barely passes).
KY = dict(L=34.0, W=26.5, t=1.6, hole=3.0, hx=26.2 / 2, hy=19.9 / 2, board_dx=-1.0, pins=4.9, stick=(-0.9, -0.85),
          pivot=6.2, dome_r=13.1, dome_base=3.3, dome_top=12.5, shaft=4.0, cap=23.0, cap_r=10.0, cap_h=4.5, tilt=33.0,
          base=20.0, x=80.0)
STICK_MARGIN, STICK_GAP = 1.0, 1.0
# buttons (2026-11-12, user: push buttons, holders for later; then "symmetric for either hand ... wide top and bottom
# buttons at the front", like a one-handed adaptive controller's bumper + trigger): two WIDE buttons on the middle line of
# the front face, one above the other: upper = index finger, lower = middle finger, the same in either hand. Each: a
# printed cap (sticks out `out`, slides in a guide sleeve, a flange inside keeps it in) pressing ONE 12 x 12 x 7.3 mm
# through-hole tactile switch at its centre (pressed near an end it tips a little in its 0.2 mm play and still clicks),
# held from behind by a separate printed bracket (2 x M2 into 2 posts beside the buttons).
# Positions are (y, z) of the button centre; cap / hole = (width along y, height along z); d = depth inward from the front
# face (x = x1 - d). Both holes lie fully on the FLAT part of the front face (|y| <= hw_front - RS = 13, z 8 .. 36).
BTN = dict(pos=((0.0, 29.75), (0.0, 16.0)), names=('index', 'middle'), cap=(24.0, 10.1), hole=(24.4, 10.5), out=3.0,
           sleeve=5.0, fl=1.2, ft=1.2, sw=1.2, posts=((16.5, 23.0), (-16.5, 23.0)))
# 12 x 12 x 7.3 mm tactile switch (ASSUMED typical, measure!): body 12 x 12 x 3.6, stem 3.7 from the body, legs on 2 sides
SW = dict(body=12.0, h=3.6, stem=3.7, stem_w=3.8, leg_y=6.25, leg_len=4.0, pocket=12.3, stroke=1.05)
# chosen (2026-11-12, user): Robu "12x12x7.3mm Tactile Push Button Switch" (SKU 618210, generic, sold in India): 7.3 mm
# overall, square stem 3.4 x 3.8, stroke 1.05 mm, 3-5 N, 300,000 cycles. Body / stem split (3.6 / 3.7) ASSUMED.
BR = dict(back=2.0, y=18.2, z=(8.6, 37.0))          # bracket: back plate thickness, half width, z range
WALL, FLOOR = 2.4, 2.4
RS = 8.0                                            # rounding radius of the outside
SH = dict(x0=0.0, x1=116.0, hw_back=16.0, hw_front=21.0, h_back=30.0, h_full=44.0, x_full=64.0)
SPLIT_Z = 10.0                                      # bottom tray (Pico rails) / top shell (stick, buttons)
BOX_R = 400.0                                       # on the table: keep its centre >= this far from the leader's pedestal axis
LOW_R = 331.0                                       # leader parts come below the joystick's top (58 mm) within this radius (1200 poses)
SCREWS = [(30.0, 15.2), (30.0, -15.2), (88.0, 16.3), (88.0, -16.3)]    # M2.5 from underneath, into the top shell's bosses
BOSS_TOP = 21.0                                     # bosses end below the KY-023 board
PARTS = {}


def layout():
    """z of the KY board's top face and of the top shell's inside at the stick: the board sits so that the cap,
    tilted KY['tilt'] in any direction, keeps STICK_MARGIN above the case top."""
    ceil = SH['h_full'] - WALL
    d = KY['cap'] - KY['pivot']; th = math.radians(KY['tilt'])
    low = d * math.cos(th) - KY['cap_r'] * math.sin(th)              # cap's lowest edge above the pivot at full tilt
    pivot_z = SH['h_full'] + STICK_MARGIN - low
    return pivot_z - KY['pivot'], ceil


def stick_parts(tilt=0.0, phi=0.0):
    """(dome, shaft + cap) of the stick, tilted `tilt` deg toward direction `phi` (deg from +X), world frame. The dome
    is a sphere cap (dia dome_r * 2 at dome_base above the pivot, top at dome_top) that turns with the stick."""
    ky_top = layout()[0]; sx_, sy_ = KY['x'] + KY['stick'][0], KY['stick'][1]; pz = ky_top + KY['pivot']
    from lib import M
    H = KY['dome_top'] - KY['dome_base']; Rs = (KY['dome_r'] ** 2 + H ** 2) / (2 * H)
    dome = M.sphere(Rs, 64).translate((0, 0, KY['dome_top'] - Rs)) ^ box(-30, 30, -30, 30, KY['dome_base'], 40)
    rod = union([cyl(KY['shaft'] / 2, 0.0, KY['cap'] - KY['pivot'] + 0.5),
                 cyl(KY['cap_r'], KY['cap'] - KY['pivot'], KY['cap'] - KY['pivot'] + KY['cap_h'])])
    Tt = T(sx_, sy_, pz) @ Rz(phi) @ Ry(tilt) @ Rz(-phi)
    return xf(dome, Tt), xf(rod, Tt)


_hole = {}


def stick_hole_r():
    """Radius of the round hole: the dome's reach (from the stick axis) between the ceiling and the top, over the full
    travel, + STICK_GAP."""
    if 'r' not in _hole:
        ky_top, ceil = layout(); sx_, sy_ = KY['x'] + KY['stick'][0], KY['stick'][1]
        slab = box(sx_ - 40, sx_ + 40, sy_ - 40, sy_ + 40, ceil - 0.5, SH['h_full'] + 0.5)
        r = 0.0
        for th in np.linspace(0, KY['tilt'], 4):
            for phi in range(0, 360, 30):
                d = stick_parts(th, phi)[0] ^ slab
                if d.volume() <= 0: continue
                v = np.asarray(d.to_mesh().vert_properties)[:, :3]
                r = max(r, float(np.max(np.hypot(v[:, 0] - sx_, v[:, 1] - sy_))))
        _hole['r'] = r + STICK_GAP
    return _hole['r']


def shell(e):
    """Outside (e = 0) or inside (e = -WALL) of the grip: hull of spheres (offset = smaller spheres, same centres)."""
    from lib import M
    r = RS + e; c = []
    xb, xm, xf_ = SH['x0'] + RS, SH['x_full'], SH['x1'] - RS
    for x, hw, h in ((xb, SH['hw_back'], SH['h_back']), (xm, SH['hw_front'], SH['h_full']), (xf_, SH['hw_front'], SH['h_full'])):
        for y in (-(hw - RS), hw - RS):
            c += [(x, y, RS), (x, y, h - RS)]
    return M.batch_hull([M.sphere(r, 32).translate(p) for p in c])


def xcyl(r, x0, x1, y, z, hole=False):
    """Cylinder along X from x0 to x1 through (y, z)."""
    return xf(cyl(r, x0, x1, -z, y, hole=hole), Ry(90))


def switch(y, z):
    """12 x 12 tactile switch envelope at button (y, z): stem forward (touching the cap's back), body, legs behind."""
    x1, b = SH['x1'], SW['body'] / 2
    d0 = BTN['sleeve'] + BTN['ft']; d1 = d0 + SW['stem']; d2 = d1 + SW['h']
    stem = box(x1 - d1, x1 - d0, y - SW['stem_w'] / 2, y + SW['stem_w'] / 2, z - SW['stem_w'] / 2, z + SW['stem_w'] / 2)
    body = box(x1 - d2, x1 - d1, y - b, y + b, z - b, z + b)
    legs = union([box(x1 - d2 - SW['leg_len'], x1 - d2, y + sy * SW['leg_y'] - 0.6, y + sy * SW['leg_y'] + 0.6, z - 3.0, z + 3.0) for sy in (1, -1)])
    return union([stem, body, legs])


def button_parts():
    """(bracket, cap) parts: the bracket holds both switches (pockets open to the front, leg slots and a wire window
    behind), 2 x M2 into the front-wall posts; one cap per button (print 2)."""
    from lib import M
    x1 = SH['x1']; d_front = BTN['sleeve'] + BTN['ft'] + SW['stem']; d_back = d_front + SW['h'] + BR['back']
    plate = box(x1 - d_back, x1 - d_front, -BR['y'], BR['y'], *BR['z'])
    pk, cut = SW['pocket'] / 2, []
    for (y, z) in BTN['pos']:
        cut.append(box(x1 - d_front - SW['h'], x1 - d_front + 1.0, y - pk, y + pk, z - pk, z + pk))
        cut += [box(x1 - d_back - 1.0, x1 - d_front - SW['h'] + 0.1, y + sy * SW['leg_y'] - 1.0, y + sy * SW['leg_y'] + 1.0, z - 4.0, z + 4.0) for sy in (1, -1)]
    for (y, z) in BTN['posts']:
        cut += [xcyl(1.1, x1 - d_back - 1.0, x1 - d_front + 1.0, y, z, hole=True), xcyl(2.2, x1 - d_back - 1.0, x1 - d_back + 1.2, y, z)]
    bracket = plate - union(cut)
    cw, ch = BTN['cap'][0] / 2, BTN['cap'][1] / 2                       # cap in its own frame (centred on y = z = 0)
    fw, fh = BTN['hole'][0] / 2 + BTN['fl'], BTN['hole'][1] / 2 + BTN['fl']
    def rbox(x0, x1_, hw, hh, r):                                       # box with rounded edges along x (corner radius r)
        return union([box(x0, x1_, -hw + r, hw - r, -hh, hh), box(x0, x1_, -hw, hw, -hh + r, hh - r)] +
                     [xcyl(r, x0, x1_, sy * (hw - r), sz * (hh - r))
                      for sy in (1, -1) for sz in (1, -1)])
    cap = union([M.batch_hull([rbox(x1 - BTN['sleeve'], x1 + BTN['out'] - 1.0, cw, ch, 2.0),
                               rbox(x1 + BTN['out'] - 1.0, x1 + BTN['out'], cw - 0.8, ch - 0.8, 1.5)]),
                 rbox(x1 - BTN['sleeve'] - BTN['ft'], x1 - BTN['sleeve'], fw, fh, 2.0)])
    br = dict(solid=bracket, print_rot=Ry(-90), color=(0.30, 0.30, 0.33), qty=1,
              note='OPTIONAL, print with the buttons: holds both 12 x 12 tactile switches (pockets to the front, leg slots '
                   'behind); 2 x M2 x 10 into the posts on the front wall; prints on its back')
    cp = dict(solid=cap, print_rot=Ry(-90), color=(0.80, 0.15, 0.15), qty=2, at=BTN['pos'],
              note='OPTIONAL, print 2 with the buttons: button cap, sticks out 3 mm; drop it in from inside before the bracket '
                   '(the flange keeps it in); prints flange down')
    return br, cp


def caps_placed():
    """The two caps at their buttons (assembly)."""
    return union([PARTS['button_cap']['solid'].translate((0, y, z)) for (y, z) in BTN['pos']])


def header_env():
    """Pico H style headers under the board: plastic strips + pins along both rows, pin stubs on top."""
    px0, px1 = PICO['x'] - PICO['L'] / 2, PICO['x'] + PICO['L'] / 2; r, h = HDR['row'], HDR['half']
    out = []
    for sy in (1, -1):
        y = sy * r
        out += [box(px0 + 0.1, px1 - 0.1, y - h, y + h, PICO_Z - HDR['plastic'], PICO_Z),
                box(px0 + 1.0, px1 - 1.0, y - 0.4, y + 0.4, PICO_Z - HDR['plastic'] - HDR['pins'], PICO_Z),   # pins 1.37 from the ends
                box(px0 + 1.0, px1 - 1.0, y - 0.6, y + 0.6, PICO_Z + PICO['t'], PICO_Z + PICO['t'] + HDR['top'])]
    return union(out)


# joystick bracket: plate under the module + 4 standoffs at its holes; fixed to 4 posts hanging from the top shell
KB = dict(stand=3.0, plate=2.4, posts=((59.0, 12.0), (59.0, -12.0), (98.2, 12.0), (98.2, -12.0)), post_r=1.9, x=(56.0, 100.0), y=14.0)
# posts r 1.9 (front pair between the module board's front end at 96 and the button bracket at 100.4), M2 x 8 self-tap


def bracket_z():
    """(plate bottom, plate top) z of the joystick bracket."""
    kz0 = layout()[0] - KY['t']
    return kz0 - KB['stand'] - KB['plate'], kz0 - KB['stand']


def ky_bracket():
    zb, zt = bracket_z(); kz0 = zt + KB['stand']
    holes = [(KY['x'] + sx * KY['hx'], sy * KY['hy']) for sx in (-1, 1) for sy in (-1, 1)]
    plate = box(KB['x'][0], KB['x'][1], -KB['y'], KB['y'], zb, zt)
    stands = [cyl(2.6, zt - 0.01, kz0, x, y) for (x, y) in holes]
    cut = [cyl(1.05, zb - 1, kz0 + 1, x, y, hole=True) for (x, y) in holes]                           # M2.5 self-tap
    cut += [cyl(1.15, zb - 1, zt + 1, x, y, hole=True) for (x, y) in KB['posts']]                     # M2 up into the posts
    cut += [box(KY['x'] - 8.0, KY['x'] + 8.0, -6.0, 6.0, zb - 1, zt + 1)]                            # window: wires / leads
    return union([plate] + stands) - union(cut)


def pico_box(e=0.0, z_extra=0.0):
    x0 = PICO['x'] - PICO['L'] / 2
    return box(x0 - e, x0 + PICO['L'] + e, -PICO['W'] / 2 - e, PICO['W'] / 2 + e, PICO_Z - e, PICO_Z + PICO['t'] + e + z_extra)


def body():
    ky_top, ceil = layout()
    outer, inner = shell(0.0), shell(-WALL)
    wall = outer - inner
    low = box(-50, 150, -50, 50, -1, SPLIT_Z); high = box(-50, 150, -50, 50, SPLIT_Z, 80)
    G = GROOVE; px0, px1 = PICO['x'] - PICO['L'] / 2, PICO['x'] + PICO['L'] / 2
    # Pico mount: side walls (play 0.15) with lips over the board's edges, a centre ridge under it between the pin rows,
    # channels under the pin rows free to the floor (headers); all open at the front end so it slides in
    zt_ = PICO_Z + PICO['t'] + 1.2; wi = PICO['W'] / 2 + G['play']
    rails = union([box(px0 - 1.0, px1, sy * wi if sy > 0 else -(wi + 1.55), wi + 1.55 if sy > 0 else -wi, FLOOR - 0.5, zt_) for sy in (1, -1)] +
                  [box(px0 - 1.0, px1, sy * (PICO['W'] / 2 - G['lip']) if sy > 0 else -wi - 0.01, wi + 0.01 if sy > 0 else -(PICO['W'] / 2 - G['lip']),
                       PICO_Z + PICO['t'] + G['play'], zt_) for sy in (1, -1)] +
                  [box(px0, px1, -G['ridge'], G['ridge'], FLOOR - 0.5, PICO_Z - G['play'])]) ^ inner
    groove = box(px0 - G['play'], px1 + 10.0, -wi, wi, PICO_Z - G['play'], PICO_Z + PICO['t'] + G['play'])
    # back stops: blocks behind the board on both sides of the USB opening (unplugging the cable pulls the board back)
    bstop = union([box(px0 - 6.0, px0 - G['play'], sy * (USB_OPEN[0] / 2 + 0.6) if sy > 0 else -PICO['W'] / 2,
                       PICO['W'] / 2 if sy > 0 else -(USB_OPEN[0] / 2 + 0.6), FLOOR - 0.5, zt_) for sy in (1, -1)]) ^ inner
    # optional M2 screws through the Pico's front holes: low bosses it slides over (top = board underside)
    fx = PICO['x'] + PICO['hx']
    pbosses = box(0, 0.1, 0, 0.1, 0, 0.1) ^ box(5, 6, 5, 6, 5, 6)                 # (empty: the ridge takes the screws)
    phole = union([cyl(0.85, PICO_Z - 7.0, PICO_Z + 1, fx, sy * PICO['hy'], hole=True) for sy in (1, -1)])
    # forward stop: two fins hanging from the top shell's side walls just in front of the Pico's front end (over its edges,
    # clear of the KY-023's pins in the middle); they reach below the split line to the board
    stop = union([box(px1 + 0.4, px1 + 4.4, sy * 6.0 if sy > 0 else -25.0, 25.0 if sy > 0 else -6.0, PICO_Z - 0.5, 16.0) for sy in (1, -1)]) ^ inner
    # KY-023 hangs from the top shell: 4 posts down from the ceiling, M3 self-tap from below through its holes
    # KY-023 on a separate BRACKET under it (its dome, dia 26.2, passes over the corner holes, so nothing can come down to
    # them from above; posts from the tray would block the Pico's slide path): 4 short standoffs, M2.5 self-tap from above
    # through its holes; the bracket screws UP into 4 posts hanging from the top shell, in front of and behind the stick
    zp = bracket_z()[1]
    kposts = union([cyl(KB['post_r'], zp, 80, x, y) for (x, y) in KB['posts']]) ^ inner
    khole = union([cyl(0.8, zp - 1.0, zp + 8.0, x, y, hole=True) for (x, y) in KB['posts']])           # M2 self-tap
    bosses = union([cyl(2.8, 0, BOSS_TOP, x, y) for (x, y) in SCREWS]) ^ inner
    from lib import cone
    def zs(x, y):                                                       # the outside's height under a screw (rounded edges)
        return (cyl(0.2, -1, 20, x, y) ^ outer).bounding_box()[2]
    scut = union([cyl(1.4, -1, SPLIT_Z + 0.2, x, y, hole=True) for (x, y) in SCREWS] +              # clearance through the tray
                 [union([cyl(2.7, -1, zs(x, y) + 0.01, x, y), cone(2.7, 1.4, zs(x, y), zs(x, y) + 1.3, x, y)])
                  for (x, y) in SCREWS] +                                                           # countersink, as deep as the outside is high
                 [cyl(1.0, SPLIT_Z, BOSS_TOP - 1.0, x, y, hole=True) for (x, y) in SCREWS])         # pilot in the top shell
    # USB opening: through the back wall right up to the Pico's socket, centred on it (plug grips up to ~12 x 8.5)
    uz = PICO_Z + PICO['t'] + PICO['usb'][1] / 2
    usb = box(SH['x0'] - 5.0, px0 - 0.2, -USB_OPEN[0] / 2, USB_OPEN[0] / 2, uz - USB_OPEN[1] / 2, uz + USB_OPEN[1] / 2)
    # button placeholders: square openings through the front face (clear space behind them is checked in boards())
    # buttons: square holes through the front wall + guide sleeves inside + 2 posts for the switch bracket
    x1 = SH['x1']; hw, hh = BTN['hole'][0] / 2, BTN['hole'][1] / 2; sw_ = BTN['sw']
    bcut = union([box(x1 - BTN['sleeve'] - 0.5, x1 + 5.0, y - hw, y + hw, z - hh, z + hh) for (y, z) in BTN['pos']])
    sleeves = union([box(x1 - BTN['sleeve'], x1 - 1.0, y - hw - sw_, y + hw + sw_, max(z - hh - sw_, SPLIT_Z), z + hh + sw_) for (y, z) in BTN['pos']])
    d_front = BTN['sleeve'] + BTN['ft'] + SW['stem']                         # switch body front = bracket front face
    bposts = union([xcyl(2.8, x1 - d_front, x1 - 1.0, y, z) for (y, z) in BTN['posts']]) ^ shell(-WALL + 0.5)   # stay inside the rounded corners
    bpilot = union([xcyl(0.8, x1 - d_front - 0.5, x1 - d_front + 7.0, y, z, hole=True) for (y, z) in BTN['posts']])
    feet = union([cyl(5.0, -1, 1.0, x, y) for x in (18.0, 98.0) for y in (-9.0, 9.0)])
    tray = union([(wall ^ low), (bosses ^ low), rails, pbosses, bstop]) - union([scut, usb, feet, groove, phole, pico_box(0.15), bcut])
    sx_, sy_ = KY['x'] + KY['stick'][0], KY['stick'][1]
    rh = stick_hole_r()                                                        # round hole: the dome comes up through it
    slot = cyl(rh, ceil - 5, 80, sx_, sy_)
    zt = SH['h_full']; ax0 = sx_ + rh + 2.5                                     # "up" arrow in front of the hole
    arrow = union([box(ax0, ax0 + 6.0, -1.0, 1.0, zt - 0.6, zt + 1), box(ax0 + 6.0, ax0 + 8.0, -3.0, 3.0, zt - 0.6, zt + 1)])
    top = union([(wall ^ high), (bosses ^ high), stop, sleeves, bposts, kposts]) - union([scut, usb, slot, arrow, bcut, bpilot, khole])
    PARTS['stick_bracket'] = dict(solid=ky_bracket(), print_rot=np.eye(4), color=(0.30, 0.30, 0.33), qty=1,
                                  note='holds the joystick module (4 standoffs, M2.5 x 8 self-tap from above through its holes); '
                                       'screws up into the top shell (4 x M2 x 8); prints flat')
    PARTS['stick_tray'] = dict(solid=tray, print_rot=np.eye(4), color=(0.25, 0.25, 0.28),
                               note='bottom tray: the Pico slide-in mount (room for headers), holes for 2 optional M2 '
                                    'screws, USB opening, 4 countersunk M2.5 from underneath, 4 rubber-feet recesses; prints flat')
    PARTS['button_bracket'], PARTS['button_cap'] = button_parts()
    PARTS['stick_top'] = dict(solid=top, print_rot=Rx(180), color=(0.85, 0.45, 0.15),
                              note='top shell (the grip): round stick hole + "up" arrow on the head, 4 posts for the joystick bracket, 2 button holes with guide sleeves + '
                                   '2 bracket posts on the front, 2 fins that stop the Pico; prints upside down (head top '
                                   'on the bed, tree supports under the sloping tail)')


def build():
    PARTS.clear(); body()
    for k in ('stick_bracket', 'button_bracket', 'button_cap'): PARTS[k] = PARTS.pop(k)   # export order: tray, top, brackets, cap


def boards(buttons=True):
    """Envelopes of the Pico (+ room above for soldered wires), the KY-023 + stick (assumed) and the button
    placeholders, for the clash check and the views."""
    ky_top, ceil = layout()
    pico = union([pico_box(), box(PICO['x'] - PICO['L'] / 2 - 0.0, PICO['x'] - PICO['L'] / 2 + 5.5, -4.0, 4.0,
                                  PICO_Z + PICO['t'], PICO_Z + PICO['t'] + PICO['usb'][1])])               # micro-USB socket
    wires = box(PICO['x'] - PICO['L'] / 2 + 6.0, PICO['x'] + PICO['L'] / 2 - 0.5, -9.7, 9.7, PICO_Z + PICO['t'], PICO_Z + PICO['t'] + HDR['top'] + 8.0)
    kz0 = ky_top - KY['t']
    bx0, bx1 = KY['x'] + KY['board_dx'] - KY['L'] / 2, KY['x'] + KY['board_dx'] + KY['L'] / 2
    ky = box(bx0, bx1, -KY['W'] / 2, KY['W'] / 2, kz0, ky_top)
    kpins = box(bx0 - KY['pins'] - 6.0, bx0, -6.5, 6.5, kz0 - 1.5, ky_top + 2.6)     # bent header + wires / plugs, to the back
    sx_, sy_ = KY['x'] + KY['stick'][0], KY['stick'][1]
    b2 = KY['base'] / 2
    stick = union(list(stick_parts()) + [box(sx_ - b2, sx_ + b2, sy_ - b2, sy_ + b2, ky_top, ky_top + KY['pivot'] + KY['dome_base'])])  # + the fixed gimbal block
    its = [pico, wires, ky, kpins, stick]
    if buttons:
        its += [switch(y, z) for (y, z) in BTN['pos']]
    return union(its)


def hand():
    """A simple right hand holding it: palm along the right side and back, thumb on the stick cap, index and middle
    finger on the two front buttons, ring and little finger under the body."""
    from lib import M
    cap_top = layout()[0] + KY['cap'] + KY['cap_h']
    sph = lambda p, r: M.sphere(r, 24).translate(p)
    cap = lambda *pts_r: M.batch_hull([sph(p, pts_r[-1]) for p in pts_r[:-1]])
    palm = M.batch_hull([sph((8.0, -30.0, 18.0), 13.0), sph((56.0, -32.0, 22.0), 13.0), sph((10.0, -26.0, 2.0), 11.0),
                         sph((56.0, -28.0, 4.0), 11.0), sph((-16.0, -22.0, 10.0), 15.0)])
    thumb = union([cap((50.0, -34.0, 34.0), (70.0, -18.0, cap_top + 1.0), 9.0), cap((70.0, -18.0, cap_top + 1.0), (KY['x'] - 2.0, 0.0, cap_top + 7.0), 8.5)])
    (yi, zi), (ym, zm) = BTN['pos']
    index = union([cap((66.0, -30.0, 32.0), (108.0, -28.0, zi), 7.0), cap((108.0, -28.0, zi), (123.0, yi + 2.0, zi), 6.5)])
    middle = union([cap((64.0, -30.0, 14.0), (106.0, -28.0, zm), 7.3), cap((106.0, -28.0, zm), (123.0, ym + 5.0, zm), 6.8)])
    ring_ = union([cap((50.0, -30.0, 2.0), (52.0, -10.0, -9.0), 7.4), cap((52.0, -10.0, -9.0), (52.0, 14.0, -8.0), 7.0)])
    little = union([cap((36.0, -28.0, 0.0), (37.0, -8.0, -8.5), 6.5), cap((37.0, -8.0, -8.5), (37.0, 10.0, -7.5), 6.0)])
    return union([palm, thumb, index, middle, ring_, little])


def check():
    """Boards / buttons clear of both shells; the Pico slides out of the tray (top off); the top shell's fins stop it
    moving forward, the groove lips stop it lifting, the rails' back end stops it going back; a micro-USB plug (moulded
    grip 11.5 x 8 mm) reaches the socket."""
    b = boards(); res = []
    tray, top = PARTS['stick_tray']['solid'], PARTS['stick_top']['solid']
    caps, brk, kb = caps_placed(), PARTS['button_bracket']['solid'], PARTS['stick_bracket']['solid']
    placed = {'stick_tray': tray, 'stick_top': top, 'button_bracket': brk, 'button_cap': caps, 'stick_bracket': kb}
    for k, sol in placed.items():
        v = (sol ^ b).volume()
        if v > 0.5: res.append((k, round(v, 1)))
    for (a, sa), (c, sc) in [(('stick bracket', kb), ('top', top)), (('stick bracket', kb), ('tray', tray)),
                             (('stick bracket', kb), ('button bracket', brk)), (('bracket', brk), ('top', top)), (('bracket', brk), ('tray', tray)), (('caps', caps), ('top', top)),
                             (('caps', caps), ('bracket', brk)), (('caps', caps), ('tray', tray))]:
        v = (sa ^ sc).volume()
        if v > 0.5: res.append((a + ' x ' + c, round(v, 1)))
    sw = union([switch(y, z) for (y, z) in BTN['pos']])
    pr = SW['stroke'] + 0.25                                           # full stroke + margin
    sw_body = union([box(SH['x1'] - (BTN['sleeve'] + BTN['ft'] + SW['stem'] + SW['h']), SH['x1'] - (BTN['sleeve'] + BTN['ft'] + SW['stem']),
                         y - 6, y + 6, z - 6, z + 6) for (y, z) in BTN['pos']])
    buttons = {'cap pulled out 0.3 mm hits the sleeve (kept in)': round((caps.translate((0.3, 0, 0)) ^ top).volume(), 1),
               f'cap pressed {pr:.2f} mm (full stroke): free in the sleeve and the bracket': round((caps.translate((-pr, 0, 0)) ^ (top + brk + sw_body)).volume(), 2),
               'cap pressed 0.4 mm: pushes the switch stem': round((caps.translate((-0.4, 0, 0)) ^ sw).volume(), 1),
               'cap still out at full stroke (mm)': round(caps.bounding_box()[3] - SH['x1'] - SW['stroke'], 2),
               'cap face out of the front (mm)': round(caps.bounding_box()[3] - SH['x1'], 1)}
    pb = pico_box()
    slide = [dx for dx in np.arange(0.0, 70.01, 2.5) if (tray ^ pb.translate((dx, 0, 0))).volume() > 0.05]
    he = header_env()
    hdr = dict(at_rest={k: round((v ^ he).volume(), 2) for k, v in (('tray', tray), ('top', top))},
               slide_blocked=[dx for dx in np.arange(0.0, 55.01, 2.5) if (tray ^ union([pb, he]).translate((dx, 0, 0))).volume() > 0.05])
    held = {'forward 0.6 mm (top on)': (top ^ pb.translate((0.6, 0, 0))).volume(),
            'up 0.4 mm': (tray ^ pb.translate((0, 0, 0.4))).volume(),
            'down 0.4 mm': (tray ^ pb.translate((0, 0, -0.4))).volume(),
            'sideways 0.4 mm': (tray ^ pb.translate((0, 0.4, 0))).volume(),
            'back 0.4 mm': (tray ^ pb.translate((-0.4, 0, 0))).volume()}
    uz = PICO_Z + PICO['t'] + PICO['usb'][1] / 2; px0 = PICO['x'] - PICO['L'] / 2
    placed = {'tray': tray, 'top': top}
    plug_hit = {}
    for kind, (gw, gh, mw, mh) in (('micro-USB', (11.5, 8.0, 7.0, 2.4)), ('USB-C', (12.6, 7.6, 8.4, 2.6))):
        plug = union([box(-40.0, px0 - 1.0, -gw / 2, gw / 2, uz - gh / 2, uz + gh / 2),               # moulded grip
                      box(px0 - 1.0, px0 + 5.0, -mw / 2, mw / 2, uz - mh / 2, uz + mh / 2)])          # metal plug
        plug_hit[kind] = {k: round((v ^ plug).volume(), 2) for k, v in placed.items()}
    # the stick tilted fully in 12 directions: shaft + cap clear of the top shell; the dome clear of its hole
    dome, _ = stick_parts()
    tilt_hits = {}
    for phi in range(0, 360, 30):
        dm, rod = stick_parts(KY['tilt'], phi)
        v = ((rod + dm) ^ (top + kb)).volume()
        if v > 0.05: tilt_hits[phi] = round(v, 1)
    stick = dict(tilt_hits=tilt_hits, dome_vs_top=round((dome ^ top).volume(), 2),
                 cap_above_top_mm=round(layout()[0] + KY['cap'] - SH['h_full'], 1),
                 dome_above_top_mm=round(layout()[0] + KY['pivot'] + KY['dome_top'] - SH['h_full'], 1), hole_r=round(stick_hole_r(), 1))
    ky_top, lid_z = layout()
    return dict(clash=res, buttons=buttons, stick=stick, headers=hdr, slide_blocked=slide, held={k: round(v, 1) for k, v in held.items()}, plug=plug_hit,
                lid_z=lid_z, ky_top=ky_top)


def export(out):
    from stlio import write_stl
    for d in ('print_ready_stl', 'assembly_stl'):
        os.makedirs(os.path.join(out, d), exist_ok=True)
        for f in os.listdir(os.path.join(out, d)):
            if f.endswith('.stl'): os.remove(os.path.join(out, d, f))
    rows = []
    for i, (nm, p) in enumerate(PARTS.items(), 1):
        num = f'S{i}_{nm}'
        V, F = print_mesh(caps_placed() if nm == 'button_cap' else p['solid']); write_stl(os.path.join(out, 'assembly_stl', num + '.stl'), V, F)
        s = xf(p['solid'], p['print_rot']); bb = s.bounding_box(); s = s.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2]))
        bb = s.bounding_box(); V, F = print_mesh(s); write_stl(os.path.join(out, 'print_ready_stl', num + '.stl'), V, F)
        rows.append(dict(file=num + '.stl', qty=p.get('qty', 1), size_mm=f'{bb[3]-bb[0]:.1f} x {bb[4]-bb[1]:.1f} x {bb[5]-bb[2]:.1f}',
                         volume_cm3=round(p['solid'].volume() / 1000, 1), note=p['note']))
    V, F = print_mesh(boards()); write_stl(os.path.join(out, 'assembly_stl', 'boards_assumed.stl'), V, F)
    with open(os.path.join(out, 'parts.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    return rows


if __name__ == '__main__':
    import lift_stick as LS
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join('..', '..', 'CAD', 'lift_stick')
    LS.build()
    c = LS.check()
    print('boards, wires, buttons clear of the case:', 'yes' if not c['clash'] else c['clash'])
    print('buttons:', c['buttons'])
    print('Pico slides out of the tray (top off): ', 'yes' if not c['slide_blocked'] else f"blocked at {c['slide_blocked']} mm")
    print('Pico held (overlap mm3 if moved; must be > 0):', c['held'])
    print('Pico WITH headers: clear at rest', c['headers']['at_rest'], '| slides out:',
          'yes (until it leaves the rails)' if not c['headers']['slide_blocked'] or min(c['headers']['slide_blocked']) > 52 else c['headers']['slide_blocked'])
    print('USB plugs reach the socket (overlap mm3, must be 0):', c['plug'])
    print('stick: full tilt', LS.KY['tilt'], 'deg in 12 directions hits the top:', c['stick']['tilt_hits'] or 'never', '| dome vs its hole:',
          c['stick']['dome_vs_top'], 'mm3 | cap underside', c['stick']['cap_above_top_mm'], 'mm and dome top',
          c['stick']['dome_above_top_mm'], 'mm above the case top; hole r', c['stick']['hole_r'])
    for r in LS.export(out): print('  ', r['file'], r['size_mm'], r['volume_cm3'], 'cm3')
    LS.render(os.path.join(out, 'preview_lift_stick.png'))


def render(path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    def draw(ax, its, el, az, zoom=1.0, cut=None, title='', center=None):
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
        lo, hi = T_.reshape(-1, 3).min(0), T_.reshape(-1, 3).max(0); c = (lo + hi) / 2 if center is None else np.array(center); r = (hi - lo).max() / 2 / zoom
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off(); ax.set_title(title, fontsize=13)
    b = (boards(buttons=False), (0.10, 0.35, 0.20))
    btn = (union([switch(y, z) for (y, z) in BTN['pos']]), (0.95, 0.85, 0.30))
    caps = (caps_placed(), PARTS['button_cap']['color']); brk = (PARTS['button_bracket']['solid'], PARTS['button_bracket']['color'])
    kbk = (PARTS['stick_bracket']['solid'], PARTS['stick_bracket']['color'])
    tray, top = PARTS['stick_tray'], PARTS['stick_top']
    T_, P_ = (tray['solid'], tray['color']), (top['solid'], top['color'])
    pico_out = (pico_box().translate((62.0, 0, 0)), (0.10, 0.35, 0.20))
    fig = plt.figure(figsize=(22, 13), facecolor='white')
    draw(fig.add_subplot(2, 3, 1, projection='3d'), [T_, P_, b, (hand(), (0.93, 0.76, 0.62))], 22, -130, zoom=1.0,
         title='in the right hand:\nthumb on the stick, index + middle on the buttons')
    draw(fig.add_subplot(2, 3, 2, projection='3d'), [T_, P_, b, caps], 0, 0, zoom=2.6, center=(SH['x1'], 0, SH['h_full'] / 2),
         title='front, straight on: two wide buttons, the same for either hand\n(upper = index finger, lower = middle finger)')
    draw(fig.add_subplot(2, 3, 3, projection='3d'), [T_, P_, b, btn, caps, brk, kbk], 0, -90, zoom=1.1, cut=box(-50, 150, 0.0, 60, -10, 80),
         title='cut along the middle: both buttons (cap, switch in yellow,\nbracket); Pico (back), KY-023 (top)')
    split = box(-50, PICO['x'] + PICO['L'] / 2 + 1.0, -50, 50, -10, 80)              # Pico (stays) | joystick (lifts)
    lift = lambda it: (it[0].translate((0, 0, 45)), it[1])
    draw(fig.add_subplot(2, 3, 4, projection='3d'), [T_, (b[0] ^ split, b[1]), lift((b[0] - split, b[1])), lift(P_), lift(btn),
                                                     lift(brk), lift(kbk), lift(caps)], 32, -60, zoom=1.0,
         title='top shell lifted: the joystick and the buttons come with it;\nthe Pico stays in its rails in the tray')
    draw(fig.add_subplot(2, 3, 5, projection='3d'), [T_, pico_out], 40, -60, zoom=1.0,
         title='to solder: slide the Pico out toward the front\n(with or without headers; solder on the top side)')
    import leader as LD
    if not LD.PARTS: LD.build()
    place = T(0, BOX_R, -LD.H_ARM) @ Rz(90) @ T(-(SH['x0'] + SH['x1']) / 2, 0, 0)
    stick = [(xf(tray['solid'], place), tray['color']), (xf(top['solid'], place), top['color']), (xf(boards(), place), (0.10, 0.35, 0.20))]
    zone = (cyl(LOW_R, -LD.H_ARM, -LD.H_ARM + 0.6) - cyl(LOW_R - 4.0, -LD.H_ARM - 1, -LD.H_ARM + 1), (0.85, 0.20, 0.20))
    its = [(s_, c_) for nm, s_, c_, bd in LD.world((0, 0, 0, 0, 0), 0.0, variant='mono')] + stick + [zone]
    draw(fig.add_subplot(2, 3, 6, projection='3d'), its, 30, -140, zoom=1.05,
         title=f'on the desk: {BOX_R / 10:.0f} cm from the leader\n(red circle: the leader can come low inside {LOW_R / 10:.0f} cm)')
    plt.tight_layout(); plt.savefig(path, dpi=75); plt.close(fig)
