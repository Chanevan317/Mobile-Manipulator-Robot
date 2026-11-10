"""Hardware: fastener specs, cutters for printed parts, visual models, and a fastener registry."""
import math
import numpy as np
from lib import M, box, cyl, ring, union, xf, T, Rx, Ry, Rz

SPEC = {
    # clearance hole, head dia, head height, counterbore dia, insert hole dia, default insert hole depth, insert outside dia
    # Inserts: the user's 350-pc kit (2026-11-10): M3 x 4 / 6 / 8 long, 5.0 OD; M5 x 4 / 6 / 8, 7.0 OD (name = M x L x OD).
    # M3 hole 4.3 is an estimate for the 5.0 OD (was 4.0 for 4.6 OD Ruthex): measure the inserts / print a test coupon.
    'M2': dict(clear=2.4, head=3.8, head_h=1.6, cb=4.4, ins=3.2, ins_d=4.5, od=3.6),
    'M3': dict(clear=3.4, head=5.5, head_h=3.0, cb=6.2, ins=4.3, ins_d=6.5, od=5.0),
    'M5': dict(clear=5.5, head=8.5, head_h=5.0, cb=9.2, ins=6.4, ins_d=10.5, od=7.0),
}


def _rot_z_to(d):
    d = np.asarray(d, float); d = d / np.linalg.norm(d)
    z = np.array([0, 0, 1.0])
    v = np.cross(z, d); c = float(np.dot(z, d))
    R = np.eye(4)
    if np.linalg.norm(v) < 1e-9:
        if c < 0:
            R = Rx(180)
        return R
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R[:3, :3] = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
    return R


def place(solid_z, p, d):
    """Solid built along +Z from the origin -> start at point p, pointing along d."""
    return xf(solid_z, T(*p) @ _rot_z_to(d))


class Reg:
    """Collects fasteners in a part's own frame. Each entry: kind, size, length, p (head seat / insert mouth), d (into the work)."""
    def __init__(self):
        self.items = []

    def add(self, kind, size, p, d, length=None, note=''):
        self.items.append(dict(kind=kind, size=size, length=length, p=tuple(float(v) for v in p),
                               d=tuple(float(v) for v in np.asarray(d, float) / np.linalg.norm(d)), note=note))


# ---------- cutters (to subtract from printed parts) ----------
def screw_cut(p, d, size, shank, cb=0.0, extra=0.5, clear=None, cb_dia=None):
    """Clearance hole from seat p along d for `shank` mm (+extra); counterbore of depth cb behind the seat (against d).
    clear / cb_dia override the SPEC hole and counterbore diameters (e.g. the SO-100 servo-case holes)."""
    s = SPEC[size]
    parts = [cyl((clear or s['clear']) / 2, -0.01, shank + extra, hole=True)]
    if cb > 0:
        parts.append(cyl((cb_dia or s['cb']) / 2, -cb - 0.01, 0.0, hole=True))
    return place(union(parts), p, d)


def insert_cut(p, d, size, depth=None, length=None):
    """Heat-set insert hole. The hole is at least `length` + 0.5 deep (the insert sits flush) and at least `depth`
    (default SPEC ins_d), which also leaves room for the screw tip below a short insert."""
    s = SPEC[size]
    depth = max(depth or s['ins_d'], (length or 0) + 0.5)
    c = union([cyl(s['ins'] / 2, -0.5, depth, hole=True), cyl(s['ins'] / 2 + 0.4, -0.5, 0.4, hole=True)])
    return place(c, p, d)


def nut_cut(p, d, size='M3', depth=2.6, af=5.75):
    r = af / 2 / math.cos(math.pi / 6)
    hexa = M.cylinder(depth, r, r, 6).translate((0, 0, 0))
    return place(xf(hexa, Rz(30)), p, d)


# ---------- visual models (not printed) ----------
def screw_model(size, length, kind='SHCS'):
    s = SPEC[size]; dia = float(size[1:])
    if kind == 'SHCS':
        head = cyl(s['head'] / 2, -s['head_h'], 0) - cyl(s['head'] / 2 * 0.45, -s['head_h'] - 0.1, -s['head_h'] + 1.5, n=6)
    else:  # pan-head self-tapper
        head = cyl(s['head'] / 2, -s['head_h'], 0)
    return union([head, cyl(dia / 2 * 0.92, 0, length, n=16)])


def insert_model(size, length=None):
    s = SPEC[size]
    return ring(float(size[1:]) / 2, s['od'] / 2, 0, length or s['ins_d'] - 0.8)


def bearing_6808():
    """6808-2RS: 40 x 52 x 7, centred on its own axis, faces at z=0..7."""
    return union([ring(20.0, 22.6, 0, 7), ring(23.6, 26.0, 0, 7), ring(22.6, 23.6, 0.6, 6.4)])


def mgn9_rail(length=170.0, pitch=20.0):
    """MGN9 rail, base on z=0, along Y, centred. 9 wide x 6.5 tall; M3 counterbored holes at `pitch`."""
    r = box(-4.5, 4.5, -length / 2, length / 2, 0, 6.5)
    r = r - box(-5, -3.6, -length / 2 - 1, length / 2 + 1, 3.2, 4.6) - box(3.6, 5, -length / 2 - 1, length / 2 + 1, 3.2, 4.6)
    n = int(round((length - 10) / pitch)) + 1
    ys = [(-(n - 1) / 2 + i) * pitch for i in range(n)]
    for y in ys:
        r = r - cyl(1.75, -1, 7, 0, y) - cyl(3.0, 3.0, 7, 0, y)
    return r, ys


def mgn9h_block():
    """MGN9H carriage on an MGN9 rail whose base is z=0: underside at z=2, top at z=10, 20 wide, 39.9 long (incl. seals)."""
    b = box(-10, 10, -19.95, 19.95, 2.0, 10.0) - box(-4.9, 4.9, -21, 21, 1.9, 6.9)
    for x in (-7.5, 7.5):
        for y in (-8.0, 8.0):
            b = b - cyl(1.25, 6.9, 10.1, x, y)
    return b
