"""Will the printed mono holder (H01, mono_holder.py) hold? Offline strength check, no simulator.

Loads come from the URDF (`prototype_mono_arm`: link masses + centres of mass, forward kinematics), over a grid of
arm poses (shoulder / elbow / wrist pan / pitch) with a payload at the TCP. Each pose is checked
  static     gravity only
  dynamic    gravity x 1.3 (the lift starting / stopping: the ring robot's Isaac load test gave 1.27) + 3 m/s2 sideways in
             the worst of 4 directions (base braking; the robot tips over at about that) + the shoulder servo's full stall
             torque 1.9 N m as the reaction on the base (a hard stop or a hit can't put more through the shoulder)
The wrench is put on the holder's real cross-sections (H01 sliced) and on its bolts / inserts:
  pad        the 12 mm plate under the arm base, between the arm and the rear bridge (layers: in plane)
  bridge     the rear bridge between the pad and each leg (y = +-35): the load runs ACROSS the print layers here
             (H01 prints on its back, layers = carriage y planes)
  legs       both legs above the top bolts (z -5), as one section (the arm's twist is a couple between them)
  bolts      4 x M5 through the gantry plate (pivot on the legs' bottom edge), the heads pulling through the 12 mm legs
  inserts    the arm base's own 4 x M5 x 8 inserts (PLA, printed arm part 02)
  camera     post bottom and the forward bar (BAR2, across the layers) with the scene camera mast on it
Strengths as mono_ring_strength.py (printed, already reduced for 4-5 walls / 15 % infill): PETG 30 MPa in the layer plane,
PETG_z 15 MPa across layers (assumed, half); M5 x 8 insert in PLA ~1200 N. Target safety factor >= 3.
Also prints the TCP force (pushing down / sideways at full reach) that would break the weakest element: what a crash
must stay under (the lift's NEMA 23 + 8 mm lead screw can push far more: limit its current / following error).

    python3 mono_holder_strength.py [payload_kg ...]          (default 0.5 1.0)
"""
import sys, os, math, itertools
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mono_holder as MH
from mono_ring_strength import bolt_tension, STRENGTH, INSERT, TARGET_SF
from lib import box, basis, xf
from urdf_fk import Robot, org

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
G = 9.81
DYN_G, DYN_A, STALL = 1.3, 3.0, 1.9
PETG, PETG_Z = STRENGTH['PETG'], 15.0
ARM_LINKS = ['left_base', 'left_upper_arm', 'left_forearm', 'left_wrist', 'left_tilt', 'left_hand', 'left_finger_right',
             'left_finger_left', 'left_wrist_camera']
CAM_LINKS = ['scene_mast', 'scene_camera']


def _inertials(rob):
    out = {}
    for nm, l in rob.links.items():
        i = l.find('inertial')
        if i is None: continue
        out[nm] = (float(i.find('mass').get('value')), org(i)[:3, 3])
    return out


ROB = Robot(PKG, 'prototype_mono_arm')
INER = _inertials(ROB)


def point_masses(q, payload, links):
    """[(mass kg, position in the CARRIAGE frame, m)] for these links at joint values q (+ payload at the TCP)."""
    W = ROB.fk(q); Ci = np.linalg.inv(W['carriage'])
    pts = []
    for nm in links:
        if nm not in INER or INER[nm][0] <= 0: continue
        m, c = INER[nm]; p = (Ci @ W[nm] @ np.r_[c, 1.0])[:3]; pts.append((m, p))
    if payload > 0 and 'left_base' in links:
        pts.append((payload, (Ci @ W['left_tcp'])[:3, 3]))
    return pts


def wrench(pts, O, g=1.0, acc=(0.0, 0.0), Mz=0.0, tcp=None, Ftcp=None):
    """Force (N) + moment (N m) about O (m, carriage frame) that the masses put on the holder: weight x g, inertia of a
    horizontal acceleration `acc` (m/s2: the holder pushes the masses, they push back), an extra torque about z, and an
    optional external force Ftcp at the point tcp."""
    F = np.zeros(3); Mo = np.zeros(3)
    for m, p in pts:
        f = m * np.array([-acc[0], -acc[1], -G * g]); F += f; Mo += np.cross(p - O, f)
    if Ftcp is not None:
        F += Ftcp; Mo += np.cross(tcp - O, Ftcp)
    Mo[2] += Mz
    return F, Mo


def poses():
    for q1 in range(-115, 116, 23):
        for q2 in range(-125, 126, 25):
            for q3 in (-120, -60, 0, 60, 120):
                for pitch in (0, 90):
                    yield {'left_shoulder_pan': math.radians(q1), 'left_elbow_pan': math.radians(q2),
                           'left_wrist_pan': math.radians(q3), 'left_wrist_pitch': math.radians(pitch),
                           'left_finger_right': 0.03, 'left_finger_left': 0.03}


# ------------------------------------------------------------------ sections (mm, carriage frame)
def section(solid, n, at, region):
    """Slice `solid` (inside the box `region`) on the plane n . x = at. Returns the section in plane coordinates
    (u, v): area, centroid (carriage, mm), the in-plane axes u / v (carriage vectors), Iuu, Ivv, Iuv about the centroid
    and the polygon vertices (u, v) relative to the centroid."""
    n = np.asarray(n, float); n /= np.linalg.norm(n)
    u = np.cross((0, 0, 1) if abs(n[2]) < 0.9 else (1, 0, 0), n); u /= np.linalg.norm(u); v = np.cross(n, u)
    B = basis(u, v, n)
    s = xf(solid ^ box(*region), np.linalg.inv(B))
    polys = s.slice(at).to_polygons()
    A = Su = Sv = Iuu = Ivv = Iuv = 0.0; P = []
    for p in polys:
        p = np.asarray(p); q = np.roll(p, -1, axis=0); cr = p[:, 0] * q[:, 1] - q[:, 0] * p[:, 1]
        A += cr.sum() / 2; Su += ((p[:, 0] + q[:, 0]) * cr).sum() / 6; Sv += ((p[:, 1] + q[:, 1]) * cr).sum() / 6
        Ivv += ((p[:, 0] ** 2 + p[:, 0] * q[:, 0] + q[:, 0] ** 2) * cr).sum() / 12      # int u^2
        Iuu += ((p[:, 1] ** 2 + p[:, 1] * q[:, 1] + q[:, 1] ** 2) * cr).sum() / 12      # int v^2
        Iuv += ((p[:, 0] * q[:, 1] + 2 * p[:, 0] * p[:, 1] + 2 * q[:, 0] * q[:, 1] + q[:, 0] * p[:, 1]) * cr).sum() / 24
        P.append(p)
    if A <= 1e-6: raise RuntimeError(f'empty section n {n} at {at}')
    uc, vc = Su / A, Sv / A
    c = B[:3, :3] @ np.array([uc, vc, at])
    P = np.vstack(P) - (uc, vc)
    return dict(A=A, c=c, u=u, v=v, n=n, Iuu=Iuu - A * vc * vc, Ivv=Ivv - A * uc * uc, Iuv=Iuv - A * uc * vc, P=P)


def stress(S, F, Mc, torsion):
    """Max normal stress (MPa) and torsion shear (MPa) on section S from force F (N) and moment Mc (N m) about its
    centroid. Bending: unsymmetric-beam formula at every vertex. torsion(T in N mm) -> shear (method per section)."""
    N = F @ S['n']; Mu = Mc @ S['u'] * 1000; Mv = Mc @ S['v'] * 1000; T = Mc @ S['n'] * 1000
    Iuu, Ivv, Iuv = S['Iuu'], S['Ivv'], S['Iuv']; D = Iuu * Ivv - Iuv * Iuv
    u, v = S['P'][:, 0], S['P'][:, 1]
    # moment about u bends with stress ~ v, about v with stress ~ -u (sigma = -(Mv Iuu + Mu Iuv) u / D + (Mu Ivv + Mv Iuv) v / D)
    sig = N / S['A'] + (Mu * Ivv + Mv * Iuv) * v / D - (Mv * Iuu + Mu * Iuv) * u / D
    return float(np.abs(sig).max()), torsion(abs(T))


def rect_torsion(b, t):
    a = 1.0 / 3.0 * (1 - 0.63 * t / b) if b >= t else rect_torsion(t, b)
    return lambda T: T / (a * b * t * t)


# ------------------------------------------------------------------ the check
def run(payloads=(0.5, 1.0)):
    MH.build_all()
    h = PARTS_H = MH.PARTS['holder']['solid']
    big = 1e4
    ax, ay, az = -MH.ARM_DX, MH.ARM_DY, MH.AZ
    O = np.array([ax, ay, az]) / 1000.0                                # arm frame origin, m
    secs = []                                                          # (name, section, side the load comes from, torsion, strength)
    for x in (-45.0, -60.0, -80.0, -95.0):                             # torsion: the pad plate alone (the rib adds little)
        S = section(h, (-1, 0, 0), -x, (-big, big, -big, big, -40.0, 60.0))
        w = float(np.ptp([p for p in S['P'][:, 0]]))                 # u = carriage y across the pad
        secs.append((f'pad + rib at x {x:.0f} ({S["A"]:.0f} mm2)', S, 'arm', rect_torsion(w, MH.PAD[1] - MH.PAD[0]), PETG))
    for sy in (1, -1):
        S = section(h, (0, sy, 0), 35.0, (-45.0, -25.0, -big, big, -90.0, 40.0))
        bb = S['P'].max(0) - S['P'].min(0)
        secs.append((f'rear bridge at y {sy * 35:+.0f} ({S["A"]:.0f} mm2, across layers)', S, 'half', rect_torsion(*sorted(bb)[::-1]), PETG_Z))
    S = section(h, (0, 0, 1), -5.0, (-60.0, -20.0, -big, big, -big, big))
    dleg = 2 * np.mean(MH.LEG['y']); aleg = S['A'] / 2
    secs.append((f'both legs at z -5 ({S["A"]:.0f} mm2)', S, 'all', lambda T: 1.5 * (T / dleg) / aleg, PETG))
    # camera: post bottom (z 30) and BAR2 next to BAR1 (across layers); loads = mast + camera masses
    Sp = section(h, (0, 0, 1), 30.0, (MH.POST['x'][0] - 1, MH.POST['x'][1] + 1, MH.POST['y'][0] - 1, MH.POST['y'][1] + 1, -big, big))
    bbp = Sp['P'].max(0) - Sp['P'].min(0)
    yb = MH.BAR1['y'][0] - 1.0                                          # BAR2 sticks out behind BAR1 (-y) under the mast
    Sb = section(h, (0, -1, 0), -yb, (MH.BAR2['x'][0] - 1, MH.BAR2['x'][1] + 1, -big, big, MH.BAR[0] - 1, MH.BAR[1] + 1))
    bbb = Sb['P'].max(0) - Sb['P'].min(0)
    xb = -80.0                                                         # BAR1 between the post and the mast
    S1 = section(h, (-1, 0, 0), -xb, (-big, big, -big, big, MH.BAR[0] - 1, MH.BAR[1] + 1))
    bb1 = S1['P'].max(0) - S1['P'].min(0)

    for payload in payloads:
        worst = {}

        def keep(key, val, info):
            if val > worst.get(key, (0, None))[0]: worst[key] = (val, info)

        cam_pts = point_masses({}, 0.0, CAM_LINKS)
        for q in poses():
            pts = point_masses(q, payload, ARM_LINKS)
            cases = [('static', dict(g=1.0))]
            for a, mz in itertools.product(((DYN_A, 0), (-DYN_A, 0), (0, DYN_A), (0, -DYN_A)), (STALL, -STALL)):
                cases.append((f'dynamic (a {a}, Mz {mz:+.1f})', dict(g=DYN_G, acc=a, Mz=mz)))
            for cname, kw in cases:
                F, Mo = wrench(pts, O, **kw)
                key = 'static' if cname == 'static' else 'dynamic'
                for nm, S, side, tor, st in secs:
                    c = S['c'] / 1000.0
                    Mc = Mo + np.cross(O - c, F)
                    if side == 'half': Fh, Mh = F / 2, Mc / 2
                    else: Fh, Mh = F, Mc
                    sig, tau = stress(S, Fh, Mh, tor)
                    eq = math.sqrt(sig * sig + 3 * tau * tau)
                    keep((key, nm), eq / st, (eq, sig, tau, st, q, cname))
                # bolts through the gantry plate: wrench about the plate's back face between the bolt rows
                P = np.array([MH.LEG['x'][1], 0.0, -42.5]) / 1000.0
                Mp = Mo + np.cross(O - P, F)
                zs = [z for (_, z) in MH.GANTRY_HOLES]; ys = [y for (y, _) in MH.GANTRY_HOLES]
                t = bolt_tension(abs(Mp[1]), MH.LEG['z'][0], zs) / 2 + bolt_tension(abs(Mp[2]), MH.LEG['y'][1], ys) / 2 + max(0.0, -F[0]) / 4
                keep((key, 'bolt'), t / 5000.0 * TARGET_SF, (t, q, cname))
                # arm base inserts: moment at the base underside, tipping about its rim (r 40) in the worst direction
                Ob = np.array([ax, ay, MH.BASE_Z]) / 1000.0
                Mb = Mo + np.cross(O - Ob, F)
                Mh_ = math.hypot(Mb[0], Mb[1]) * 1000
                if Mh_ > 0:
                    tip = np.array([-Mb[1], Mb[0]]) / math.hypot(Mb[0], Mb[1])  # the load leans toward this (carriage xy)
                    bolts = [np.array([ax - bx, ay - by]) for bx, by in MH.BASE_M5]   # arm yaw 180: arm x -> carriage -x
                    d = [40.0 - (b - (ax, ay)) @ tip for b in bolts]
                    k = Mh_ / sum(x * x for x in d); tb = k * max(d) - F[2] / 4
                    keep((key, 'insert'), tb / INSERT[('M5', 8)] * TARGET_SF, (tb, q, cname))
        # camera post / bar (pose independent)
        for cname, kw in [('static', dict(g=1.0))] + [(f'dynamic (a {a})', dict(g=DYN_G, acc=a)) for a in ((DYN_A, 0), (-DYN_A, 0), (0, DYN_A), (0, -DYN_A))]:
            key = 'static' if cname == 'static' else 'dynamic'
            for nm, S, tor, st in ((f'camera post at z 30 ({Sp["A"]:.0f} mm2)', Sp, rect_torsion(*sorted(bbp)[::-1]), PETG),
                                   (f'camera bar BAR1 at x {xb:.0f} ({S1["A"]:.0f} mm2)', S1, rect_torsion(*sorted(bb1)[::-1]), PETG),
                                   (f'camera bar BAR2 at y {yb:.0f} ({Sb["A"]:.0f} mm2, across layers)', Sb, rect_torsion(*sorted(bbb)[::-1]), PETG_Z)):
                c = S['c'] / 1000.0
                F, Mc = wrench(cam_pts, c, **kw)                       # the whole mast + camera is on the far side of each cut
                sig, tau = stress(S, F, Mc, tor); eq = math.sqrt(sig * sig + 3 * tau * tau)
                keep((key, nm), eq / st, (eq, sig, tau, st, {}, cname))

        print(f'\n=== payload {payload} kg at the TCP ({len(list(poses()))} poses) ===')
        print(f'{"element":62s} {"case":8s} {"stress / load":>16s} {"strength":>10s} {"SF":>6s}   worst pose (q1 q2 q3 pitch) / case')
        for (key, nm), (r, info) in sorted(worst.items(), key=lambda kv: (kv[0][1], kv[0][0] != 'static')):
            sf = TARGET_SF / r if nm in ('bolt', 'insert') else 1.0 / r
            if nm == 'bolt':
                t, q, cn = info; label, val, strg = 'M5 bolt through the gantry plate (tension)', f'{t:.0f} N', 'M5 ~5 kN'
            elif nm == 'insert':
                t, q, cn = info; label, val, strg = 'arm base M5 x 8 insert in PLA (pull-out)', f'{t:.0f} N', f'{INSERT[("M5", 8)]:.0f} N'
            else:
                eq, sig, tau, st, q, cn = info; label, val, strg = nm, f'{eq:.1f} MPa', f'{st:.0f} MPa'
            qs = ' '.join(f'{math.degrees(q[j]):.0f}' for j in ('left_shoulder_pan', 'left_elbow_pan', 'left_wrist_pan', 'left_wrist_pitch')) if q else '-'
            flag = '' if sf >= TARGET_SF else '  <-- below 3'
            print(f'{label:62s} {key:8s} {val:>16s} {strg:>10s} {sf:6.1f}   {qs} / {cn}{flag}')
        if payload == payloads[0]:
            crash(secs, O, payload)


def crash(secs, O, payload):
    """TCP force (static weights + payload included) at which the weakest pad / bridge / leg section reaches its
    strength (SF 1), the arm stretched straight out (worst lever), pushing up / down (a lift crash) and sideways."""
    print('\nTCP force that breaks the holder (arm stretched, static loads included):')
    for q1 in (0, 90, -90):
        q = {'left_shoulder_pan': math.radians(q1), 'left_wrist_pitch': 0.0}
        pts = point_masses(q, payload, ARM_LINKS)
        tcp = point_masses(q, 1e-9, ARM_LINKS)[-1][1]
        out = []
        for lab, d in (('down', (0, 0, -1)), ('up', (0, 0, 1)), ('sideways', None)):
            if d is None:                                           # perpendicular to the arm, horizontal
                r = tcp - O; d = np.cross((0, 0, 1), r); d /= np.linalg.norm(d)
            lo, hi = 0.0, 5000.0
            for _ in range(40):
                f = (lo + hi) / 2; F, Mo = wrench(pts, O, tcp=tcp, Ftcp=f * np.asarray(d, float))
                bad = False
                for nm, S, side, tor, st in secs:
                    c = S['c'] / 1000.0; Mc = Mo + np.cross(O - c, F)
                    Fh, Mh = (F / 2, Mc / 2) if side == 'half' else (F, Mc)
                    sig, tau = stress(S, Fh, Mh, tor)
                    if math.sqrt(sig * sig + 3 * tau * tau) > st: bad = True; break
                lo, hi = (lo, f) if bad else (f, hi)
            out.append(f'{lab} {lo:.0f} N')
        print(f'  shoulder {q1:+4d} deg, reach {np.linalg.norm((tcp - O)[:2]) * 1000:.0f} mm: ' + ', '.join(out))


if __name__ == '__main__':
    pl = [float(a) for a in sys.argv[1:]] or [0.5, 1.0]
    run(pl)
