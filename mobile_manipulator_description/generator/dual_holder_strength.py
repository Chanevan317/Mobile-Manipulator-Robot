"""Will the printed dual-arm holder hold? Stresses from the Isaac load test (scripts/isaac_dual_load_test.py).

The sim gives the wrench each arm base puts on the holder (`left_arm_mount` / `right_arm_mount`: force + torque at the
arm frame origin, in the base's frame) with both arms stretched out in different directions, swinging, stopping hard
and riding the lift with a payload. This script puts them on the holder's real cross-sections (dual_holder.py, sliced)
and on its bolts:
  the pad + bridge section just outside the side cheek (everything that holds one arm), the legs' bolts through the
  gantry plate (holder tipping forward / back about the plate), the lap-joint bolts between the two halves, and the
  arm base's own M5 x 8 inserts.
Strengths as mono_ring_strength.py (printed, already reduced for 4-5 walls / 15 % infill): PETG 30 MPa; inserts M5 x 8
in PLA ~1200 N (typical, assumed); target safety factor >= 3.

    python3 dual_holder_strength.py ../dual_load_test_0.5kg.json [more json ...]
"""
import sys, os, json, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dual_holder as DH
from mono_ring_strength import bolt_tension, STRENGTH, INSERT, TARGET_SF
from lib import Rz


def wrenches(files):
    """[(phase, side, F (3), M (3)) in the CARRIAGE frame at the arm frame origin]."""
    out = []
    for f in files:
        d = json.load(open(f))['log']
        for ph, rec in d.items():
            if ph.endswith('_effort'): continue
            for side, sy in (('left', 1), ('right', -1)):
                W = rec[f'{side}_arm_mount']['W']
                if W is None: continue
                R = Rz(sy * DH.YAW)[:3, :3]
                out.append((ph, side, R @ np.array(W[:3]), R @ np.array(W[3:])))
    return out


def run(files):
    DH.build_all()
    W = wrenches(files)
    rows = []

    def row(part, what, load, stress, strength, unit='MPa'):
        sf = strength / stress if stress > 0 else float('inf')
        rows.append((part, what, load, f'{stress:.1f} {unit}', f'{strength:.0f} {unit}', sf))

    # 1) the section that carries one whole arm: plane y = 80 (just outside the cheek) through pad + front bridge
    half = DH.PARTS['holder_left']['solid']
    worst = (0, None)
    for yc in (78.0, 90.0):
        A, I_h, c_h, I_v, c_v = _ysection(half, yc)
        for ph, side, F, M in W:
            if side != 'left': continue
            O = np.array([0.0, DH.ARM_Y, DH.AZ]) / 1000.0
            P = np.array([_YC['xc'], yc, _YC['zc']]) / 1000.0
            Mp = M + np.cross(O - P, F)
            s = abs(Mp[0]) * 1000 * c_h / I_h + abs(Mp[2]) * 1000 * c_v / I_v
            if s > worst[0]: worst = (s, (ph, yc, np.round(Mp, 2), _YC['A']))
    s, info = worst
    row('holder (PETG)', f'pad + bridge leaving the cheek (y {info[1]:.0f}, {info[3]:.0f} mm2), worst {info[0]}', f'{np.linalg.norm(info[2][[0, 2]]):.2f} N m', s, STRENGTH['PETG'])
    # 2) whole holder on the gantry plate: both arms' wrenches summed about the plate bolts
    by_phase = {}
    for ph, side, F, M in W:
        O = np.array([0.0, (1 if side == 'left' else -1) * DH.ARM_Y, DH.AZ]) / 1000.0
        P = np.array([-27.5, 0.0, -30.0]) / 1000.0                     # on the plate's back face, between the bolt rows
        f0, m0 = by_phase.get(ph, (np.zeros(3), np.zeros(3)))
        by_phase[ph] = (f0 + F, m0 + M + np.cross(O - P, F))
    tf = tb = tr = 0.0; ph_w = ''
    zs = [z for (_, z) in DH.GANTRY_HOLES]
    for ph, (F, M) in by_phase.items():
        my = abs(M[1]); mx = abs(M[0])
        t_front = bolt_tension(my, 20.0, zs)          # front-heavy: pivots on the plate top (z 20), the low bolts pull
        t_back = bolt_tension(my, DH.LEG['z'][0], zs)  # back-heavy: pivots on the legs' bottom (z -80)
        t_roll = bolt_tension(mx, 62.5, [y for (y, _) in DH.GANTRY_HOLES])
        t = max(t_front, t_back) / 2 + t_roll / 2 + abs(F[2]) / 4
        if t > max(tf, tb): tf, ph_w, tr = t, ph, mx
    rows.append(('holder legs', f'M5 bolt through the gantry plate (worst: {ph_w})', '', f'{tf:.0f} N', 'M5 steel ~5 kN', 5000.0 / tf))
    t_pet = tf / (math.pi * 8.5 * (DH.LEG['x'][1] - DH.LEG['x'][0]))         # head pulling through the 12 mm leg
    row('holder (PETG)', 'M5 head pulling through a leg (12 mm)', f'{tf:.0f} N', t_pet, STRENGTH['PETG'] * 0.6)
    # 3) lap joints between the halves: worst one-sided moment across y = 0, taken by the closest bolt pair
    m0 = 0.0
    for ph, side, F, M in W:
        if side != 'left': continue
        O = np.array([0.0, DH.ARM_Y, DH.AZ]) / 1000.0
        m0 = max(m0, np.linalg.norm((M + np.cross(O, F))[[0, 2]]))
    Fs = m0 / 0.028                                  # couple over the smallest bolt spacing in a lap (28 mm)
    rows.append(('lap joint', 'M5 shear (one arm loaded, the other not)', f'{m0:.2f} N m', f'{Fs:.0f} N', 'M5 steel ~3 kN shear', 3000.0 / Fs))
    tl = min(DH.FRONT['z'][1] - DH.FRONT['zm'], DH.FRONT['zm'] - DH.FRONT['z'][0], DH.REAR['z'][1] - DH.REAR['zm'], DH.REAR['zm'] - DH.REAR['z'][0])
    row('holder (PETG)', f'bolt bearing on the {tl:.1f} mm lap layer', f'{Fs:.0f} N', Fs / (5.0 * tl), STRENGTH['PETG'])
    # 4) the arm base's own inserts: tipping about its outer edge (x +36 from the shoulder)
    mb = max(np.linalg.norm(M[[0, 1]]) for _, _, F, M in W)
    tb = bolt_tension(mb, 36.0, [-80.0, -80.0, -54.0, -54.0])
    rows.append(('arm base (PLA, printed)', 'M5 x 8 inserts in the base bottom', f'{mb:.2f} N m', f'{tb:.0f} N', f'{INSERT[("M5", 8)]:.0f} N', INSERT[('M5', 8)] / tb))
    return rows


_YC = {}


def _ysection(solid, yc):
    """Section of `solid` on the plane y = yc: (centroid info in _YC, I about x through the centroid (bending from Mx,
    stress ~ z), c_z, I about z (bending from Mz, stress ~ x), c_x)."""
    from lib import basis, xf
    Mb = basis((1, 0, 0), (0, 0, 1), (0, -1, 0))          # carriage (x, z, -y) -> (u, v, w): slice at w = -yc
    s = xf(solid, np.linalg.inv(Mb))
    polys = s.slice(-yc).to_polygons()
    A = Su = Sv = Iuu = Ivv = 0.0; pts = []
    for p in polys:
        p = np.asarray(p); q = np.roll(p, -1, axis=0); cr = p[:, 0] * q[:, 1] - q[:, 0] * p[:, 1]
        A += cr.sum() / 2; Su += ((p[:, 0] + q[:, 0]) * cr).sum() / 6; Sv += ((p[:, 1] + q[:, 1]) * cr).sum() / 6
        Ivv += ((p[:, 0] ** 2 + p[:, 0] * q[:, 0] + q[:, 0] ** 2) * cr).sum() / 12
        Iuu += ((p[:, 1] ** 2 + p[:, 1] * q[:, 1] + q[:, 1] ** 2) * cr).sum() / 12
        pts.append(p)
    A = abs(A); uc, vc = Su / A, Sv / A
    if A <= 0: raise RuntimeError('empty section')
    P = np.vstack(pts)
    _YC.update(A=A, xc=uc, zc=vc)
    return A, abs(Iuu) - A * vc * vc, float(np.abs(P[:, 1] - vc).max()), abs(Ivv) - A * uc * uc, float(np.abs(P[:, 0] - uc).max())


if __name__ == '__main__':
    files = [a for a in sys.argv[1:] if a.endswith('.json')]
    rows = run(files)
    print(f'{"part":26s} {"what":78s} {"load":10s} {"stress":10s} {"strength":22s} SF')
    for p, w, l, s, st, sf in rows:
        flag = '' if sf >= TARGET_SF else '  <-- below 3'
        print(f'{p:26s} {w:78s} {l:10s} {s:10s} {st:22s} {sf:5.1f}{flag}')
