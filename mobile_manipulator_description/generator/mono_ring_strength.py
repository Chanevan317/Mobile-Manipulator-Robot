"""Will the mono ring's printed parts hold? Stresses from the Isaac load test (scripts/isaac_mono_load_test.py).

Isaac's bodies are rigid, so the sim gives the LOADS (the wrench through the ring, at the arm base's shoulder, the ring
drive torque) while the arm swings, stops hard and rides the lift with a payload. This script puts the worst of them on
the real cross-sections of the CAD (mono_ring.py, sliced) and simple beam / bolt / gear formulas:
  shelf + ribs where they leave the rotor, hub tube above the disc, bracket leg, the bolts / inserts that hold the ring
  on the bracket, the arm base's 4 M5, the gear teeth (Lewis), the bearings.
Stresses use the SOLID section; a print with 4-5 walls and 15 % infill is weaker inside, so the printed strengths used
below are already reduced (PETG 30 MPa, PLA 30 MPa in the layer plane, 15 MPa across layers) and the target is a safety
factor >= 3 (sustained load, creep, fatigue). Insert pull-out values are typical published figures for heat-set inserts
in PLA (M5 x 4: ~600 N, M5 x 8: ~1200 N), not measured.

    python3 mono_ring_strength.py ../mono_load_test_0.5kg.json [more json ...]
"""
import sys, os, json, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mono_ring as MR
from lib import xf, basis

STRENGTH = {'PETG': 30.0, 'PLA': 30.0, 'PLA_z': 15.0}      # MPa, printed, in the layer plane (PLA_z: across layers)
INSERT = {('M5', 4): 600.0, ('M5', 8): 1200.0, ('M3', 6): 400.0}
TARGET_SF = 3.0
BEARING_C = 19600.0                                          # N, 61820 basic dynamic load rating (catalogue)


def worst(files):
    W = dict(ring_M=0.0, ring_Fz=0.0, ring_Fr=0.0, sh_M=0.0, sh_Fz=0.0, sh_Fr=0.0, ring_T=0.0, sh_T=0.0)
    for f in files:
        d = json.load(open(f))['log']
        for ph, rec in d.items():
            if ph.endswith('_effort'):
                W['ring_T'] = max(W['ring_T'], rec['ring']); W['sh_T'] = max(W['sh_T'], rec['shoulder']); continue
            r, s = rec['left_shoulder_pan'], rec['left_elbow_pan']
            W['ring_M'] = max(W['ring_M'], r['Mxy']); W['ring_Fz'] = max(W['ring_Fz'], r['Fz']); W['ring_Fr'] = max(W['ring_Fr'], r['Fxy'])
            W['sh_M'] = max(W['sh_M'], s['Mxy']); W['sh_Fz'] = max(W['sh_Fz'], s['Fz']); W['sh_Fr'] = max(W['sh_Fr'], s['Fxy'])
    return W


def section(solid, axis, at):
    """Cross-section of `solid` on the plane axis = at ('x' or 'z'); returns (area mm2, I about the horizontal centroid
    axis mm4, c = farthest fibre mm, I about the vertical centroid axis, c_v). In-plane coordinates (u, v): for 'x' the
    plane is (y, z) and v = z (bending from a tipping moment); for 'z' it is (x, y)."""
    if axis == 'x':
        M = basis((0, 1, 0), (0, 0, 1), (1, 0, 0))            # carriage (y, z, x) -> (u, v, w)
        s = xf(solid, np.linalg.inv(M))
    else:
        s = solid
    polys = s.slice(at).to_polygons()
    A = Su = Sv = Iuu = Ivv = 0.0
    pts = []
    for p in polys:
        p = np.asarray(p); q = np.roll(p, -1, axis=0); cr = p[:, 0] * q[:, 1] - q[:, 0] * p[:, 1]
        A += cr.sum() / 2; Su += ((p[:, 0] + q[:, 0]) * cr).sum() / 6; Sv += ((p[:, 1] + q[:, 1]) * cr).sum() / 6
        Ivv += ((p[:, 0] ** 2 + p[:, 0] * q[:, 0] + q[:, 0] ** 2) * cr).sum() / 12
        Iuu += ((p[:, 1] ** 2 + p[:, 1] * q[:, 1] + q[:, 1] ** 2) * cr).sum() / 12
        pts.append(p)
    if A <= 0: return 0, 0, 0, 0, 0
    uc, vc = Su / A, Sv / A
    P = np.vstack(pts)
    I_h = Iuu - A * vc * vc; I_v = Ivv - A * uc * uc
    return A, I_h, float(np.abs(P[:, 1] - vc).max()), I_v, float(np.abs(P[:, 0] - uc).max())


def bolt_tension(M, pivot, positions):
    """Bolts in a row pattern resisting moment M (N m) about a pivot line; tension linear with distance (mm)."""
    d = [abs(p - pivot) for p in positions]
    k = M * 1000.0 / sum(x * x for x in d)
    return max(k * x for x in d)


def run(files):
    W = worst(files)
    MR.build_all()
    rows = []

    def row(part, what, load, stress, strength, unit='MPa'):
        sf = strength / stress if stress > 0 else float('inf')
        rows.append((part, what, load, f'{stress:.1f} {unit}', f'{strength:.0f} {unit}', sf))

    rot = MR.PARTS['rotor']['solid']
    # 1) shelf + ribs leaving the rotor body: moment = the arm's tipping moment at the shoulder + its weight times the
    #    distance from the shoulder axis (L1) to the section (+ the base's own weight, ~3.7 N, 25 mm inside the shoulder)
    for x in (71.0, 76.0, 81.0):
        A, I, c, Iv, cv = section(rot, 'x', x)
        M = W['sh_M'] + W['sh_Fz'] * (MR.L1 - x) / 1000.0 + 3.7 * (MR.L1 - 25.0 - x) / 1000.0
        row('rotor (PETG)', f'shelf + ribs at r {x:.0f} (section {A:.0f} mm2)', f'{M:.2f} N m', M * 1000.0 * c / I, STRENGTH['PETG'])
    # 2) hub tube above the disc (the whole arm's tipping moment through the bearings)
    A, I, c, Iv, cv = section(MR.PARTS['ring_base']['solid'], 'z', MR.DISC_Z[1] + 2.0)
    Imin = min(I, Iv); cmax = max(c, cv)
    row('ring base (PLA)', f'hub tube above the disc (section {A:.0f} mm2)', f'{W["ring_M"]:.2f} N m', W['ring_M'] * 1000.0 * cmax / Imin, STRENGTH['PLA_z'])
    # 3) bearings: the tipping moment as a couple on the two bearings (centres 21 mm apart) + the radial force
    span = (MR.B2_Z - MR.B1_Z) / 1000.0
    Fr = W['ring_M'] / span + W['ring_Fr']
    rows.append(('bearings 61820', 'radial load per bearing', f'{Fr:.0f} N', f'{Fr:.0f} N', f'C {BEARING_C:.0f} N', BEARING_C / Fr))
    p = Fr / (MR.BRG['d'] * MR.BRG['B'])
    row('ring base (PLA)', 'bearing seat pressure on the hub', f'{Fr:.0f} N', p, STRENGTH['PLA'])
    row('rotor (PETG)', 'bearing seat pressure in the rotor', f'{Fr:.0f} N', Fr / (MR.BRG['D'] * MR.BRG['B']), STRENGTH['PETG'])
    # 4) ring held on the bracket: arm pointing back (pivot on the bracket's rear edge) / forward (pivot on the plate's
    #    front edge) / sideways (pivot on the disc edge); bolts: 2 from below at x -50 (M5 x 4 inserts in the disc),
    #    2 hub bolts at x -33 (M5 x 8 inserts in the bracket)
    xs = [-50.0, -50.0] + [b[0] for b in MR.HUB_BOLTS]
    t_back = bolt_tension(W['ring_M'], MR.BRACKET['x0'], xs)
    t_front = bolt_tension(W['ring_M'], -21.5, xs)
    ys = [20.0, -20.0] + [b[1] for b in MR.HUB_BOLTS]
    t_side = bolt_tension(W['ring_M'], MR.DISC_R, ys)
    T = max(t_back, t_front, t_side) + W['ring_Fz'] / 4
    rows.append(('ring to bracket', f'bolt tension (back {t_back:.0f} / front {t_front:.0f} / side {t_side:.0f} N)', f'{W["ring_M"]:.2f} N m',
                 f'{T:.0f} N', f'{INSERT[("M5", 4)]:.0f} N (M5 x 4 insert)', INSERT[('M5', 4)] / T))
    # old design (2 bolts at x -50 only) for comparison
    t_old = bolt_tension(W['ring_M'], -60.0, [-50.0, -50.0])
    rows.append(('(earlier mount)', 'only the 2 bolts at x -50, arm pointing back', f'{W["ring_M"]:.2f} N m', f'{t_old:.0f} N',
                 f'{INSERT[("M5", 4)]:.0f} N', INSERT[('M5', 4)] / t_old))
    # 5) bracket leg (12 mm, 112 wide), bending at its top: the ring's moment goes down the leg to the plate bolts
    B = MR.BRACKET; t = B['x1'] - B['leg_x0']; w = 2 * B['leg_w']
    row('L bracket (PLA)', f'leg {t:.0f} x {w:.0f} mm, bending', f'{W["ring_M"]:.2f} N m', W['ring_M'] * 1000.0 * 6 / (w * t * t), STRENGTH['PLA'])
    tp = W['ring_M'] * 1000.0 / 60.0 / 2 + W['ring_Fz'] / 4                                   # plate bolts 60 mm apart vertically
    rows.append(('L bracket', 'M5 bolts through the gantry plate (tension)', f'{W["ring_M"]:.2f} N m', f'{tp:.0f} N', 'M5 steel ~5 kN', 5000.0 / tp))
    # 6) the arm base on the shelf: 4 M5 (base inserts M5 x 8, heads sunk in the PETG shelf); the base tips about its
    #    outer edge (x +36 from the shoulder): bolts at x -80 / -54
    tb = bolt_tension(W['sh_M'] + W['sh_Fz'] * 0.0, 36.0, [-80.0, -80.0, -54.0, -54.0])
    rows.append(('arm base (PLA, printed)', 'M5 x 8 inserts in the base bottom', f'{W["sh_M"]:.2f} N m', f'{tb:.0f} N', f'{INSERT[("M5", 8)]:.0f} N', INSERT[('M5', 8)] / tb))
    head = tb / (math.pi * 8.5 * 6.5)
    row('rotor (PETG)', 'M5 head pulling through the shelf (6.5 mm under the head)', f'{tb:.0f} N', head, STRENGTH['PETG'] * 0.6)
    # 7) gear teeth (Lewis): measured peak ring torque and the drive's stall (7.6 N m through the 4:1 gear)
    for Tq, lab in ((W['ring_T'], 'load test peak'), (0.6 * 1.9 * MR.ZR / MR.ZP, 'stall at the 60 % torque limit'),
                    (1.9 * MR.ZR / MR.ZP, 'stall, NO torque limit')):
        Ft = Tq / (MR.MOD * MR.ZR / 2 / 1000.0)
        b = MR.GEAR_Z[1] - MR.GEAR_Z[0]
        row('pinion (PETG)', f'tooth root, {lab}', f'{Tq:.2f} N m ring', Ft / (b * MR.MOD * 0.309), STRENGTH['PETG'])
        row('rotor gear (PETG)', f'tooth root, {lab}', f'{Tq:.2f} N m ring', Ft / (b * MR.MOD * 0.43), STRENGTH['PETG'])
    return W, rows


if __name__ == '__main__':
    files = [a for a in sys.argv[1:] if a.endswith('.json')]
    W, rows = run(files)
    print('worst loads:', {k: round(v, 2) for k, v in W.items()})
    print(f'{"part":26s} {"what":62s} {"load":14s} {"stress":12s} {"strength":22s} SF')
    for p, w, l, s, st, sf in rows:
        flag = '' if sf >= TARGET_SF else '  <-- below 3'
        print(f'{p:26s} {w:62s} {l:14s} {s:12s} {st:22s} {sf:5.1f}{flag}')
