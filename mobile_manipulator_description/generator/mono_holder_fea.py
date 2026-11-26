"""FEA of the mono holder H01 (voxel_fea.py): where the arm's load goes in, how it spreads through the holder, and where
the stress peaks. Complements mono_holder_strength.py (beam formulas on chosen sections) with the real 3D shape.

Model (carriage frame, mm):
  holder     H01 (mono_holder.py), voxels of h mm, PETG isotropic E 1800 MPa, nu 0.35 (stresses barely depend on E here)
  arm base   a block of the pad's footprint, 40 mm tall (the printed base 01 + 02), 50 x stiffer, bonded to the pad top
  plate      the legs' faces against the gantry plate are held fixed: 4 x M5 preloaded to ~1-3 kN clamp them, far
             above the ~50-70 N the loads try to pull them off with (mono_holder_strength.py), so they don't slip or open
  loads      the arm's wrench (mono_holder_strength: URDF masses, 0.5 kg payload) spread rigidly over the block top; the
             scene camera's on the mast pocket floor
Strength: PETG 30 MPa in the layer plane (von Mises), 15 MPa across the layers (H01 prints on its back: layers are
carriage y planes; interlayer stress = sqrt(max(s_yy, 0)^2 + 3 (t_xy^2 + t_yz^2))). Voxel corners are sharp, as the CAD
is: the max is a real (if mesh-sensitive) concentration; the 99th percentile shows the region's working level.

    python3 mono_holder_fea.py [h_mm]          (default 2.0) -> table + ../../CAD/mono_holder/fea_mono_holder.png
"""
import sys, os, math, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mono_holder as MH
import mono_holder_strength as MS
import voxel_fea as VF
from lib import CS, xf

PETG, PETG_Z, STIFF = 30.0, 15.0, 50.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'CAD', 'mono_holder', 'fea_mono_holder.png')


BLOCK_H = 10.0                    # the arm base stand-in: a 10 mm plate 50 x stiffer than PETG (rigid next to the 12 mm pad)


def base_block():
    sh = (CS.circle(40.0, 96) + CS([[(-89.0, -24.5), (-46.0, -24.5), (-46.0, 24.5), (-89.0, 24.5)]])).hull()
    return xf(sh.extrude(BLOCK_H).translate((0, 0, MH.PAD[1] - MH.AZ)), MH.arm_frame())


def regions(c):
    x, y, z = c.T
    R = np.full(len(c), 'post / bar', dtype=object)
    R[(z < MH.PAD[1]) & (x < MH.REAR['x'][0])] = 'pad'
    R[(z < MH.PAD[0]) & (x < MH.REAR['x'][0])] = 'rib'
    R[(x >= MH.REAR['x'][0]) & (x <= MH.REAR['x'][1]) & (np.abs(y) < MH.LEG['y'][0]) & (z < MH.PAD[1])] = 'bridge + web'
    R[(x >= MH.LEG['x'][0]) & (np.abs(y) >= MH.LEG['y'][0]) & (z < MH.PAD[1])] = 'legs'
    R[(x > MH.LEG['x'][1]) & (z < MH.PAD[1] + 1)] = 'lip'
    return R


def cases(payload):
    O = np.array([-MH.ARM_DX, MH.ARM_DY, MH.AZ]) / 1000.0
    out = []
    for lab, q1, kw in (('arm straight out, braking sideways (worst bridge)', 0, dict(g=MS.DYN_G, acc=(0, MS.DYN_A), Mz=MS.STALL)),
                        ('arm straight out, braking along it', 0, dict(g=MS.DYN_G, acc=(MS.DYN_A, 0), Mz=MS.STALL)),
                        ('arm to the robot front (+92), braking', 92, dict(g=MS.DYN_G, acc=(0, MS.DYN_A), Mz=MS.STALL)),
                        ('arm to the robot back (-92), braking', -92, dict(g=MS.DYN_G, acc=(0, -MS.DYN_A), Mz=-MS.STALL)),
                        ('arm straight out, gravity only', 0, dict(g=1.0))):
        q = {'left_shoulder_pan': math.radians(q1), 'left_finger_right': 0.03, 'left_finger_left': 0.03}
        F, M = MS.wrench(MS.point_masses(q, payload, MS.ARM_LINKS), O, **kw)
        cam = MS.point_masses({}, 0.0, MS.CAM_LINKS)
        Fc, Mc = MS.wrench(cam, O, **{k: v for k, v in kw.items() if k != 'Mz'})
        out.append((lab, F, M, Fc, Mc, O))
    q = {'left_shoulder_pan': 0.0}
    pts = MS.point_masses(q, payload, MS.ARM_LINKS); tcp = pts[-1][1]
    F, M = MS.wrench(pts, O, tcp=tcp, Ftcp=np.array([0, 0, -50.0]))
    Fc, Mc = MS.wrench(MS.point_masses({}, 0.0, MS.CAM_LINKS), O)
    out.append(('CRASH: 50 N pushing down on the stretched hand', F, M, Fc, Mc, O))
    return out


def run(h=2.0, payload=0.5):
    t0 = time.time()
    MH.build_all()
    H = MH.PARTS['holder']['solid']
    o = (-40.0 - 200 * h, -200 * h, 0.0)                               # node planes on x -40 / -28 (leg faces)
    vh = VF.voxelize(H, h, o); vb = VF.voxelize(base_block(), h, o)
    key = lambda v: (v[:, 0].astype(np.int64) + 5000) * 10 ** 8 + (v[:, 1] + 5000) * 10 ** 4 + (v[:, 2] + 5000)
    vb = vb[~np.isin(key(vb), key(vh))]                               # the block only where the holder isn't
    vox = np.vstack([vh, vb]); scale = np.r_[np.ones(len(vh)), np.full(len(vb), STIFF)]
    m = VF.Model(vox, h, o, scale=scale)
    nh = len(vh)
    print(f'h {h} mm: holder {nh} voxels ({nh * h ** 3 / 1000:.0f} cm3, CAD {H.volume() / 1000:.0f}), base block {len(vb)}, '
          f'{3 * len(m.xyz)} DOF ({time.time() - t0:.0f} s)', flush=True)
    hold_nodes = np.unique(m.conn[:nh])
    X = m.xyz
    face = hold_nodes[(np.abs(X[hold_nodes, 0] - MH.LEG['x'][1]) < 1e-6) & (np.abs(X[hold_nodes, 1]) >= MH.LEG['y'][0] - 1e-6)
                      & (X[hold_nodes, 2] <= MH.PAD[1])]
    m.factor(face)
    print(f'fixed {len(face)} leg-face nodes; solver {m.method} ready ({time.time() - t0:.0f} s)', flush=True)
    bn = np.unique(m.conn[nh:]); top = bn[np.abs(X[bn, 2] - X[bn, 2].max()) < 1e-6]
    pk = hold_nodes[(np.abs(X[hold_nodes, 2] - round(MH.CAM_Z / h) * h) < h / 2 + 1e-6) &
                    (X[hold_nodes, 0] > MH.POCKET['x'][0]) & (X[hold_nodes, 0] < MH.POCKET['x'][1]) &
                    (X[hold_nodes, 1] > MH.POCKET['y'][0]) & (X[hold_nodes, 1] < MH.POCKET['y'][1])]
    R = regions(m.cen[:nh]); names = ['pad', 'rib', 'bridge + web', 'legs', 'lip', 'post / bar']
    Vol = h ** 3; Dinv = np.linalg.inv(m.D)
    results = []
    for lab, F, M, Fc, Mc, O in cases(payload):
        Fn = np.zeros((len(X), 3))
        Fn[top] += VF.rbe3(X, top, F, M * 1000.0, O * 1000.0)
        if len(pk): Fn[pk] += VF.rbe3(X, pk, Fc, Mc * 1000.0, O * 1000.0)
        u = m.solve(Fn); S = m.stress(u)[:nh]
        if m.method == 'amg': print(f'  [{lab}: CG {m.last_iters} iterations, residual {m.last_res:.1e}, {time.time() - t0:.0f} s]', flush=True)
        vm = VF.von_mises(S); il = np.sqrt(np.maximum(S[:, 1], 0) ** 2 + 3 * (S[:, 5] ** 2 + S[:, 3] ** 2))
        U = 0.5 * np.einsum('ij,ij->i', S, S @ Dinv.T) * Vol           # strain energy per voxel (N mm)
        Rc = m.reactions(u, Fn)[face]
        rows = []
        for nm in names:
            s = R == nm
            if not s.any(): continue
            i_vm = np.argmax(np.where(s, vm, -1)); i_il = np.argmax(np.where(s, il, -1))
            rows.append(dict(name=nm, vm99=np.percentile(vm[s], 99), vm=vm[i_vm], at_vm=m.cen[i_vm], il99=np.percentile(il[s], 99),
                             il=il[i_il], at_il=m.cen[i_il], U=U[s].sum() / U.sum() * 100,
                             sf=min(PETG / np.percentile(vm[s], 99), PETG_Z / max(np.percentile(il[s], 99), 1e-9)),
                             sfmax=min(PETG / vm[i_vm], PETG_Z / max(il[i_il], 1e-9))))
        Y = X[face, 1]; Z = X[face, 2]
        react = {f'{"+" if sy > 0 else "-"}y leg, {zz}': Rc[(np.sign(Y) == sy) & ((Z > -42.5) == (zz == 'upper half'))]
                 for sy in (1, -1) for zz in ('upper half', 'lower half')}
        results.append((lab, F, M, rows, react, vm, il, u))
        # the arm base moves as a rigid body: fit u = t + w x r over the whole block -> where that puts the stretched TCP
        P = X[bn]; c = P.mean(0); r = P - c
        A = np.zeros((3 * len(bn), 6)); A[:, :3] = np.tile(np.eye(3), (len(bn), 1))
        for i in range(3):
            e = np.zeros(3); e[i] = 1.0; A[:, 3 + i] = np.cross(e, r).ravel()
        sol = np.linalg.lstsq(A, u[bn].ravel(), rcond=None)[0]; t, w = sol[:3], sol[3:]
        tcp = MS.point_masses({'left_shoulder_pan': 0.0}, 1e-9, MS.ARM_LINKS)[-1][1] * 1000.0
        dt = t + np.cross(w, tcp - c)
        print(f'\n--- {lab}: F {np.round(F, 1)} N, M {np.round(M, 2)} N m at the arm frame; arm base tilts {math.degrees(np.linalg.norm(w[:2])):.2f} deg, '
              f'twists {math.degrees(w[2]):.2f} deg -> the stretched TCP moves {np.round(dt, 1)} mm; camera pocket {np.linalg.norm(u[pk], axis=1).max() if len(pk) else 0:.2f} mm')
        print(f'{"region":14s} {"energy %":>8s} {"vM 99%":>7s} {"vM max":>7s} {"at (x y z)":>18s} {"layer 99%":>9s} {"layer max":>9s} {"at (x y z)":>18s} {"SF 99%":>6s} {"SF max":>6s}')
        for r in rows:
            print(f'{r["name"]:14s} {r["U"]:8.1f} {r["vm99"]:7.2f} {r["vm"]:7.2f} {str(np.round(r["at_vm"]).astype(int)):>18s} '
                  f'{r["il99"]:9.2f} {r["il"]:9.2f} {str(np.round(r["at_il"]).astype(int)):>18s} {r["sf"]:6.1f} {r["sfmax"]:6.1f}')
        print('plate reactions on the holder (N; +x = pulling it onto the plate = the bolts must hold, -x = pressing):')
        for k, r in react.items():
            print(f'  {k:22s} pull {r[r[:, 0] > 0, 0].sum():7.1f}  press {-r[r[:, 0] < 0, 0].sum():7.1f}  shear y {r[:, 1].sum():7.1f} z {r[:, 2].sum():7.1f}')
    render(m, nh, R, results, h)
    print(f'\npicture: {os.path.normpath(OUT)} ({time.time() - t0:.0f} s)')
    return results


def render(m, nh, R, results, h):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    lab, F, M, rows, react, vm, il, u = results[0]
    C = m.cen[:nh]
    fig, ax = plt.subplots(2, 3, figsize=(21, 12.5))
    sl = [('side cut y = 0 (pad, rib, web)', 1, 0.0, (0, 2), 'x (mm, robot left <-)', 'z (mm)'),
          ('cut z = -5 (legs, web)', 2, -5.0, (0, 1), 'x (mm)', 'y (mm, robot front ->)'),
          ('cut x = -35.5 (rear bridge + web, seen from behind)', 0, -35.5, (1, 2), 'y (mm, robot front ->)', 'z (mm)')]
    for col, (title, axn, at, (a, b), xl, yl) in enumerate(sl):
        s = np.abs(C[:, axn] - at) <= h / 2 + 1e-6
        if not s.any():
            s = np.abs(C[:, axn] - (np.round((at - h / 2) / h) * h + h / 2)) <= h / 2 + 1e-6
        for row, (val, lim, nm) in enumerate(((vm, PETG, 'von Mises / 30 MPa (in the layers)'), (il, PETG_Z, 'interlayer / 15 MPa'))):
            A = ax[row, col]
            sc = A.scatter(C[s, a], C[s, b], c=np.clip(val[s] / lim, 0, 0.4), s=(72 / 6 * h) ** 2 / 9, marker='s', cmap='inferno', vmin=0, vmax=0.4)
            A.set_aspect('equal'); A.set_title(f'{title}\n{nm} (1 / SF, scale to 0.4)', fontsize=11); A.set_xlabel(xl); A.set_ylabel(yl)
            A.set_facecolor((0.92, 0.93, 0.95))
    fig.colorbar(sc, ax=ax, shrink=0.6, label='stress / strength (0.33 = safety factor 3)')
    fig.suptitle(f'H01 mono holder FEA, {lab} (0.5 kg payload, dynamic), voxels {h} mm', fontsize=14)
    plt.savefig(OUT, dpi=80); plt.close(fig)


if __name__ == '__main__':
    a = [x for x in sys.argv[1:] if not x.startswith('--')]
    if '--v1' in sys.argv:                                          # the first fix (2026-11-25 morning): no fan, rib to z -30
        MH.FAN = None; MH.RIB = dict(MH.RIB, z=-30.0); OUT = OUT.replace('.png', '_v1.png')
    run(float(a[0]) if a else 2.0)
