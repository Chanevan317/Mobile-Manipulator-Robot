"""Export of the DRAFT mono-arm shoulder ring (mono_ring.py): print STLs (print orientation), assembly STLs
(carriage frame, mm, with the bought bearings, the drive servo and the estimated lift parts), parts / fastener CSVs and a
preview image. The README in the output folder is written by hand."""
import os, csv, collections
import numpy as np
import mono_ring as R
from lib import xf, mesh_arrays, union, T, Rz, box
from stlio import write_stl
from joint import SERVO


def _write(path, solid):
    V, F = mesh_arrays(solid); write_stl(path, V, F)


def export(out):
    for d in ('print_ready_stl', 'assembly_stl', 'assembly_stl/context'):
        os.makedirs(os.path.join(out, d), exist_ok=True)
        for f in os.listdir(os.path.join(out, d)):                   # generated output only: start clean
            if f.endswith('.stl'): os.remove(os.path.join(out, d, f))
    rows, fast = [], collections.Counter()
    for i, (nm, p) in enumerate(R.PARTS.items(), 1):
        num = f'M{i:02d}_{nm}'
        _write(os.path.join(out, 'assembly_stl', num + '.stl'), p['solid'])
        s = xf(p['solid'], p['print_rot']); bb = s.bounding_box()
        s = s.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2])); bb = s.bounding_box()
        _write(os.path.join(out, 'print_ready_stl', num + '.stl'), s)
        rows.append(dict(file=num + '.stl', part=nm, qty=1, turns_with_arm='yes' if p['moving'] else 'no',
                         size_mm=f'{bb[3]-bb[0]:.1f} x {bb[4]-bb[1]:.1f} x {bb[5]-bb[2]:.1f}',
                         volume_cm3=round(p['solid'].volume() / 1000, 1), note=p['note']))
        for it in p['reg'].items:
            fast[(it['kind'], it['size'], it['length'], it['note'])] += 1
    _write(os.path.join(out, 'assembly_stl', 'context', 'balls_6mm.stl'), R.balls())
    _write(os.path.join(out, 'assembly_stl', 'context', 'drive_servo_STS3215.stl'), xf(SERVO, R.servo_frame()))
    _write(os.path.join(out, 'assembly_stl', 'context', 'servo_board_on_SO101_plate.stl'), R.CONTEXT['servo_board'])
    _write(os.path.join(out, 'assembly_stl', 'context', 'lift_ESTIMATE_gantry_wheels_beam.stl'),
           union([R.GANTRY, R.WHEELS, R.COLUMN ^ box(-30, 30, -50, 50, -110, 140)]))
    _write(os.path.join(out, 'assembly_stl', 'context', 'forearm_elbow_60deg.stl'), R.forearm_at(0, 60))
    with open(os.path.join(out, 'printed_parts.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    with open(os.path.join(out, 'fasteners.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['kind', 'size', 'length_mm', 'qty', 'where'])
        for (k, s, L, n), q in sorted(fast.items(), key=lambda t: (t[0][1], t[0][0], str(t[0][2]))):
            w.writerow([k, s, L, q, n])
    render(os.path.join(out, 'preview.png'))
    print(f'exported {len(rows)} parts to {out}')


def render(path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    items = [(p['solid'], p['color']) for p in R.PARTS.values()]
    items += [(R.balls(), (0.75, 0.77, 0.80)), (xf(SERVO, R.servo_frame()), (0.12, 0.12, 0.14)),
              (R.forearm_at(0, 60), (0.33, 0.56, 0.86)), (R.CONTEXT['servo_board'], (0.10, 0.35, 0.20))]
    ctx = [(R.GANTRY, (0.80, 0.80, 0.82)), (R.WHEELS, (0.35, 0.35, 0.38)), (R.COLUMN ^ box(-30, 30, -50, 50, -110, 58), (0.70, 0.72, 0.76))]

    def draw(ax, its, el, az, cut=None, zoom=1.0, center=None, title=''):
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
        lo, hi = T_.reshape(-1, 3).min(0), T_.reshape(-1, 3).max(0); c = (lo + hi) / 2 if center is None else np.array(center)
        r = (hi - lo).max() / 2 / zoom
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off(); ax.set_title(title, fontsize=13)
    fig = plt.figure(figsize=(20, 13), facecolor='white')
    draw(fig.add_subplot(2, 2, 1, projection='3d'), items + ctx, 25, -55, zoom=1.1,
         title='DRAFT mono shoulder ring on the lift carriage (grey: lift parts, estimated)')
    draw(fig.add_subplot(2, 2, 2, projection='3d'), items + ctx, 25, 125, zoom=1.1,
         title='from behind: L bracket to the gantry plate, drive servo + pinion (front left)')
    draw(fig.add_subplot(2, 2, 3, projection='3d'), items + ctx, 0, -90, cut=box(-200, 200, 0, 200, -200, 300), zoom=1.6,
         center=(10, 0, 40), title='section (y = 0): hub = inner race, 2 ball rows, 3-ring rotor (outer race), top cap, lug under the tube')
    u = R.PIN_C
    draw(fig.add_subplot(2, 2, 4, projection='3d'), items + ctx, 15, 80, zoom=2.2, center=(u[0], u[1] + 10, 5),
         title='drive: STS3215 in the cradle (output up), 18 T pinion on the 72 T rotor gear, slotted for backlash')
    plt.tight_layout(); plt.savefig(path, dpi=55); plt.close(fig)
