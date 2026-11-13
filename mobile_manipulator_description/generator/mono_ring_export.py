"""Export of the DRAFT mono-arm shoulder ring (mono_ring.py): print STLs (print orientation), assembly STLs
(carriage frame, mm, with the bought bearings, the drive servo and the estimated lift parts), parts / fastener CSVs and a
preview image. The README in the output folder is written by hand."""
import os, csv, collections
import numpy as np
import mono_ring as R
from lib import xf, print_mesh, union, T, Rz, box, ring
from stlio import write_stl
from joint import SERVO


def _write(path, solid):
    V, F = print_mesh(solid); write_stl(path, V, F)


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
    render(os.path.join(out, 'preview_mono_ring.png'))
    print(f'exported {len(rows)} parts to {out}')


def render(path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    COLLAR_C = (0.10, 0.62, 0.55)                                               # the arm's collar: teal, to see it
    reg = ring(R.COLLAR['ri'] - 0.2, R.COLLAR['ro'] + 0.2, R.COLLAR['z0'] - 0.2, R.AZ + 0.2)
    ab = R.PARTS['mono_upper_arm_bottom']
    items = [(p['solid'], p['color']) for k, p in R.PARTS.items() if k != 'mono_upper_arm_bottom']
    items += [(ab['solid'] - reg, ab['color']), (ab['solid'] ^ reg, COLLAR_C)]
    arm = ('mono_upper_arm_top',)
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
    # exploded: the arm (both halves + the forearm) lifted 70 mm off the rotor
    up = lambda it: (it[0].translate((0, 0, 70)), it[1])
    lifted = [(ab['solid'] - reg, ab['color']), (ab['solid'] ^ reg, COLLAR_C), (R.PARTS['mono_upper_arm_top']['solid'], R.PARTS['mono_upper_arm_top']['color']),
              (R.forearm_at(0, 60), (0.33, 0.56, 0.86))]
    rest = [(p['solid'], p['color']) for k, p in R.PARTS.items() if k not in ('mono_upper_arm_bottom', 'mono_upper_arm_top')]
    rest += [(R.balls(), (0.75, 0.77, 0.80)), (xf(SERVO, R.servo_frame()), (0.12, 0.12, 0.14)), (R.CONTEXT['servo_board'], (0.10, 0.35, 0.20))]
    fig = plt.figure(figsize=(24, 14), facecolor='white')
    draw(fig.add_subplot(2, 3, 1, projection='3d'), items + ctx, 25, -55, zoom=1.1,
         title='DRAFT mono shoulder ring on the lift carriage\n(teal: the arm\'s collar round the rotor; grey: lift parts, estimated)')
    draw(fig.add_subplot(2, 3, 2, projection='3d'), items + ctx, 12, -35, zoom=2.4, center=(45, 0, 45),
         title='the arm holds the ring with a COLLAR all the way round it\n(6 radial M3 into the rotor; the old lug + 4 screws are gone)')
    draw(fig.add_subplot(2, 3, 3, projection='3d'), [up(it) for it in lifted] + rest + ctx, 22, -50, zoom=1.0,
         title='exploded: the arm lifted off; its collar slides down over the rotor')
    draw(fig.add_subplot(2, 3, 4, projection='3d'), items + ctx, 0, -90, cut=box(-200, 200, 0, 200, -200, 300), zoom=1.6,
         center=(10, 0, 40), title='section (y = 0): hub = inner race, 2 ball rows, 3-ring rotor\n(outer race), top cap, the arm collar (teal) round the rotor')
    draw(fig.add_subplot(2, 3, 5, projection='3d'), items + ctx, 25, 125, zoom=1.1,
         title='from behind: L bracket to the gantry plate, drive servo + pinion')
    u = R.PIN_C
    draw(fig.add_subplot(2, 3, 6, projection='3d'), items + ctx, 15, 80, zoom=2.2, center=(u[0], u[1] + 10, 5),
         title='drive: STS3215 in the cradle (output up), 18 T pinion\non the 72 T rotor gear, slotted for backlash')
    plt.tight_layout(); plt.savefig(path, dpi=60); plt.close(fig)
