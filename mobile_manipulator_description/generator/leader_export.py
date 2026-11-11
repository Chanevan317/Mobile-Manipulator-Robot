"""Export the leader arm (v3): print-ready STLs (arm module + both bases), assembly STLs (mono and proto, with the
servos), parts list, fasteners."""
import sys, os, csv, collections
sys.path.insert(0, '.')
import numpy as np
import leader as LD
from lib import xf, Rx, Ry, mesh_arrays
from stlio import write_stl
OUT = sys.argv[1] if len(sys.argv) > 1 else '../../leader_arm_CAD'
for d in ('print_ready_stl', 'assembly_stl', 'assembly_stl/mono', 'assembly_stl/proto'):
    os.makedirs(os.path.join(OUT, d), exist_ok=True)
    for f in os.listdir(os.path.join(OUT, d)):
        if f.endswith('.stl'): os.remove(os.path.join(OUT, d, f))
# print orientations: links / wrist / handle on their side (profiles in the XZ plane), moving jaw upside down
PRINT_ROT = {'link1': Rx(90), 'link2': Rx(90), 'wrist': Rx(90), 'forearm_handle': Rx(90), 'pitch_fork': Rx(-90),
             'gripper_jaw': Rx(180)}
LD.build()
rows = []; i = 0
for nm, p in list(LD.PARTS.items()) + list(LD.BASE_PARTS.items()):
    i += 1; num = f'{i:02d}_{nm}'
    s = xf(p['solid'], PRINT_ROT.get(nm, np.eye(4)))
    bb = s.bounding_box(); s = s.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2])); bb = s.bounding_box()
    V, Fc = mesh_arrays(s); write_stl(os.path.join(OUT, 'print_ready_stl', num + '.stl'), V, Fc)
    arm = nm in LD.PARTS
    rows.append(dict(file=num + '.stl', part=nm, group='arm module' if arm else 'base',
                     qty_mono=1 if arm else p['qty_mono'], qty_proto=2 if arm else p['qty_proto'],
                     size_mm=f'{bb[3]-bb[0]:.1f} x {bb[4]-bb[1]:.1f} x {bb[5]-bb[2]:.1f}',
                     volume_cm3=round(p['solid'].volume() / 1000, 1), note=p['note']))
for variant in ('mono', 'proto'):                                       # assemblies (zero pose), incl. the servos
    for nm, s, c, body in LD.world((0, 0, 0, 0, 0), variant=variant):
        V, Fc = mesh_arrays(s); write_stl(os.path.join(OUT, 'assembly_stl', variant, nm + '.stl'), V, Fc)
with open(os.path.join(OUT, 'printed_parts.csv'), 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
for r in rows: print(r['file'], r['qty_mono'], r['qty_proto'], r['size_mm'], r['volume_cm3'])
fq = collections.OrderedDict()
for kind, size, length, qty, note in LD.FASTENERS:
    k = (kind, size, length); fq.setdefault(k, [0, []]); fq[k][0] += qty; fq[k][1].append(note)
with open(os.path.join(OUT, 'fasteners.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['type', 'size', 'length_mm', 'qty_per_arm_module', 'used for'])
    for (kind, size, length), (qty, notes) in fq.items():
        w.writerow([kind, size, length or '', qty, '; '.join(notes)]); print(kind, size, length, qty)
