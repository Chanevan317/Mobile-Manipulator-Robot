"""Export the leader arm (v2): print-ready STLs (one per part, laid flat), assembly STLs (parts + servos), parts list."""
import sys, os, csv
sys.path.insert(0, '.')
import numpy as np
import leader as LD
from lib import xf, Rx, Ry, mesh_arrays
from stlio import write_stl
OUT = sys.argv[1] if len(sys.argv) > 1 else '../../leader_arm_CAD'
for d in ('print_ready_stl', 'assembly_stl'):
    os.makedirs(os.path.join(OUT, d), exist_ok=True)
    for f in os.listdir(os.path.join(OUT, d)):
        if f.endswith('.stl'): os.remove(os.path.join(OUT, d, f))
# print orientations: the flattest face on the bed (supports: link1's top prong, the clamp pockets)
PRINT_ROT = {'stand_top': Rx(180), 'link2': Rx(180), 'wrist': Rx(180), 'pitch_fork': Rx(-90), 'grip': Rx(90),
             'trigger': Rx(-90)}
LD.build()
rows = []; i = 0
merged = {nm: [nm] for nm in LD.PARTS}
F = LD.frames()
for key, names in merged.items():
    i += 1; num = f'{i:02d}_{key}'
    p = LD.PARTS[names[0]]
    s = xf(p['solid'], PRINT_ROT.get(names[0], np.eye(4)))
    bb = s.bounding_box(); s = s.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2])); bb = s.bounding_box()
    V, Fc = mesh_arrays(s); write_stl(os.path.join(OUT, 'print_ready_stl', num + '.stl'), V, Fc)
    for nm in names:
        V, Fc = mesh_arrays(xf(LD.PARTS[nm]['solid'], F[LD.PARTS[nm]['frame']])); write_stl(os.path.join(OUT, 'assembly_stl', f'{i:02d}_{nm}.stl'), V, Fc)
    rows.append(dict(file=num + '.stl', part=key, qty_per_leader=len(names), qty_pair=2 * len(names),
                     size_mm=f'{bb[3]-bb[0]:.1f} x {bb[4]-bb[1]:.1f} x {bb[5]-bb[2]:.1f}', volume_cm3=round(p['solid'].volume() / 1000, 1), note=p['note']))
for nm, body, M in LD.SERVOS:                                          # servos, for the assembly view
    V, Fc = mesh_arrays(xf(LD.SERVO, F[body] @ M)); write_stl(os.path.join(OUT, 'assembly_stl', f'servo_{nm}.stl'), V, Fc)
with open(os.path.join(OUT, 'printed_parts.csv'), 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
for r in rows: print(r['file'], r['qty_per_leader'], r['size_mm'], r['volume_cm3'])
import collections
fq = collections.OrderedDict()
for kind, size, length, qty, note in LD.FASTENERS:
    k = (kind, size, length); fq.setdefault(k, [0, []]); fq[k][0] += qty; fq[k][1].append(note)
with open(os.path.join(OUT, 'fasteners.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['type', 'size', 'length_mm', 'qty_per_leader', 'used for'])
    for (kind, size, length), (qty, notes) in fq.items():
        w.writerow([kind, size, length or '', qty, '; '.join(notes)]); print(kind, size, length, qty)
