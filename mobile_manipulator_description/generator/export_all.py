import sys, os, csv, collections, math, json
sys.path.insert(0, '.')
from build import *
from export import write_glb, write_3mf, write_obj, write_step
from stlio import write_stl

OUT = sys.argv[1] if len(sys.argv) > 1 else 'out/SCARA_arm_CAD'
for d in ('print_ready_stl', 'assembly_stl', 'assembly_stl/purchased', 'step'):
    os.makedirs(os.path.join(OUT, d), exist_ok=True)

F = frames((0, 0, 0), 0, 0, 60.0)
build_parts(F)
B = bought(F)
FS = fasteners()

PRINT_ROT = {
    'base_top': np.eye(4), 'base_bottom': Rx(180), 'upper_arm_top': np.eye(4), 'upper_arm_bottom': Rx(180),
    'forearm_top': np.eye(4), 'forearm_bottom': Rx(180), 'wrist_top': np.eye(4), 'wrist_bottom': Rx(180),
    'pitch_housing_horn': Rx(-90), 'pitch_housing_idler': Rx(90), 'tilt_fork_horn': Rx(-90), 'tilt_fork_idler': Rx(90),
    'roll_yoke_gripper_housing': Ry(-90), 'gripper_front_plate': Ry(-90),
    'pinion_24T_m1': np.eye(4), 'finger_carrier_right': Ry(90), 'finger_carrier_left': Ry(90),
    'finger_right': Rx(-90), 'finger_left': Rx(90),
}

asm_items = []; print_rows = []
for i, (nm, p) in enumerate(arm.PARTS.items(), 1):
    num = f'{i:02d}_{nm}'
    sw = xf(p['solid'], p['frame'])
    V, Fc = mesh_arrays(sw)
    write_stl(os.path.join(OUT, 'assembly_stl', num + '.stl'), V, Fc)
    asm_items.append(dict(name=num, V=V, F=Fc, color=p['color']))
    if nm == 'camera_cradle':
        from arm import cam_frame
        loc = xf(xf(p['solid'], np.linalg.inv(cam_frame())), Rx(90))       # on its side wall (y = -20.4) on the bed
    else:
        loc = p['solid']
    s = xf(loc, PRINT_ROT.get(nm, np.eye(4)))
    bb = s.bounding_box()
    s = s.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2]))
    V2, F2 = mesh_arrays(s); bb = s.bounding_box()
    write_stl(os.path.join(OUT, 'print_ready_stl', num + '.stl'), V2, F2)
    print_rows.append(dict(file=num + '.stl', part=nm, qty_per_arm=1, qty_two_arms=2,
                           size_mm=f'{bb[3]-bb[0]:.1f} x {bb[4]-bb[1]:.1f} x {bb[5]-bb[2]:.1f}',
                           volume_cm3=round(p['solid'].volume() / 1000, 1), note=p['note']))

# calibration plate (not part of the robot): clamp on a table edge for calibration / bench tests
from test_stand import stand
_cp = stand(); _num = f'{len(arm.PARTS) + 1:02d}_calibration_plate'
V, Fc = mesh_arrays(_cp); write_stl(os.path.join(OUT, 'assembly_stl', _num + '.stl'), V, Fc)
_bb = _cp.bounding_box(); _cpp = _cp.translate((-(_bb[0] + _bb[3]) / 2, -(_bb[1] + _bb[4]) / 2, -_bb[2]))
V2, F2 = mesh_arrays(_cpp); write_stl(os.path.join(OUT, 'print_ready_stl', _num + '.stl'), V2, F2)
print_rows.append(dict(file=_num + '.stl', part='calibration_plate', qty_per_arm=1, qty_two_arms=1,
                       size_mm=f'{_bb[3]-_bb[0]:.1f} x {_bb[4]-_bb[1]:.1f} x {_bb[5]-_bb[2]:.1f}', volume_cm3=round(_cp.volume() / 1000, 1),
                       note='optional: bench / calibration plate, C-clamp on a table edge; 4 x M5 x 12 from below into the base inserts'))
# heat-set insert test block (not part of the robot): print first to pick the insert hole size
from insert_test import block as _insert_block
_ib = _insert_block(); _num = f'{len(arm.PARTS) + 2:02d}_insert_test_block'
_bb = _ib.bounding_box(); _ib = _ib.translate((-(_bb[0] + _bb[3]) / 2, -(_bb[1] + _bb[4]) / 2, -_bb[2]))
V2, F2 = mesh_arrays(_ib); write_stl(os.path.join(OUT, 'print_ready_stl', _num + '.stl'), V2, F2)
print_rows.append(dict(file=_num + '.stl', part='insert_test_block', qty_per_arm=1, qty_two_arms=1,
                       size_mm=f'{_bb[3]-_bb[0]:.1f} x {_bb[4]-_bb[1]:.1f} x {_bb[5]-_bb[2]:.1f}', volume_cm3=round(_ib.volume() / 1000, 1),
                       note='print first: M3 holes 4.1-4.5 and M5 holes 6.2-6.6 (small end at the notch) to pick the insert hole size'))
pur_items = []
for b in B:
    V, Fc = mesh_arrays(b['solid'])
    fn = b['name'].split(' ')[0]
    write_stl(os.path.join(OUT, 'assembly_stl', 'purchased', fn + '.stl'), V, Fc)
    pur_items.append(dict(name=b['name'], V=V, F=Fc, color=b['color'], metal=b.get('metal', 0)))

# fasteners as solids, grouped by kind/size for the assembly files
groups = collections.defaultdict(list)
for f in FS:
    key = f"{f['kind']}_{f['size']}" + (f"x{f['length']}" if f['length'] else '')
    groups[key].append(fastener_solid(f))
fast_items = []
for key, sols in sorted(groups.items()):
    V, Fc = mesh_arrays(union(sols))
    col = (0.85, 0.65, 0.20) if key.startswith('insert') else (0.55, 0.57, 0.60) if key.startswith('nut') else (0.08, 0.08, 0.09)
    write_stl(os.path.join(OUT, 'assembly_stl', 'purchased', 'fasteners_' + key + '.stl'), V, Fc)
    fast_items.append(dict(name='fasteners_' + key, V=V, F=Fc, color=col, metal=0.6))

ALL = asm_items + pur_items + fast_items
write_glb(os.path.join(OUT, 'SCARA_arm_assembly.glb'), ALL)
write_3mf(os.path.join(OUT, 'SCARA_arm_assembly.3mf'), ALL)
write_obj(os.path.join(OUT, 'SCARA_arm_assembly.obj'), ALL)
write_step(os.path.join(OUT, 'step', 'SCARA_arm_printed_parts_assembly.step'), asm_items)
write_step(os.path.join(OUT, 'step', 'SCARA_arm_purchased_parts.step'), pur_items)

# fastener schedule
with open(os.path.join(OUT, 'fasteners_by_location.csv'), 'w', newline='') as fh:
    w = csv.writer(fh)
    w.writerow(['#', 'goes in part', 'type', 'size', 'length_mm', 'purpose',
                'x_part', 'y_part', 'z_part', 'dir_x', 'dir_y', 'dir_z', 'x_arm', 'y_arm', 'z_arm'])
    for i, f in enumerate(FS, 1):
        w.writerow([i, f['part'], f['kind'], f['size'], f['length'] or '', f['note']] +
                   [f'{v:.2f}' for v in f['p_local']] + [f'{v:.3f}' for v in f['d_local']] + [f'{v:.2f}' for v in f['p']])
cnt = collections.Counter((f['kind'], f['size'], f['length']) for f in FS)
with open(os.path.join(OUT, 'fastener_BOM.csv'), 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['type', 'size', 'length_mm', 'qty_per_arm', 'qty_two_arms', 'used for'])
    for (k, s, l), n in sorted(cnt.items(), key=lambda t: (t[0][0], t[0][1], t[0][2] or 0)):
        uses = sorted(set(f['note'] for f in FS if (f['kind'], f['size'], f['length']) == (k, s, l)))
        w.writerow([k, s, l or '', n, 2 * n, '; '.join(uses)])
with open(os.path.join(OUT, 'printed_parts.csv'), 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(print_rows[0].keys())); w.writeheader(); w.writerows(print_rows)
json.dump(dict(fasteners=len(FS), parts=len(arm.PARTS), purchased=len(B)), open(os.path.join(OUT, 'summary.json'), 'w'))
print('done', len(FS), 'fasteners')
for (k, s, l), n in sorted(cnt.items(), key=lambda t: (t[0][0], t[0][1], t[0][2] or 0)):
    print(k, s, l, n)
