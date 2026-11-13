"""Real OpenBuilds C-Beam XLarge actuator (1000 mm) for the URDF, from the kit's STEP file.

Run once with FreeCAD (not needed for urdf_build.py; its output is committed in generator/cbeam/):
    freecadcmd -c "import sys; sys.argv=['x', '<path>/C Beam X Large Linear Actuator.step']; exec(open('cbeam_import.py').read())"

The STEP is the 500 mm version. Every part is used as it is, except:
  - beam and lead screw: stretched by 500 mm in the middle (their cross-section is the same along the length),
    top end mount + its bearing / shim / collar / bolts moved up 500 mm;
  - the 4 V-wheels: the STEP has them lying loose; they are put on their bearings (eccentric / 6 mm spacers).
Output (mm), LIFT frame: origin on the beam axis at its bottom end (the bottom end mount's inner face), z up the beam,
x toward the beam's closed face (robot front), y to the left; the open 80 mm face (and the gantry plate) at x -20.
  cbeam/<group>.stl + cbeam/cbeam.json (mass, density, part list per group). Carriage groups are in the PLATE frame:
  the same axes, origin at the plate's centre height (z), on the beam axis (x, y).
"""
import sys, json, os
import FreeCAD, Import, Mesh, MeshPart
STEP = sys.argv[-1]
HERE = os.path.dirname(os.path.abspath(__file__)) if '__file__' in dir() else os.getcwd()
OUT = os.path.join(HERE, 'cbeam'); os.makedirs(OUT, exist_ok=True)
doc = FreeCAD.newDocument('cb'); Import.insert(STEP, 'cb')
objs = {o.Label: o for o in doc.Objects if o.TypeId == 'Part::Feature' and o.Shape.Solids}
beam = objs['SOLID'].Shape.BoundBox
XC, ZC, Y0, YT = (beam.XMin + beam.XMax) / 2, (beam.ZMin + beam.ZMax) / 2, beam.YMin, beam.YMax
STRETCH = 500.0; YCUT = (Y0 + YT) / 2
plate = objs['125.125.6 v3'].Shape.BoundBox; PY = (plate.YMin + plate.YMax) / 2
print('beam', beam, 'plate', plate)

ALU, STEEL, POM = 2.70e-3, 7.85e-3, 1.41e-3            # g / mm^3
def dens(lb):
    if lb.startswith(('C-Beam End Mount', '40mm spacer', 'coupler', '125.125.6', '6mm Spacer', '3mm spacer', 'SOLID')): return ALU
    if lb.startswith(('Plates Block', 'Solid V-Wheel')): return POM
    return STEEL

def tess(shape):
    m = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.05, AngularDeflection=0.35, Relative=False)
    return [[tuple(p) for p in f.Points] for f in m.Facets]

def to_lift(p, dy=0.0, ref=None):
    x, y, z = p
    if ref is None: ref = Y0
    return (-(z - ZC), -(x - XC), y + dy - ref)

def stretched(tris):
    return [[(x, y + (STRETCH if y > YCUT else 0.0), z) for (x, y, z) in t] for t in tris]

groups = {}
def add(group, label, tris, mass=None):
    g = groups.setdefault(group, dict(tris=[], mass=0.0, parts=[]))
    g['tris'] += tris
    g['mass'] += mass if mass is not None else 0.0
    g['parts'].append(label)

TOP = {'C-Beam End Mount_2pcs v003', 'Bearing 625 2RS_2pcs v1', 'M5 Shim v1', 'C beam linear actuator v002'} | \
      {f'M5x20 LP_8pcs v00{i}' if i > 1 else 'M5x20 LP_8pcs v1' for i in range(1, 5)}
CARR_PREFIX = ('125.125.6', 'Plates Block', '3mm spacer', 'M5x25', 'M5 Nylock', 'Eccentric', '6mm Spacer', '1mm 5mm shim')
CARR_BEARINGS = {f'Bearing 625 2RS_2pcs v{i:03d}' for i in range(3, 11)}
wheel_src = None
for lb, o in objs.items():
    sh = o.Shape; m = sh.Volume * dens(lb) / 1000.0              # kg
    if lb.startswith('Solid V-Wheel'):
        if wheel_src is None: wheel_src = (lb, sh, m)
        continue
    if lb == 'SOLID':
        add('beam', lb, [[to_lift(p) for p in t] for t in stretched(tess(sh))], m * (YT - Y0 + STRETCH) / (YT - Y0)); continue
    if lb.startswith('Shaft'):
        L0 = sh.BoundBox.YLength
        add('column_hw', lb, [[to_lift(p) for p in t] for t in stretched(tess(sh))], m * (L0 + STRETCH) / L0); continue
    if lb.startswith('Nema 23'):
        add('motor', lb, [[to_lift(p) for p in t] for t in tess(sh)], 0.70); continue   # 56 mm NEMA 23: ~0.7 kg (catalogue), not solid metal
    if lb in TOP:
        add('top', lb, [[to_lift(p, STRETCH) for p in t] for t in tess(sh)], m); continue
    if lb.startswith(CARR_PREFIX) or lb in CARR_BEARINGS:
        add('carriage', lb, [[to_lift(p, 0.0, PY) for p in t] for t in tess(sh)], m); continue
    add('column_hw', lb, [[to_lift(p) for p in t] for t in tess(sh)], m)

# V-wheels on their bearing pairs (bearing axis = STEP z; wheel axis in the STEP part = its y)
lb, sh, m = wheel_src
c = sh.BoundBox.Center
bpos = {}
for i in range(3, 11):
    b = objs[f'Bearing 625 2RS_2pcs v{i:03d}'].Shape.BoundBox
    bpos.setdefault((round(b.Center.x, 1), round(b.Center.y, 1)), []).append(b)
wt = tess(sh)
for (bx, by), bs in sorted(bpos.items()):
    zc = (min(b.ZMin for b in bs) + max(b.ZMax for b in bs)) / 2
    # rotate about the wheel centre: (x, y, z) -> (x, z, y) (wheel axis y -> z; a reflection, so flip the winding)
    tris = [[(bx + (x - c.x), by + (z - c.z), zc + (y - c.y)) for (x, y, z) in reversed(t)] for t in wt]
    add('carriage', f'{lb} on bearing ({bx:.1f}, {by:.1f})', [[to_lift(p, 0.0, PY) for p in t] for t in tris], m)

meta = {}
for g, d in groups.items():
    mesh = Mesh.Mesh(d['tris']); mesh.write(os.path.join(OUT, g + '.stl'))
    bb = mesh.BoundBox
    meta[g] = dict(mass=round(d['mass'], 4), n_tris=mesh.CountFacets, parts=sorted(d['parts']),
                   bbox=[round(v, 2) for v in (bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax)])
    print(g, meta[g]['mass'], 'kg', mesh.CountFacets, 'tris', meta[g]['bbox'])
meta['_frame'] = dict(beam_len=round(YT - Y0 + STRETCH, 2), plate_back_x=round(-(plate.ZMax - ZC), 2),
                      plate_front_x=round(-(plate.ZMin - ZC), 2), plate_h=round(plate.YLength, 2),
                      source=os.path.basename(STEP), note='lift frame in mm, see cbeam_import.py')
json.dump(meta, open(os.path.join(OUT, 'cbeam.json'), 'w'), indent=1)
