# Dual-arm holder

Holds **both arm bases** (your printed parts **01 base_top + 02 base_bottom**, with their shoulder servos) on the C-Beam
XLarge's gantry plate, for the two-arm robot. Printed in **PETG**, one frame around the beam, printed as two halves.
It replaces the two aluminium brackets the robot model used to guess at.

![dual holder](preview_dual_holder.png)

Made by `mobile_manipulator_description/generator/dual_holder.py` (shape, checks, these files, the preview);
`dual_holder_strength.py` checks its strength with the loads from the Isaac load test (`scripts/robot_tests/isaac_dual_load_test.py`).

## What it is (heights above the gantry plate's top)

| Part of the frame | Where | What it does |
|---|---|---|
| **2 pads** | under each arm base, from 6.5 mm below to 5.5 mm above the plate top (12 mm thick) | each base bolts down with 4 × M5 × 20 from below into its own M5 × 8 inserts; the bases sit 150 mm each side, turned ±90° (±135° until 2026-11-25); each pad's inner end (under the base's pillar, which points at the column) is a 5 mm skin, 0.5 mm over the gantry plate and V-wheel tops |
| **front bridge** | in front of the beam (1 mm clear), 24 mm deep | joins the two pads; carries the **scene camera mast** (`../scene_cam/`) in a 2 mm pocket across its top, on 4 × M3 × 8 inserts in the pocket floor |
| **cheeks** | beside the gantry plate and its wheels | join pads, bridges and legs |
| **legs** | behind the gantry plate | bolt through 4 of the plate's own 5.2 mm holes (50 mm each side of the centre, 12.5 and 72.5 mm below its top), nylocs on the plate's front face; a lip rests on the plate's top edge |
| **rear bridge** | behind the plate | closes the frame around the beam |

Nothing stands over the beam above the plate's top, so the lift keeps its full travel (0.871 m).
The scene camera stands on the front bridge (OAK-D Pro on a pitch servo, `../scene_cam/`). Its mast flange sits in a
2 mm deep pocket across the bridge top (the pocket's side walls locate it), so the flange's screw heads stay low under
the arms. The flange covers the bridge top, so the front lap joint's screws come from below into inserts.

## Parts

| # | File | What | Material | Print |
|---|---|---|---|---|
| D01 | `D01_holder_left.stl` | left half (the side of the left arm): pad, cheek, leg, upper layer of both lap joints, scene camera mast inserts | **PETG** | on its outer side face (the cheek down); supports under the pad |
| D02 | `D02_holder_right.stl` | right half: pad, cheek, leg, lower layer of both lap joints | **PETG** | the same, mirrored |

`printed_parts.csv` has sizes and volumes; `fasteners.csv` every screw and nut. `assembly_stl/` has both halves in
place plus `context/` (the arm bases, the camera mast stand-in, the kit's plate / wheels / beam).

## Assembly

1. Bolt each arm base onto its half: 4 × M5 × 20 from below into the base's M5 × 8 inserts.
2. Put the two halves together around the beam (front bridge in front of it, rear bridge behind the plate) and bolt the
   lap joints: front bridge 4 × M5 × 16 from below into M5 × 6 inserts (heat-set into the left half's upper layer, from
   the lap face, before assembly; × 6, not × 8: the pocket above leaves room for 6 mm), rear bridge 2 × M5 + nylocs.
3. Rest the lips on the gantry plate's top edge and bolt the legs through the plate: 4 × M5 × 25 + nylocs on the front
   face of the plate (between the wheels).
4. Scene camera mast into the pocket on the front bridge: 4 × M3 × 10 into its inserts (see `../scene_cam/README.md`).

## Checked

`generator/dual_holder.py`:
- No contact with the kit's gantry plate, V-wheels, the beam (1 mm clear) or the column foot at the lowest lift; nothing
  inside the top end mount's footprint above the plate top.
- The arm bases sit on the pads without overlap; each half is one solid and the halves don't overlap.
- Arm sweep (left arm, 396 poses; the holder is symmetric; mounts ±90°): 77 poses touch the holder or the camera post.
  50 of them also hit the beam, gantry plate, V-wheels, top end mount or the other arm base (impossible anyway). The
  other 27 are the hand level and curled back under its own arm into the pad under its base: the pad turns with the
  base, so these are the same arm poses as at ±135°, and no plan uses them. The holder blocks no usable pose.

**Strength** (`generator/dual_holder_strength.py`, loads from `isaac_dual_load_test.py` with 0.25 / 0.5 kg in each
gripper: both arms stretched out sideways, to the front limit, back over the tote and back-inward; both shoulders
swinging at full speed then stopping hard; the lift going up and down; worst per arm base: tipping 8.3 N·m, down 32 N).
Printed strengths as for the ring (PETG 30 MPa, already reduced for walls / infill); target safety factor 3.
Recomputed 2026-11-25 on the ±90° holder with the 0.5 kg loads (measured at ±135°; the force sizes barely depend on the mount angle):

| Part | What | Stress / force | Safety factor |
|---|---|---|---|
| holder (PETG) | pad + bridge leaving the cheek (1519 mm² section, worst: the lift stopping at the bottom) | 1.8 MPa | 16.9 |
| legs | M5 bolt through the gantry plate | 65 N | 76 |
| legs (PETG) | bolt head pulling through a 12 mm leg | 0.2 MPa | 88 |
| lap joint | M5 shear, one arm loaded and the other not | 439 N | 6.8 |
| lap joint (PETG) | bolt bearing on an 11 mm lap layer | 8.0 MPa | 3.8 |
| arm base | its own M5 × 8 inserts | 22 N | 54 |

The lap joint was the weak spot with an 8 mm layer (safety factor 2.6), so the front bridge is 24 mm deep. The mast
pocket takes 2 mm off its top, so the lap face moved down 1 mm (11 mm layers, safety factor 3.0). That
figure is pessimistic: it puts one arm's whole moment on the closest bolt pair, while the plate bolts and the other
bolts share it. Insert values are typical published figures, not measured.
