# Dual-arm holder

Holds **both arm bases** (your printed parts **01 base_top + 02 base_bottom**, with their shoulder servos) on the C-Beam
XLarge's gantry plate, for the two-arm robot. Printed in **PETG**, one frame around the beam, printed as two halves.
It replaces the two aluminium brackets the robot model used to guess at.

![dual holder](preview_dual_holder.png)

Made by `mobile_manipulator_description/generator/dual_holder.py` (shape, checks, these files, the preview);
`dual_holder_strength.py` checks its strength with the loads from the Isaac load test (`scripts/isaac_dual_load_test.py`).

## What it is (heights above the gantry plate's top)

| Part of the frame | Where | What it does |
|---|---|---|
| **2 pads** | under each arm base, from 6.5 mm below to 5.5 mm above the plate top (12 mm thick) | each base bolts down with 4 × M5 × 20 from below into its own M5 × 8 inserts; the bases sit exactly where they were (150 mm each side, turned ±135°) |
| **front bridge** | in front of the beam (1 mm clear), 24 mm deep | joins the two pads; carries the **scene camera's post** in a socket (as on the one-arm robot) |
| **cheeks** | beside the gantry plate and its wheels | join pads, bridges and legs |
| **legs** | behind the gantry plate | bolt through 4 of the plate's own 5.2 mm holes (50 mm each side of the centre, 12.5 and 72.5 mm below its top), nylocs on the plate's front face; a lip rests on the plate's top edge |
| **rear bridge** | behind the plate | closes the frame around the beam |

Nothing stands over the beam above the plate's top, so the lift keeps its full travel (0.871 m).
The scene camera's post moved here from behind the plate: the left leg bolts on where it used to stand. The camera
itself is where it was.

## Parts

| # | File | What | Material | Print |
|---|---|---|---|---|
| D01 | `D01_holder_left.stl` | left half (the side of the left arm): pad, cheek, leg, upper layer of both lap joints, camera post socket | **PETG** | on its outer side face (the cheek down); supports under the pad |
| D02 | `D02_holder_right.stl` | right half: pad, cheek, leg, lower layer of both lap joints | **PETG** | the same, mirrored |

`printed_parts.csv` has sizes and volumes; `fasteners.csv` every screw and nut. `assembly_stl/` has both halves in
place plus `context/` (the arm bases, the camera post, the kit's plate / wheels / beam).

## Assembly

1. Bolt each arm base onto its half: 4 × M5 × 20 from below into the base's M5 × 8 inserts.
2. Put the two halves together around the beam (front bridge in front of it, rear bridge behind the plate) and bolt the
   lap joints: 6 × M5 + nylocs (4 in the front bridge, 2 in the rear).
3. Rest the lips on the gantry plate's top edge and bolt the legs through the plate: 4 × M5 × 25 + nylocs on the front
   face of the plate (between the wheels).
4. Scene camera post into the socket on the front bridge; M3 set screw from the front.

## Checked

`generator/dual_holder.py`:
- No contact with the kit's gantry plate, V-wheels, the beam (1 mm clear) or the column foot at the lowest lift; nothing
  inside the top end mount's footprint above the plate top.
- The arm bases sit on the pads without overlap; each half is one solid and the halves don't overlap.
- Arm sweep (left arm, 396 poses; the holder is symmetric): the arm touches the holder or the camera post only in
  poses that were already blocked before (by the old brackets, the old camera post or the beam): the forearm or hand
  folded back toward the column. The holder blocks no pose that was free.

**Strength** (`generator/dual_holder_strength.py`, loads from `isaac_dual_load_test.py` with 0.25 / 0.5 kg in each
gripper: both arms stretched out sideways, to the front limit, back over the tote and back-inward; both shoulders
swinging at full speed then stopping hard; the lift going up and down; worst per arm base: tipping 8.3 N·m, down 32 N).
Printed strengths as for the ring (PETG 30 MPa, already reduced for walls / infill); target safety factor 3:

| Part | What | Stress / force | Safety factor |
|---|---|---|---|
| holder (PETG) | pad + bridge leaving the cheek (936 mm² section, worst: the lift stopping at the bottom) | 2.8 MPa | 10.6 |
| legs | M5 bolt through the gantry plate | 132 N | 38 |
| legs (PETG) | bolt head pulling through a 12 mm leg | 0.4 MPa | 44 |
| lap joint | M5 shear, one arm loaded and the other not | 465 N | 6.5 |
| lap joint (PETG) | bolt bearing on a 12 mm lap layer | 7.7 MPa | 3.9 |
| arm base | its own M5 × 8 inserts | 22 N | 54 |

The lap joint was the weak spot with an 8 mm layer (safety factor 2.6), so the front bridge is 24 mm deep. That
figure is pessimistic: it puts one arm's whole moment on the closest bolt pair, while the plate bolts and the other
bolts share it. Insert values are typical published figures, not measured.
