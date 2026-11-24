# Mono holder (one-arm robot)

Holds the **one arm's base** (your printed parts **01 base_top + 02 base_bottom**, with the shoulder servo) and the
**scene camera** on the C-Beam XLarge's gantry plate. Printed in **PETG**, one piece. It replaces the shoulder ring
(`../mono_ring/`, now parked): the arm keeps its own shoulder servo (±115°).

![mono holder](preview_mono_holder.png)

Made by `mobile_manipulator_description/generator/mono_holder.py` (shape, checks, these files, the preview).

## Layout on the robot

- The base is **0.45 × 0.45 m**. The lift column stands **0.155 m in front of the base centre and 0.122 m to the right** of the centreline. It is
  turned so its gantry plate (the carriage) **faces the centre**: the arm's shoulder then sits on the centreline, level
  with the column. The carriage's front (plate, legs) ends at the bumper.
- The arm's zero pose points **left**, away from the column. Its ±115° covers the front, the left and the back; the
  blind sector faces the column.
- The **scene camera sits straight above the shoulder**, on a bar 10 mm over the arm's highest point, looking forward.
  Nothing sticks out in front of the carriage, so the robot can stand 0.255 m from furniture (2 cm past its bumper).
- The **tote stays on the base**, as in the original design, **L-shaped over the whole deck**: behind the arm
  (x −0.218 … 0.065 m, full width) plus the free left side up to the front edge (x 0.065 … 0.218 m, y 0.06 … 0.218 m),
  walls 0.12 m. Its floor is flat where the hand reaches pointing down (plus 50 mm of room for the hand) and rises at
  30° from there to the walls, so products that slide into a corner come back to where the arm gets them. The lift
  lowers the hand into it. The shape is in `tote/` (`tote_L_sloped.stl`, `tote.csv`); the base team builds it (sheet +
  sloped inserts), it is not a print.
- Nothing stands over the beam above the plate's top, so the lift keeps its full travel (0–0.871 m).

## What it is (heights above the gantry plate's top)

| Part | Where | What it does |
|---|---|---|
| **pad** | under the arm base, 6.5 mm below to 5.5 mm above the plate top (12 mm thick) | the base bolts down with 4 × M5 × 20 from below into its own M5 × 8 inserts |
| **legs, lip, rear bridge** | behind the gantry plate, as on the dual holder | bolt through 4 of the plate's own 5.2 mm holes (50 mm each side of the centre, 12.5 and 72.5 mm below its top), nylocs on the plate's front face; the lip rests on the plate's top edge |
| **post** | on the rear leg, behind the beam and its wheels, from 1 mm over the plate / wheel tops up to the bar (34 × 29 mm) | carries the bar; it stands where the shoulder's ±115° never takes the arm |
| **bar** | 145–167 mm over the plate top (the arm never rises above 135 mm), from the post along behind the arm, then forward over the shoulder | a 2 mm pocket for the camera mast's flange, 4 × M3 × 8 inserts in its floor |

## Parts

| # | File | What | Material | Print |
|---|---|---|---|---|
| H01 | `H01_holder.stl` | the whole holder, 152 × 247 × 150 mm, 453 cm³ | **PETG** | on its back (the rear leg and post face on the bed); supports under the bar and the pad |

The camera mast for this robot is `../scene_cam/C02_mast_98_mono.stl` (98 mm; the head and cradle are the same as the
two-arm robot's). `printed_parts.csv` has the size and volume; `fasteners.csv` every screw. `assembly_stl/` has the
holder in place plus `context/` (the arm base, the camera mast, the kit's plate / wheels / beam).

## Fasteners

| What | Qty | Where |
|---|---|---|
| M5 × 20 socket head | 4 | arm base (01 + 02) to the pad, from below into the base's M5 × 8 inserts |
| M5 × 25 socket head + nyloc | 4 | legs through the gantry plate's 5.2 mm holes |
| M3 × 8 heat-set insert | 4 | bar pocket floor, for the camera mast flange |

## Assembly

1. Press the 4 M3 inserts into the bar's pocket floor.
2. Bolt the arm base onto the pad: 4 × M5 × 20 from below.
3. Hang the holder on the gantry plate (lip on the plate's top edge) and bolt the legs through the plate: 4 × M5 × 25,
   nylocs on the plate's front face.
4. Set the camera mast's flange in the bar's pocket and screw it down (see `../scene_cam/`).

## Checked (no simulator)

- No clashes with the kit (gantry plate, V-wheels, beam with 1 mm clearance, top end mount, column foot), the arm base,
  the camera mast or the camera's pitch sweep; one piece.
- The mast flange sits on 1683 of its 1760 mm² on the pocket floor.
- The base tote is clear of the holder at the lowest lift (holder bottom 340 mm above the floor; the tote ends 15 mm
  before the rear legs, and its left leg is on the other side of the arm from the column).
- In the robot model: the URDF checks are clean, the travel fold is clear from lift 0 to 0.871 m (numbers and Isaac
  results in `mobile_manipulator_description/README.md`).

Not checked: strength and stiffness. The bar holds the mast, head, pitch servo and camera about 100 mm out from the post;
if the camera image shakes when the lift stops, a second post or a gusset under the bar is the fix.
