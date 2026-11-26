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
| **web** | the rear bridge carried down between the legs to their bottom (9 mm thick, 105 mm tall) | carries the arm's tipping moment from the pad to the legs (the bridge alone was too weak, see Strength); two round slots let the inner arm-base screws seat and be driven from below |
| **rib** | under the pad on the centreline, 6 mm thick, from 73 mm deep at the web to nothing 60 mm out | stiffens the pad (and carries a third of the load) |
| **corner gussets** | where the pad meets the bridge, the pad widens in plan from ±24.5 mm (16 mm out) to ±42 mm | spread the load into the whole bridge instead of two corners; any wider and the folded arm's hand hits them |
| **post** | on the rear leg, behind the beam and its wheels, from 1 mm over the plate / wheel tops up to the bar (34 × 29 mm) | carries the bar; it stands where the shoulder's ±115° never takes the arm |
| **bar** | 145–167 mm over the plate top (the arm never rises above 135 mm), from the post along behind the arm, then forward over the shoulder | a 2 mm pocket for the camera mast's flange, 4 × M3 × 8 inserts in its floor |

## Parts

| # | File | What | Material | Print |
|---|---|---|---|---|
| H01 | `H01_holder.stl` | the whole holder, 520 cm³ | **PETG** | on its back (the rear leg and post face on the bed); supports under the bar and the pad |

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
2. Bolt the arm base onto the pad: 4 × M5 × 20 from below (the two screws next to the bridge go in through the round
   slots in the web).
3. Hang the holder on the gantry plate (lip on the plate's top edge) and bolt the legs through the plate: 4 × M5 × 25,
   nylocs on the plate's front face.
4. Set the camera mast's flange in the bar's pocket and screw it down (see `../scene_cam/`).

## Checked (no simulator)

- No clashes with the kit (gantry plate, V-wheels, beam with 1 mm clearance, top end mount, column foot), the arm base,
  the camera mast or the camera's pitch sweep; one piece.
- The web, rib and corner gussets don't block any arm pose. A 3025-pose sweep (URDF meshes, 3 mm margin) flagged a few
  poses near them; the exact CAD check shows those either already hit the old holder, gantry plate or beam, or don't
  touch the new parts at all.
- The mast flange sits on 1413 of its 1760 mm² on the pocket floor.
- The base tote is clear of the holder at the lowest lift (holder bottom 340 mm above the floor; the tote ends 15 mm
  before the rear legs, and its left leg is on the other side of the arm from the column).
- Isaac, 2026-11-25 (after the strength changes): all 14 pickable store products picked into the tote and put back
  1–6 mm from where they stood, nothing else knocked.
- In the robot model: the URDF checks are clean, the travel fold is clear from lift 0 to 0.871 m (numbers and Isaac
  results in `mobile_manipulator_description/README.md`).

## Strength

Two checks, no simulator. The arm's masses come from the URDF, with 0.5 kg in the gripper. Every case is run with gravity
alone, and with a dynamic case: gravity × 1.3 (the lift starting or stopping), 3 m/s² sideways (hard braking) and the
shoulder servo's full stall torque, all at once. PETG is taken as 30 MPa along the layers and 15 MPa across them (the
holder prints on its back, so the bridge and web are loaded across the layers). Target safety factor: 3.

**FEA** (`python3 mobile_manipulator_description/generator/mono_holder_fea.py`; needs `pyamg` for the fast solver):
- **Model:** the real holder cut into 2 mm cubes (270k unknowns), with the arm base as a stiff plate bolted on top. The
  legs are held where they bolt onto the gantry plate: the M5 preload is far above what the loads pull with.
- **Load cases:** the arm stretched straight out, to the front and to the back.
- **Output:** a stress map, `fea_mono_holder.png`.

Where the load goes: from the arm base into the pad, then down the rib (a third of it) and along the pad into the top of
the bridge. From there it goes into the upper halves of the legs, and through them into the gantry plate. The lower web
does little.

| Worst dynamic case | First fix: web + short rib | Now: + deep rib + corner gussets |
|---|---|---|
| Peak stress (one sharp inner corner), SF | 1.3 (pad corner into the bridge, across layers) | 3.5 (top of the bridge at the leg) |
| Every region at its 99th percentile, SF | ≥ 5.4 | ≥ 10 |
| Hand drop from the holder bending (arm stretched) | 3.1 mm (gravity), 4.0 mm (dynamic) | 1.6 mm, 2.1 mm |

**Beam check** (`mono_holder_strength.py`): 1210 arm poses on chosen sections.
- Bridge + web SF 5.4, legs 4.5 and the arm base inserts 4.7.
- The gantry bolts and the camera post / bar are above 90.
- The pad section under the arm base reads 3.3. That number is low because the beam method ignores the stiff arm base
  bolted on top of the pad. The FEA, which includes it, gives that area SF ≥ 10.

History (2026-11-25):
- **Original design:** the rear bridge (9 × 25.5 mm) was the only path from the pad to the legs. It had SF 1.0 with gravity
  alone and 0.7 dynamic, and the inner arm-base screws' heads hit it.
- **First fix:** the web, a short rib and the screw slots.
- **Then, from the FEA:** the rib was deepened, and the corner gussets were added where the narrow pad had pushed the load
  into the bridge at two points.
- **Rejected:** a wider fan, out to the legs' outer faces. It was stronger, but it blocked the folded arm.

1 kg in the gripper is more than the arm can hold (the wrist pitch servo gives up first). 0.5 kg is the design case.

**Crash limit:** a push on the stretched hand of about 75 N starts a crack at the sharp corner, and about 200 N breaks the
bulk of the holder. The lift's NEMA 23 and lead screw push far harder than that. If the hand lands on a shelf or the tote
rim, the lift must stop: use the lift driver's stall or following-error alarm, or a current limit.

Not checked: stiffness of the camera bar. It holds the mast, head, pitch servo and camera about 100 mm out from the post;
the stresses are tiny, but if the camera image shakes when the lift stops, a second post or a gusset under the bar is the fix.
