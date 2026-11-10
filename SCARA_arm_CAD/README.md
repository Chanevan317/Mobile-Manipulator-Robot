# SCARA arm, prototype: 3D files

One arm. For the two-arm robot, print and buy everything twice. Both arms use the same parts; nothing is mirrored.

**Printing?** Go to [Print settings](#print-settings-bambu-lab-a1-pla-pro) and print the fit test first.

## What's in the folder

| Path | What it is | Opens in |
|---|---|---|
| `print_ready_stl/` | One STL per printed part, laid flat and ready to slice | Any slicer |
| `assembly_stl/` | The same parts in their assembled positions (they line up when you import them all) | Any CAD program or viewer |
| `assembly_stl/purchased/` | Servos, 6808 bearings, MGN9 rail and blocks, camera envelope, and every screw, insert and nut, grouped by size | Any CAD program or viewer |
| `step/` | Printed and purchased parts as STEP solid bodies (faceted) | Fusion 360, SolidWorks, Onshape, FreeCAD, Inventor |
| `SCARA_arm_assembly.glb` | The whole arm with colours, including every fastener | Windows 3D Viewer, Blender, online glTF viewers |
| `SCARA_arm_assembly.3mf` / `.obj` | The same assembly | PrusaSlicer, Bambu Studio, Fusion, Blender |
| `fastener_BOM.csv` | How many of each screw, insert and nut, and what each is for | Excel, Sheets |
| `fasteners_by_location.csv` | Every fastener: which part it goes in, its exact position and direction (part frame and arm frame, in mm) | Excel, Sheets |
| `printed_parts.csv` | Printed parts with their print size and volume | Excel, Sheets |
| `../mobile_manipulator_description/generator/` | The Python generator. Change a number and rebuild. | Python 3 + manifold3d |

Units are millimetres. The files use the **arm frame**: Z is up, and the origin is on the shoulder axis at the shoulder servo's mid-height. X points along the arm at zero pose. The carriage mount plate's top face is at **Z = -40**.

## How accurate it is

**Servo casing.** The casing comes from Feetech's STS3215 CAD model (SO-ARM100 repo, Apache-2.0). Every servo pocket is cut from that real mesh, not from a box. Each pocket follows the slid-out direction, so a clamshell half lifts straight off.
- **Every servo is an STS3215, held exactly like on the SO-100 (SO-ARM100)**, so a servo moves between the SO-100 and this arm with its own screws. The socket was measured from the SO-100 leader print files.
- The pocket is line-to-line with the case, as on the SO-100: no clearance around the sides, the horn face flush, 0.1 mm play on the idler side.

Measured from the CAD model:

| Feature | Value |
|---|---|
| Case | 45.4 × 24.8 mm. Faces at ±14.4 mm, rims at ±15.9 mm |
| Shaft | 10.2 mm from the front end |
| Idler horn (cable-socket face, spins freely) | Ø20 mm, face at +17.7 mm, 4 × Ø2.5 tapped holes (M3) on a 9.9 mm square |
| Output horn (opposite face, driven) | Ø20 mm, face at -18.7 mm, 4 × Ø2.5 tapped holes (M3) on a 9.9 mm square, centre nub to -20.2 mm |
| Case screw holes, horn face | Ø1.5 mm (M2 self-tapping), at (-32.8, ±10.25) and (-8.3, ±10.25) mm |
| Case screw holes, idler face | Ø1.5 mm (M2 self-tapping), at (-29.0, ±10.25) and (-8.3, ±10.25) mm |
| Connectors | On the idler face at X = -14.05 mm |

**Screws.**
- Every screw hole is cut where its screw sits, at the length listed.
- An automatic check placed a model of each screw. Every screw sits only in its own clearance hole and thread. The only overlaps are the thread engagement in the servo horns and the MGN9H blocks.
- Every heat-set insert has at least 1 mm of plastic around it.
- Hole polygons are drawn around the nominal diameter, so a printed hole is never smaller than nominal.

**Fit.** At the assembled pose, an automatic check found no part touching another, apart from the intended contacts.

## Printed parts (21 per arm)

| # | File | Size on the bed (mm) | Volume (cm³) | What it is |
|---|---|---|---|---|
| 1 | `01_base_top.stl` | 121.0 x 72.0 x 35.0 | 154.5 | shoulder servo clamshell + pillar; prints split-face down |
| 2 | `02_base_bottom.stl` | 121.0 x 72.0 x 40.0 | 148.1 | shoulder servo clamshell + pillar; prints split-face down |
| 3 | `03_upper_arm_top.stl` | 204.0 x 76.0 x 49.6 | 182.2 | link 130 mm: hub/fork at the near joint, servo clamshell at the far joint; prints split-face down |
| 4 | `04_upper_arm_bottom.stl` | 204.0 x 76.0 x 33.6 | 154.2 | link 130 mm: hub/fork at the near joint, servo clamshell at the far joint; prints split-face down |
| 5 | `05_forearm_top.stl` | 204.0 x 76.0 x 49.6 | 182.2 | link 130 mm: hub/fork at the near joint, servo clamshell at the far joint; prints split-face down |
| 6 | `06_forearm_bottom.stl` | 204.0 x 76.0 x 33.6 | 154.2 | link 130 mm: hub/fork at the near joint, servo clamshell at the far joint; prints split-face down |
| 7 | `07_wrist_top.stl` | 104.0 x 76.0 x 49.6 | 64.5 | wrist-pan fork; the pitch housing hangs under the bottom prong |
| 8 | `08_wrist_bottom.stl` | 112.0 x 76.0 x 33.6 | 59.6 | wrist-pan fork; the pitch housing hangs under the bottom prong |
| 9 | `09_pitch_housing_horn.stl` | 72.0 x 77.0 x 35.0 | 105.1 | pitch-servo clamshell (STS3215, SO-100 socket); split plane is vertical |
| 10 | `10_pitch_housing_idler.stl` | 72.0 x 77.0 x 27.0 | 82.2 | pitch-servo clamshell (STS3215, SO-100 socket); split plane is vertical |
| 11 | `11_tilt_fork_horn.stl` | 140.0 x 156.0 x 49.6 | 100.5 | tilt fork half: hub/prong on the pitch servo + half of the roll-servo clamp; split plane is vertical |
| 12 | `12_tilt_fork_idler.stl` | 140.0 x 156.0 x 33.6 | 94.9 | tilt fork half: hub/prong on the pitch servo + half of the roll-servo clamp; split plane is vertical |
| 13 | `13_camera_cradle.stl` | 19.0 x 36.0 x 40.8 | 7.5 | holds the 32 x 32 OV camera board (4 x M2 on 28 x 28); tilts 0..60 deg on one M3 bolt |
| 14 | `14_roll_yoke_gripper_housing.stl` | 61.4 x 44.0 x 48.3 | 41.1 | bolts to the roll servo horn, holds the gripper servo; servo slides in from the front |
| 15 | `15_gripper_front_plate.stl` | 54.4 x 170.0 x 8.0 | 28.9 | closes the gripper housing and carries the 170 mm MGN9 rail |
| 16 | `16_pinion_24T_m1.stl` | 26.0 x 26.0 x 14.8 | 4.6 | 24 T module 1 spur pinion on the gripper servo horn |
| 17 | `17_finger_carrier_right.stl` | 62.2 x 93.5 x 17.5 | 14.8 | bolts to one MGN9H carriage; its rack meshes with the pinion |
| 18 | `18_finger_carrier_left.stl` | 62.0 x 93.5 x 17.5 | 14.8 | bolts to one MGN9H carriage; its rack meshes with the pinion |
| 19 | `19_finger_right.stl` | 50.0 x 40.0 x 14.0 | 17.1 | wedge finger, 1.5 mm rubber pad pocket on the grip face |
| 20 | `20_finger_left.stl` | 50.0 x 40.0 x 14.0 | 17.1 | wedge finger, 1.5 mm rubber pad pocket on the grip face |
| 21 | `21_calibration_plate.stl` | 150.0 x 150.0 x 10.0 | 223.0 | optional: bench / calibration plate, C-clamp on a table edge; 4 x M5 x 12 from below into the base inserts |

## Print settings (Bambu Lab A1, PLA Pro+)

Files: `print_ready_stl/`, already in the print orientation. Print every part **twice** (two arms).

### Step 1: fit test before everything else

| Print | Then check | Good result |
|---|---|---|
| `01_base_top` + `02_base_bottom` (same servo socket as every joint) | Servo in the pocket | Snug, the same feel as in your SO-100 parts |
| | 4 × M2×6 case screws | All four line up with the servo's case holes |
| | Horn centre screw | Stays in; the hub sits flat on the horn |
| | 6808 bearing in its seat (Ø52.15) | Light press |
| | Hub in the bearing (Ø39.9) | Slip fit |

If a fit is wrong, change `SO100` (servo), `P['seat']` or `P['hub']` in `generator/joint.py` and export again.

### Step 2: slicer settings (Bambu Studio)

Start from these presets (the A1 ones, **not** "@BBL P1P"), change the values below, and save your own process preset:

| Preset | Select |
|---|---|
| Printer | Bambu Lab A1 0.4 nozzle |
| Process | 0.20mm Standard @BBL A1 |
| Filament | Your brand's PLA+ profile, or Generic PLA @BBL A1 |

| Setting | Value | Where | Why |
|---|---|---|---|
| Nozzle | 0.4 mm | Printer | – |
| Layer height | 0.20 mm (0.12 mm for `17_pinion_24T_m1`) | Quality (pinion: per-object setting) | Gear teeth need the finer layers |
| Walls | 4 | Strength → Wall loops | The walls carry the load and hold the inserts |
| Top / bottom layers | 5 / 5 | Strength → Top/Bottom shell layers | Flat tops and pocket ceilings close properly over 15 % infill |
| Infill | 15 %, cubic | Strength → Sparse infill | Like the SO-100. Too flexible? Add walls, not infill |
| Supports | On, tree (auto), threshold 30° | Support | Hubs, link channels, cable slots |
| Supports on build plate only | **Off** | Support | A few small ceilings need support that starts on the part |
| Elephant-foot compensation | 0.15 mm | Quality → Precision | The pocket openings are on the bed face |
| X-Y hole / contour compensation | 0 / 0 | Quality → Precision | Keeps the Ø2.0 / Ø4.0 holes like on the SO-100 |
| Avoid crossing walls | On | Quality → Advanced | Less stringing |
| Nozzle temperature | From your temperature tower (Step 3) | Filament settings | The A1 does not set it automatically |

**Plate:** every part fits the 256 × 256 mm bed. The upper-arm halves are 254 mm long: **rotate them 45°**.

### Step 3: find the nozzle temperature (once per spool)

The A1 calibrates bed leveling, vibration and flow dynamics by itself, but **not** temperature.

| # | Do this | Detail |
|---|---|---|
| 1 | Bambu Studio → **Calibration → Temperature** | Your printer, plate and filament profile (your brand's PLA+, or "Generic PLA") |
| 2 | Range 230 → 200 °C, 5 °C steps | Print the tower (about 30–40 min) |
| 3 | Pick the best block | Fewest strings, clean bridges, smooth glossy walls. Two look equal? Take the hotter one: stronger layers |
| 4 | Save it | Filament settings → nozzle temperature (first layer and other layers) → save as your own preset |
| 5 | Optional: **Calibration → Flow rate** | Over-extrusion makes the servo pockets tight |
| 6 | Optional: **Calibration → Retraction** | Only if strings remain after steps 1–4 |

### Problems

| You see | Likely cause | Fix |
|---|---|---|
| Thin hair-like strings | Nozzle too hot, travel over open areas, wet filament | Temperature tower (Step 3), Avoid crossing walls on, dry the spool (50–55 °C, 4–6 h) |
| Servo too tight | Over-extrusion, elephant foot, support marks on the pocket ceiling | Flow-rate calibration, elephant-foot 0.15 mm, clean the ceiling |
| Servo loose | Under-extrusion | Flow-rate calibration |
| Strings in pockets or holes | – | Clear them before fitting servos and screws |

**Heat-set inserts (M3, M5):** see "Heat-set inserts" below (iron at about 200–220 °C for PLA).

## Purchased parts (per arm)

- **Servos:** 6 × STS3215 7.4 V (the SO-100 servo, taken from your SO-100; not in the purchase BOM): shoulder, elbow, wrist pan, wrist pitch, wrist roll and gripper.
  - Payload: the wrist pitch holds the hand and the item against gravity. With the wrist level, 0.25 kg in the gripper uses about 60 % of the STS3215's 1.9 N·m and 0.5 kg about 90 %. Plan for items up to about 0.25–0.3 kg.
- **6808-2RS bearings:** 8 (40 × 52 × 7). Two each on the shoulder, elbow, wrist-pan and pitch joints.
- **Linear rail:** 1 × MGN9 rail, 170 mm, with 2 × MGN9H blocks. Catalogue dimensions used: W 20, block-mounting holes 15 × 16, height 10.
- **Camera:** 1 × OV camera module, 32 × 32 mm board, 4 mounting holes on a 28 × 28 mm square (M2 self-tapping
  screws into the cradle's standoffs).
- **Fasteners:** see `fastener_BOM.csv`.
  - M3 socket-head screws: ISO 4762, steel 8.8 or A2.
  - M3 heat-set inserts: hole Ø4.0 × 6.5 deep. For example Ruthex M3×5.7 or CNC Kitchen M3 standard.
  - M5 heat-set inserts: hole Ø6.4 × 10.5 deep. These hold the arm on the carriage plate.
  - M2×6 self-tapping screws: the same ones the SO-100 uses for its servo cases.
  - M3 nuts: hold the rail; one M3 nyloc nut on the camera tilt bolt.

| Item | Per arm | Two arms | Used for |
|---|---|---|---|
| Socket-head cap screw M3×8 | 14 | 28 | camera post; carrier to MGN9H; pitch housing |
| Socket-head cap screw M3×10 | 32 | 64 | finger; front plate to housing; horn (output); pinion to gripper horn; roll horn |
| Socket-head cap screw M3×12 | 6 | 12 | MGN9 rail |
| Socket-head cap screw M3×20 | 4 | 8 | roll clamp |
| Socket-head cap screw M3×30 | 10 | 20 | clamshell |
| Socket-head cap screw M3×35 | 14 | 28 | clamshell; fork base |
| Socket-head cap screw M3×55 | 1 | 2 | camera tilt bolt (through both cheeks + the cradle tube, nyloc nut) |
| Socket-head cap screw M5×16 | 4 | 8 | from under the 8 mm mount plate |
| Heat-set insert M3 | 42 | 84 | camera bracket; clamshell; finger; fork base; gripper front plate; roll clamp; to wrist bracket |
| Heat-set insert M5 | 4 | 8 | mount plate |
| Hex nut M3 | 7 | 14 | MGN9 rail (nut trap); camera tilt bolt (nyloc) |
| Self-tapping screw M2×6 | 22 | 44 | gripper servo case (SO-100); roll servo case (SO-100); servo case (SO-100) |
| Self-tapping screw M2×12 | 4 | 8 | camera board (through the standoff) |

## Where the screws go

The full list is in `fasteners_by_location.csv`.

**Which servo face is which.** The face with the two **cable sockets** carries the **free-spinning idler**. The
opposite face is the **driven output**. Only the output face is screwed (design B). The top half of the next part
turns in the top 6808 bearing above the idler and is not screwed to it: the two bearings carry and centre the part,
the servo only turns it, and the cable can go up through the middle of the joint. (The SO-100 screws both faces
because it has no bearings.)

1. **Clamshell bolts.** The two halves of every servo clamshell are held by 4 × M3. On links they are M3×30, put in
   from the bottom. On the shoulder base they are M3×35, put in from the top, because the base sits on the mount plate:
   their heads sit 6 mm deep in the counterbores so the bolt fills the whole insert (the inserts are in the bottom
   half, pressed in from the split face).
   **Each bolt screws into a brass heat-set insert in the other half** (the larger Ø4.0 hole). Without the insert the
   bolt has nothing to grip. Fit the inserts first (see "Heat-set inserts" below).
2. **Horn screws (output face only).** The bottom half of the next part fixes to the output horn with 4 × M3×10 from
   below, on the 9.9 mm square. The top half is not screwed to the idler.
   - Keep each horn's own centre screw in: the bottom hubs, the roll yoke and the gripper pinion have a Ø7.2 recess
     with 3.5 mm of room past the centre boss.
   - The idler bar in each parent top half is anchored at both ends and prints as a 27 mm bridge (no support needed).
   - Print one top hub first and check it in a bearing: it should slide in snugly with no play (hub Ø39.9 in the
     40 mm bore). If it's loose, the hub can be made 0.1–0.2 mm bigger.
   - Set the servo to centre (2048) before fitting the horns.
3. **Case screws (same as the SO-100).** M2×6 self-tapping screws go through a 2.0 mm wall into the servo's case holes.
   The holes are Ø2.0 with a Ø4.0 head counterbore.
   - Shoulder, elbow, wrist pan and pitch: 2 through the socket-face half at (-32.8, ±10.25) and 2 through the
     output-face half at (-29, ±10.25), as on the SO-100 arm joints.
   - Wrist roll: 4 at (-32.8, ±10.25) and (-8.3, ±10.25) on the socket face, through the tilt-fork base. Fit them before
     the tilt fork goes onto the pitch servo.
   - Gripper: 2 at (-32.8, ±10.25) on the socket face, from behind through the yoke. To reach them, turn the roll 90° or 180°.
4. **Fork bases.** Each fork base gets 2 × M3×35 from below. The links have no bolts along the tube: the bolts at both
   ends hold the two halves.
5. **Mount plate.** 4 × M5×16 come up through the 8 mm carriage plate into **M5 heat-set inserts** in the bottom of the base.
   Press these 4 inserts in from underneath the base (the bottom face is flat, easy to reach with the iron).

**Roll and gripper servos face forward with their output.** Both are fitted with the cable-socket face toward the
back (the roll servo's sockets face the tilt-fork base, the gripper servo's face the yoke's back wall), so the
yoke and the pinion are driven by the output horn. Their cables leave through a channel behind the sockets: up out of
the top of the roll clamp, and out of the yoke's back wall.

## Wrist camera (tilt with one bolt)

The camera board screws to the cradle's 4 standoffs (M2 × 12 self-tapping, 28 × 28 mm pattern), lens forward. The
cradle sits between the two cheeks of the fork on the roll clamp, and **one M3 × 55 bolt** goes through both cheeks
and the tube on the back of the cradle, with a nyloc nut. Loosen the nut, tilt the camera, tighten: the cheeks clamp
the tube ends and hold the angle (like a GoPro mount).
- Tilt range 0° (looking straight ahead) to 60° down. At about **35° down** the TCP between the fingertips is in the
  middle of the image, and the view clears the gripper rail by 9 mm.
- If it slips, add a washer under the nut or a drop of rubber/PLA glue on the tube ends; don't overtighten (the cheeks
  flex).

## Calibration plate (bench tests)

`21_calibration_plate.stl` is not part of the robot. It's a 150 × 150 × 10 mm plate to clamp one arm on a table for
calibration and bench tests. Bolt the base on with 4 × **M5×12** from underneath (heads flush in the counterbores)
into the base's M5 inserts, the same holes the carriage bracket uses. Put the plate's front edge at the table edge:
the base's round joint end and the shoulder then stick out ~15 mm past the edge and the arm swings out over the floor
side. Clamp it with two C-clamps whose pads sit in the two **clamp pockets** (2 mm deep) beside the pillar, toward the
back: that's the only place near the table edge the upper arm never swings over (clear up to ±120°). The pockets
are 49–75 mm from the table edge, so use C-clamps with **at least 80 mm of throat depth**.

The wrist hangs about 7 cm below the base, so don't swing it back over the table: calibrate the shoulder over the
front half, and keep the elbow fairly open when the arm points sideways.

## Heat-set inserts

The clamshell halves, fork bases, roll clamp, gripper and camera bracket use **M3 brass heat-set inserts**, and the
base uses **4 × M5** for the carriage plate. Per arm: 42 × M3 and 4 × M5 (`fastener_BOM.csv`).

| Insert | Hole in the part | Fits |
|---|---|---|
| M3 | Ø4.0 × 6.5 mm deep | M3 × 5.7 mm long, ~4.6 mm outside (Ruthex / CNC Kitchen "M3 standard" type) |
| M5 | Ø6.4 × 10.5 mm deep | M5 × 9.5 mm long, ~7 mm outside |

If your inserts have a different outside diameter or length, tell me their size and I'll change the hole (`SPEC` in
`generator/hw.py`) and re-export.

Every insert was checked in CAD: at least 1.5 mm of plastic all round it, nothing over the opening (the iron can
reach it), and its screw fills the insert's thread without bottoming out in the hole.

How to fit one:
1. Soldering iron with a heat-set tip of the insert's size, at about **200–220 °C for PLA**.
2. Put the insert on the hole, small end down. Press it in straight with the hot tip, slowly, with light force.
   Stop when it is flush or 0.2 mm below the surface.
3. Pull the tip out straight and, while the plastic is still soft, press the insert flush with a flat metal object.
4. Let it cool for 10 s before you put any load on it.

## Control board (Waveshare bus servo adapter)

The board goes on the **SO-101 Waveshare mounting plate** (`../SO100/WaveShare_Mounting_Plate_SO101.stl`). Its 18 mm
square tapered peg **slides sideways into the dovetail socket on the back face of the base pillar** (bottom half, open
on one side), right under the rear cable slot. The cables come out of the slot and plug straight into the board. You
can take the board off without opening the base. The board screws to the plate as on the SO-101.
Slide the plate in with its **long side (51 mm) horizontal**: then it spans from 2 mm above the bottom of the base
to just above the split line, and stays clear of the carriage bracket. Turned 90° (the square peg allows it) it
would hang about 7.5 mm below the base onto the bracket. Its 5 mm overhang past each side of the pillar is fine:
nothing moves there.

## Assembly order (each pan joint)

1. Fit the heat-set inserts. Press the two 6808 bearings into their seats in the parent halves.
2. Put the servo in the parent's bottom half **with the cable-socket face up**. Fit the 2 M2 screws from below.
3. Plug in both cables. Lead the incoming cable back through the rear slot. Lead the outgoing cable (to the next servo)
   forward over the idler bar (the thin fixed bar across the joint above the idler) to the middle of the joint.
4. Close the top half, fit the 2 M2 screws from above (down the two holes behind the bearing), then the 4 clamshell
   bolts.
5. Pass the outgoing cable's plug up through the **Ø12 hole in the middle of the child's top hub**, then push the hub
   into the top bearing (no screws).
6. Push the child's bottom hub up through the bottom bearing onto the output horn. Fit its 4 horn screws from below.
7. Bolt the child halves together: fork base, then the next clamshell.

**Cables.** Every cable runs forward through the joints, never in a loop outside:
- **Board → shoulder servo:** out of the rear slot of the base pillar and down into the board on the pillar's back face.
- **Through each pan joint:** from the servo's socket over the **idler bar** (a 10 × 2 mm fixed bar across the
  parent's top half, under the cable hole, so the cable never touches the free-spinning idler or its screw) to the
  middle of the joint, straight up through
  the Ø12 hole on the axis of the next part's hub, and under the **two cable arches** on top of its prong (opening
  10 × 6 mm, the plug fits through). On the axis the cable only twists (±120°); it doesn't slide.
- **Along each link (over the top):** from the second arch, over the fork base and straight along the top of the link
  to the **notch at the back of the next servo's clamshell**, down into its chamber and forward to its socket. The
  cable never dips into the tube, which keeps it short. It stays clear of every part over the full joint ranges.
- Leave 1–2 cm of slack at each joint. Turn the joint through its full range by hand before you power it.

**Cable lengths.** The links are 130 mm (upper arm and forearm are the same part) so that the stock ~20 cm
STS3215 cables reach. Inside each joint the cable goes from the socket to the axis (about 15 mm) and twists there,
so its length doesn't change with the joint angle:

| Run | Path (worst joint angle) | With bends and plug ends | 20 cm stock cable |
|---|---|---|---|
| Board → shoulder | ~11 cm | ~14 cm | OK |
| Shoulder → elbow (upper arm) | ~16 cm | ~18 cm | OK |
| Elbow → wrist pan (forearm) | ~16 cm | ~18 cm | OK |
| Wrist pan → pitch | ~23 cm | ~25 cm | **too short: longer cable** |
| Pitch → roll | ~19 cm | ~21 cm | **too short: longer cable** |
| Roll → gripper | ~15 cm | ~17 cm | OK |

Longer cables: a 5264 3-pin bus-servo cable (e.g. Waveshare's 5264-3PIN, for the ST / SC series) in 300 mm or 900 mm.
Each arm needs 2 for the wrist (4 for both arms). With 900 mm cables, coil the extra loosely (no knots) and zip-tie
it where the cable doesn't move: on top of the wrist bracket behind its arches (wrist pan → pitch) and on the tilt
fork's outer face (pitch → roll).

**Wrist cable routes** (checked clear of every part over the wrist pan, pitch and roll ranges):
- **Wrist pan → pitch:** up through the hole on the wrist-bracket hub's axis, under its two arches, over the front edge, down the
  front face of the bracket (24 mm to the −Y side, so the camera clears it at full tilt) through the clip on the
  bottom half, and into the pitch servo's chamber through the opening in the front of the pitch housing.
- **Pitch → roll:** from the pitch servo's socket to the joint axis, out through the hole on the tilt fork's hub axis, under the two
  arches on the fork's outer face, up the side of the fork base and over the roll clamp into its channel, down to
  the roll servo's socket.
- **Roll → gripper:** up the same clamp channel, under the clip on the clamp's −Y side, down the clamp's front into
  the gap between clamp and gripper housing. There it wraps around the roll hub (about 3 cm of slack for ±120°)
  and goes through the channel in the housing's back wall to the gripper servo.

Dry-fit one cable on the first printed link before printing the rest: plug it into both servos, lay it along the
route, and turn the joint through its whole range by hand.

**Cable bending.** Only the short piece inside each joint moves: it twists ±120° over about 25–30 mm between the
socket and the top of the hub (the roll joint is the exception: there the cable wraps around the roll hub in the gap
behind the gripper housing, with about 3 cm of slack). Everything outside the joints (the arches, the clips, the
notches) is fixed to one part and doesn't flex. A 6 × 3 mm strip along every route clears every part over the whole
joint ranges (checked in CAD). Flex life can't be checked in CAD; watch the cables at the hub holes in the first
hours of use.

## Joint ranges (checked against the geometry)

The ranges come from rotating each joint through its full travel and checking every part for collisions:

| Joint | Collision-free range | Suggested software limit |
|---|---|---|
| Shoulder pan (J1) | ±119° (stopped by the base pillar) | ±115° |
| Elbow pan (J2) | ±130° | ±125° |
| Wrist pan (J3) | ±127° | ±122° |
| Wrist pitch | -20° (up) to +120° (down) | -15° to +115° |
| Wrist roll | Full turn, no collision | ±120° (the cable loop in the roll gap; ±90° would also do) |

The full-robot motion planner assumed ±135° on the pan joints. Before reusing its parked poses, update the limits to the numbers above.

## Changes from the concept model

These came up when the parts were made real:

- **Top bearing raised.** The servo connectors sit on the horn face. Their plugs need space, so the top bearing moved up by 3 mm. The joint stack is now 75 mm tall, up from 70 mm.
- **Roll joint uses the horn only.** In the concept model, the idler-side yoke had a web under the roll servo. It hit the servo case after ±20° of roll. The gripper now hangs on the roll horn only, which is what the SO-100 does. The roll servo is fully enclosed by the tilt fork clamp, so roll is limited only by the cable.
- **Gripper blocks offset outboard.** Each finger's MGN9H block sits outboard of the finger. Both 40 mm blocks fit on the 170 mm rail, and the gripper still closes fully. Maximum opening is 90 mm, which matches the 90 mm width rule in the planner.
- **Shoulder height.** The shoulder shaft is 40 mm above the mount plate, not 39 mm.
- **Pitch axis.** The pitch axis is 74.6 mm below the arm plane, not 73.5 mm.
- **Hand length.** The fingertip is about 6 mm further from the wrist than in the concept model.

## Measured vs assumed

- **Measured from the STEP/STL:** all servo geometry above.
- **Assumed:**
  - The horn thread is M3. The Ø2.5 tapped holes in the CAD model match M3. Check with one screw before printing.
  - Which face drives: the CAD model can't show it. From the real servo (user, 2026-11-06): the cable-socket face is the free idler, the other face is the output.
  - MGN9H catalogue dimensions.
  - The camera envelope (lens Ø16 × 16 mm, parts up to 6 mm behind the board) is approximate: check your module's lens and plug clear the cradle window.
- **Not included:** leader arms, lift, carriage and mobile base. Those are mostly catalogue or extrusion parts.

## About the STEP files

The STEP files contain exact solid bodies, but they are faceted: curved surfaces are made of small flat facets about 0.6 mm wide. They import as solid bodies you can measure, combine with other parts, and mate. They are not feature-history CAD. To change a dimension, edit the parameter in `../mobile_manipulator_description/generator/` and rebuild: run `python3 export_all.py ../../SCARA_arm_CAD` there, which needs `manifold3d`. To get editable faces in Fusion 360, import the STL from `assembly_stl/` and use Mesh → Convert Mesh → Prismatic.
