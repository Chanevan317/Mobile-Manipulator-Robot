# Leader arm v2 (teleoperation)

![leader arm](preview.png)

You move this arm by hand, and the SCARA follower arm copies its joint angles 1:1. The leader has the same
joints in the same order: shoulder pan, elbow pan, wrist pan, wrist pitch, wrist roll, plus a trigger for the gripper.

- **The joints are STS3215 servos**, the same servo as the follower but in the low-gear "leader" versions that are
  easy to turn by hand. They read their own angle, so there are no extra sensors and no extra electronics: the leader
  uses the same Waveshare bus board and the same LeRobot code as the follower. Because the joints are servos, the
  leader can also move itself to match the follower (for example after you let go of the clutch).
- **The servos are the structure.** Each link is one flat printed plate between two servos. There are 8 printed
  parts, no heat-set inserts and no bearings.
- **It hangs.** The shoulder sits at the top of a stand and each joint hangs below the one before it, so the wrist is
  the lowest part. The hand can point straight down (the usual picking pose) without hitting anything, just like the
  follower's hand.
- **It is a scaled copy of the follower** seen from above: links 52 mm (0.4 × 130 mm) and the wrist pitch 16 mm in
  front of the wrist pan (0.4 × 40 mm), so the follower's gripper moves the way your hand moves. The grip itself is
  full size. The whole leader is about 33 cm tall and stands on a 20 × 9 cm foot.
- The clutch and the lift are on a **USB foot pedal** (3 pedals: clutch, lift up, lift down), so both hands stay on
  the grips and there is only one lift control for both arms.

Made by `mobile_manipulator_description/generator/leader.py` (shape), `leader_check.py` (collision checks) and
`leader_export.py` (these files). The old v1 design (AS5600 encoders) is in the git history.

## Printed parts (one leader)

| File | What it is | Size on the bed (mm) | Volume (cm³) |
|---|---|---|---|
| `01_stand_top.stl` | holds the shoulder servo + the upper half of the stand column | 122 × 40 × 176 | 185 |
| `02_stand_foot.stl` | foot plate + the lower half of the column, with a 45 mm plug that goes into the upper half | 203 × 90 × 195 | 186 |
| `03_link1.stl` | shoulder fork and first link; the elbow servo hangs under its far end | 77 × 30 × 55 | 20.9 |
| `04_link2.stl` | second link; the wrist-pan servo hangs under its far end | 77 × 30 × 8 | 13.8 |
| `05_wrist.stl` | plate under the wrist-pan servo + clamp holding the pitch servo | 44 × 36 × 28 | 16.3 |
| `06_pitch_fork.stl` | fork around the pitch servo + plate holding the roll servo | 35 × 49 × 50 | 18.0 |
| `07_grip.stl` | the handle; holds the trigger servo | 80 × 29 × 35 | 18.6 |
| `08_trigger.stl` | trigger lever, finger bar under the grip | 39 × 49 × 39 | 7.1 |

About 466 cm³ in total, of which the stand is 371 cm³. The part you move (links, wrist, grip) is only about 92 cm³
of plastic; most of its weight is the five servos (55 g each).

The STLs are already turned to their best side for printing. Print settings: the same as the SCARA arm (PLA, 0.2 mm,
4 walls, 15 % cubic infill), with at least 5 top and 5 bottom layers on the links: the plates bend, so their top and
bottom skins carry the load, not the infill.
Turn supports on (tree): `03_link1` needs them under its top prong, and the servo clamps in `01_stand_top` and
`05_wrist` need them inside the pocket.

## Purchased parts (one leader)

| Item | Qty | Notes |
|---|---|---|
| STS3215 servo, leader version | 6 | see the table below. Leader 1 uses the 6 servos from your SO-100 leader |
| Servo horns | 6 + 2 | each servo comes with a driving horn and a free (idler) horn; the shoulder and the pitch use both |
| Waveshare bus servo board + power supply | 1 | the same as the follower's |
| Servo cables (3-pin, 15–20 cm) | 6 | the stock cables are fine; leave a loose loop at every joint |
| USB foot pedal, 3 pedals | 1 | clutch, lift up, lift down (shared by both leaders) |
| M2 × 6 self-tapping screws | 24 | the SO-100 servo-case screws: 4 per servo |
| M3 × 8 socket-head screws | 24 | horns: shoulder 4 + 4, elbow 4, wrist pan 4, pitch 4 + 4 |
| M3 × 6 socket-head screws | 8 | roll horn (from inside the grip) 4, trigger horn 4 |
| M3 × 50 socket-head screw + M3 nyloc nut | 2 + 2 | join the two stand parts (across the column) |
| M5 screws or a C-clamp | 4 / 1 | hold the foot on the desk (the leader tips over if you don't) |

**Which servo goes where.** Any low-gear version works on the pan joints, because gravity doesn't act on them. Put
the stiffest servo on the **wrist pitch**: it holds the hand up.

| Joint | Servo |
|---|---|
| Shoulder, elbow, wrist pan | C044 (1:191) or C046 (1:147) |
| Wrist pitch | the stiffest one you have (C001 1:345 if your leader has one, else C044) |
| Wrist roll, trigger | C046 (1:147) |

Never put a C044 or C046 in the follower: it can't hold the follower's payload.

## Assembly

1. **Set every servo to its centre (position 2048) before you fit a horn**, as on the follower. Give each servo its
   bus ID (1 shoulder … 6 trigger) with LeRobot's motor setup before you build it in.
2. **Stand.** Push the foot's plug into the upper column and put the 2 × M3 × 50 bolts across, with nyloc nuts.
   Clamp or screw the foot to the desk.
3. **Shoulder.** Slide the shoulder servo, back end first, into the clamp at the top of the stand, cable sockets up.
   Fix it with 4 × M2 × 6 (2 from above, 2 from below).
4. **Link 1.** Screw its bottom plate to the shoulder's output horn (4 × M3 × 8 from below) and its top prong to the
   idler horn (4 × M3 × 8 from above).
5. **Elbow and wrist pan.** Hold each servo under the far end of its link, cable sockets up against the plate, and
   screw it on with 4 × M2 × 6 from above. The cable plugs go up through the slot in the plate. Then screw the next
   plate (link 2, then the wrist) to that servo's horn (4 × M3 × 8).
6. **Pitch.** Slide the pitch servo, back end first, up into the clamp under the wrist plate (output horn on the
   left when you look along the arm from the shoulder), fix it with 4 × M2 × 6, then screw the pitch fork to both of its horns
   (4 + 4 × M3 × 8).
7. **Roll.** Screw the roll servo to the back plate of the pitch fork (4 × M2 × 6 from behind), body down, output
   horn forward.
8. **Grip.** Screw the grip to the roll horn (4 × M3 × 6 from inside the grip). Slide the trigger servo into the grip
   from the open side and fix it with 4 × M2 × 6 through the closed side. Screw the trigger to its horn (4 × M3 × 6).
9. **Cables.** Chain the servos (board → shoulder → elbow → … → trigger). Leave a loose loop at every joint so it can
   turn its full range, and tie the cables to the links.

## Joint ranges (checked in CAD)

| Joint | Leader free range | Follower limit |
|---|---|---|
| Shoulder pan | ±125° without contact | ±115° |
| Elbow pan | ±135° without contact | ±125° |
| Wrist pan | ±132° without contact | ±122° |
| Wrist pitch | −20° … +125°; the fork touches the wrist clamp at about −21° (a natural end stop) | −15° … +115° |
| Wrist roll | ±130° without contact | ±120° |
| Trigger | 0 … 25° (the finger bar meets the grip between 25° and 30°) | maps to the 0–90 mm jaw opening |

Also checked: 63 combined poses (hand pointing down or back with the pan joints at their limits) are all clear, a
grid of 605 hand-down poses never touches the stand, and the hand stays at least 19 mm above the desk.

## Strength (FEA, not tested yet)

Every load-carrying part was checked with a finite-element analysis (solid PLA, quadratic elements) for 10 N at the
grip, pushed down, sideways and forward, with the arm stretched out, plus the weight of everything hanging below.
10 N is a firm push; normal teleoperation is 3–5 N. Printed at 4 walls / 15 %, a part keeps roughly 70 % of solid
strength; PLA is about 45 MPa along the layers and 10–20 MPa across them.

| Part | Worst stress (99 % of the part) | Where | Across the layers | Verdict |
|---|---|---|---|---|
| Link 1 | 11.4 MPa | next to the shoulder horn, side push | 1.7 MPa | OK (safety factor ~3 at 10 N) |
| Link 2 | 5.1 MPa | next to the elbow horn, push down | 1.4 MPa | OK |
| Wrist plate | 4.1 MPa | | 1.8 MPa | OK |
| Pitch fork | 2.2 MPa | back plate (4 mm since the FEA, was 2.5 mm: 17.7 MPa) | 3.5 MPa peak | OK |
| Stand top | 4.4 MPa peak | beam–column corner | 2.6 MPa | OK |
| Stand foot | 5.4 MPa peak | top of the rear rib | 5.4 MPa peak | OK at normal forces; the weakest spot (safety factor ~1.3–2.6 at 10 N) |

At 10 N the grip moves about 3 mm (stand ~1.2 mm, links ~1.8 mm); at normal forces about 1 mm. **Clamp or screw
the foot to the desk.**

The one thing the numbers can't tell: the elbow and wrist-pan servos hold their links by the output horn only, so a
push at the hand bends their output shafts (about 1.7 N·m at 10 N, 0.5 N·m at 3 N). **Test one stage before printing
everything:** print `04_link2` and `05_wrist`, screw on two servos, hang about 1 kg from the wrist end and push it
around. If the horn wobbles, tell me before printing the rest.

## Using it

- **Place each leader like its follower arm.** The follower arms' zero poses point 45° toward the rear tote; turn each
  leader the same way relative to where you sit.
- **If you let go of the grip, the hand drops to hanging straight down** (the pitch is the only joint gravity acts
  on). Release the clutch pedal first, or let the software hold the pitch with a little servo torque.
- The hand moves only sideways and forward at a fixed height, like the follower's SCARA arm; the height is on the pedals.

## Still to do

- A LeRobot "teleoperator" for this leader: read the 6 servos, map each joint 1:1 (with direction and zero offset) to
  the follower, the foot pedal for the clutch and the lift, and a gentle "spring" on the trigger (low servo torque)
  so it opens by itself.
- A place for the bus board on the stand (it can sit on the foot plate for now).
- Cable clips on the links.
