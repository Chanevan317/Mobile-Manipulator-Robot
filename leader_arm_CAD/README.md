# Leader arm v3 (teleoperation)

![leader arm](preview_leader_arm.png)

You move this arm by hand, and the follower arm copies its joint angles 1:1. **v3 is a 0.7-scale copy of the
follower**: the same joints in the same order, the same layout and the same proportions. Your hand moves the way the
follower's hand moves, without translating in your head.

- **Same shape as the follower, 0.7 × the size.** Links 91 mm (follower 130 mm); the wrist pitch 28 mm in front of the
  wrist pan (follower 40); the roll axis 59.5 mm further (follower 85). The arm lies flat like the follower's: every
  pan joint is a fork around its servo, and all the servos sit in one plane.
- **One arm module, two bases.** The same arm works for **both followers**; only the base changes:
  - **Mono base**: one pedestal on a foot. The shoulder turns **±180°**, like the mono arm's ring.
  - **Prototype base**: two pedestals 210 mm apart (0.7 × the follower's 300 mm), each arm turned ±135° like the
    follower's mounts, with a stand-in for the lift column between them. Two leaders, one per hand.
  - Swapping takes 4 screws: the arm's shoulder mount bolts onto the pedestal top, which has holes every 45°.
- **Light leader-only parts.** No bearings, no heat-set inserts: each joint is carried by the servo's own output
  horn and idler horn (like the SO-100 leader). The joints are **STS3215 low-gear servos**, which also read their
  own angle: no extra sensors, and the same Waveshare board and LeRobot code as the follower.
- **A pinch gripper you hold like the robot's gripper.** Thumb in one finger loop, two or three fingers in the
  other; squeezing closes the jaws, opening your hand opens them, so you feel how wide the robot's gripper is. The
  loops pull the jaws open too (as on surgical robots' masters). It sits where the follower's gripper sits, on the
  roll axis, so turning your hand turns the robot's gripper the same way.
- **One hand or two, both on the gripper** (as you held the SO-100 leader): one pinching in the loops, the other
  steadying and steering the gripper. No handle on the arm is needed: holding the gripper's position and heading
  fixes the arm's whole shape (it moves in a plane, with three pan joints for three things: x, y and heading).
  The base is on a **gamepad** between picks. The **lift** is on a small **joystick box** beside the leader (KY-023
  thumbstick + Raspberry Pi Pico; push forward = up): see [`lift_stick/`](lift_stick/README.md).
- **The pinch:** thumb and fingers sit **38 mm apart closed and 84 mm apart fully open (40°)**, the comfortable pinch
  range, with 46 mm of finger travel for the robot's 0–90 mm.
- **Made for sitting at a desk.** The arm is 28 cm above the desk; with the gripper hanging straight down it stays
  22 mm above it.

Made by `mobile_manipulator_description/generator/leader.py` (shape), `leader_check.py` (collision checks),
`leader_export.py` (these files) and `leader_render.py` (the preview). v1 (AS5600 encoders) and v2 (hanging servo
stack, 0.4 scale) are in the git history.

## Printed parts

| File | What it is | Mono | Prototype | Volume (cm³) |
|---|---|---|---|---|
| `01_shoulder_mount.stl` | holds the shoulder servo upright (horn up); 4 × M3 onto the pedestal | 1 | 2 | 46 |
| `02_link1.stl` | upper arm (91 mm): plate on the shoulder horn, clamp on the elbow servo | 1 | 2 | 28 |
| `03_link2.stl` | forearm (91 mm): fork around the elbow servo, clamp on the wrist-pan servo | 1 | 2 | 32 |
| `04_wrist.stl` | fork around the wrist-pan servo + clamp holding the pitch servo | 1 | 2 | 32 |
| `05_pitch_fork.stl` | fork around the pitch servo + plate holding the roll servo | 1 | 2 | 30 |
| `06_gripper_frame.stl` | on the roll horn: holds the gripper servo (horn down, body forward) | 1 | 2 | 22 |
| `07_gripper_fixed_jaw.stl` | the fixed jaw with its finger loop, bolted under the frame | 1 | 2 | 33 |
| `08_gripper_jaw.stl` | the moving jaw on the gripper servo's horn, with its finger loop | 1 | 2 | 27 |
| `09_pedestal.stl` | column from the desk to the shoulder mount | 1 | 2 | 190 |
| `10_mono_foot.stl` | foot for one leader (mono) | 1 | – | 174 |
| `11_proto_foot_left.stl`, `12_proto_foot_right.stl` | the two halves of the prototype foot, with the column stand-in | – | 1 each | 205 each |

Print like the follower (0.2 mm, 4 walls, 15 % cubic). Orientations are set in the STLs: the links, wrist and
the wrist print on their side, the gripper frame and the moving jaw upside down, the fixed jaw standing on its loop. The pedestal's cap overhangs its column by 8 mm (use tree supports there if your printer sags).
Sizes are in `printed_parts.csv`; screws in `fasteners.csv` (per arm module).

## Purchased parts (one arm module)

| Item | Qty | Notes |
|---|---|---|
| STS3215 servo, leader version | 6 | see below |
| Servo horns | 6 + 3 | output horns on all six; idler horns on the elbow, wrist pan and pitch (the forks) |
| Waveshare bus servo board + power supply | 1 | the same as the follower's |
| Servo cables (3-pin, 15–20 cm) | 6 | the stock cables; leave a loose loop at every joint |
| Gamepad (USB / Bluetooth) | 1 | drives the base and the lift between picks (one for both leaders) |
| USB foot pedal (optional) | 1 | clutch, if you'd rather keep it off the gamepad |
| M2 × 6 self-tapping | 24 | servo cases (4 per servo) |
| M3 × 8 | 32 | horns: 4 on the shoulder and the gripper servo, 4 + 4 (output + idler) on the elbow, wrist pan and pitch |
| M3 × 6 | 4 | roll horn (gripper frame) |
| M3 × 20 + nut | 4 + 4 | shoulder mount to pedestal (nuts in the pedestal cap's underside) |
| M3 × 30 + 2 nuts | 2 | fixed jaw to the gripper frame (nuts slide into the frame's side wall) |
| M5 × 12 + nut | 3 + 3 | pedestal to foot (nuts in the hex pockets under the foot; the bolt ends flush) |
| C-clamps | 2 | hold the foot on the desk |

**Which servo goes where.** Gravity acts only on the wrist pitch, so put the stiffest servo there.

| Joint | Servo |
|---|---|
| Shoulder, elbow, wrist pan | C044 (1:191) or C046 (1:147) |
| Wrist pitch | the stiffest you have (C044) |
| Wrist roll, gripper | C046 (1:147) |

Never put a C044 or C046 in the follower: it can't hold the follower's load.

## Assembly

1. Screw the shoulder servo into the shoulder mount, horn up (4 × M2 through the bottom).
2. Elbow and wrist-pan servos: slide each one's rear end into the clamp at the end of the link before it, and screw
   it (4 × M2). Then fit the next fork: bottom prong on the output horn (4 × M3 × 8), top prong on the idler (4 × M3 × 8).
3. Link 1 goes on the shoulder horn (4 × M3 × 8).
4. Pitch servo: rear end up into the wrist's clamp (4 × M2); pitch fork on its two horns; roll servo into the fork's
   back plate (4 × M2).
5. Gripper servo into the gripper frame's clamp, horn down, body forward (4 × M2); frame on the roll horn
   (4 × M3 × 6); fixed jaw under the frame (2 × M3 × 30 into the nuts in the frame's side wall); moving jaw on the
   gripper servo's horn (4 × M3 × 8) with the blades just touching.
6. Pedestal onto the foot: 3 × M5 × 12 down through the flange into nuts in the hex pockets under the foot. Shoulder mount onto the pedestal (4 × M3 × 20, nuts in the cap's underside),
   **turned like the follower's arm**: straight ahead for the mono base, ±135° on the prototype base.
7. Run the servo cables along the links to the bus board; the pedestal has a cable exit at its foot.

## Joint ranges (checked in CAD)

Every joint reaches at least the follower's limit + 5°:

| Joint | Leader limit | Clear to | Follower (mono / prototype) |
|---|---|---|---|
| Shoulder | ±180° (mono), ±120° (prototype) | the whole turn | ±180° / ±115° |
| Elbow | ±130° | ±148° | ±117° / ±125° |
| Wrist pan | ±127° | ±144° | ±122° |
| Wrist pitch | −20° … +120° | −20° … +125° (just touches at −25°) | −15° … +115° |
| Wrist roll | ±125° | ±130° | ±120° |
| Gripper | 0 … 40° (thumb-to-fingers 38 → 84 mm) | 0° (the blades touch: closed stop) … 45° | 0 … 90 mm |

**Where it can't go** (and the follower can't either): with the elbow folded to its limit and the wrist turned back,
a level hand points into the pedestal. The follower's hand hits its own column and carriage in those poses, much
more. On the prototype base the two leaders meet when both reach far inward, as the two follower arms do.

## Using it

- **Sit facing the leader as you would face the robot.** On the mono base the arm's zero pose points straight ahead;
  on the prototype base each arm points 45° back, like the follower arms.
- **Hands:** thumb and two or three fingers in the gripper's loops (either hand); the other hand, if you like, on
  the gripper frame to steady it. Release the clutch before letting go: the pitch servo's gear friction holds the gripper in most poses; if
  it sinks, the software can hold it with a little servo torque.
- The gripper servo can act as a soft spring (low torque toward open) so the jaws open when you relax your hand.
- The hand moves sideways and forward at a fixed height, like the SCARA follower; the height is on the pedals.

## Still to do

- The LeRobot "teleoperator": read the 6 servos, map each joint 1:1 (direction and zero offset) to the follower,
  the gripper's 0..40° to the follower's 0..90 mm, the gamepad for the base / lift and the clutch, and a gentle
  "spring" on the gripper servo (low torque) so it opens by itself.
- Print the gripper first and try the pinch: loop sizes and the 40° travel are first guesses.
- A mount for the bus board on the pedestal or foot (it can lie on the foot for now).
- Cable clips on the links.
- Print one link and check the servo fit before printing the rest (the clamps hold the servo's rear end, as v2's
  stand did).
