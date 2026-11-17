# Leader arm v4 (teleoperation)

![leader arm](preview_leader_arm.png)

You move this arm by hand, and the follower arm copies its joint angles 1:1. **v4 is a 0.7-scale copy of the
follower**: the same joints in the same order, the same layout and the same proportions. Your hand moves the way the
follower's hand moves, without translating in your head.

- **Same shape as the follower, 0.7 × the size.** Links 91 mm (follower 130 mm); the wrist pitch 28 mm in front of the
  wrist pan (follower 40); the roll axis 59.5 mm further (follower 85). The arm lies flat like the follower's: every
  pan joint is a fork around its servo, and all the servos sit in one plane.
- **One arm module, two bases.** The same arm works for **both followers**; only the base changes:
  - **Mono base**: one pedestal on a foot. The shoulder turns **±173°** (an end stop at ±174°), like the mono arm's ring.
  - **Prototype base**: two pedestals 210 mm apart (0.7 × the follower's 300 mm), each arm turned ±135° like the
    follower's mounts, with a stand-in for the lift column between them. Two leaders, one per hand.
  - Swapping takes 4 screws: the arm's shoulder hub bolts onto the pedestal top, which has holes every 45°.
- **Every servo cable is a stock 20 cm cable (v4, 2026-11-17), as on the SO-100.** The shoulder servo **turns with
  link 1** (its horn faces down onto a fixed hub, its sockets up), and the **servo board rides on link 1** (a dovetail
  socket on a short tail behind the shoulder servo). No servo cable crosses the shoulder: only the board's USB and power
  cables do, with a slack loop down to the desk. The elbow and wrist-pan servos are **turned over** (sockets down), so
  their cables loop under the joints and go straight on to the next servo. The roll range is **±90°**, so the roll
  cable only twists a quarter turn either way around the roll servo's top. v3 (needed 30–40 cm cables) is in git.
- **The shoulder carries the arm on a ring, not on the servo alone.** The arm (~435 g, its weight 170–215 mm out)
  would hang on the shoulder servo's horn only (0.7–0.9 N·m of tipping load, ~3 N·m more when you press on the
  gripper). A printed ring on the shoulder hub sits **0.3 mm under link 1's beam**: link 1 rides on it as soon as it droops
  or you push down, so the load goes into the hub instead of the servo's gears. A **peg under link 1 hits a rib on
  the ring** at ±174°, so the shoulder can't reach ±180°, where the servo's reading jumps from +180 to −180 (the follower
  would swing the long way round), and its cables can't wind up. This works only if the shoulder's zero is calibrated
  with link 1 pointing straight ahead (away from the rib): the jump then lies inside the blocked ±174..180° gap.
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
  The base is on a **gamepad** between picks. The **lift** is on a one-hand **controller** beside the leader (thumbstick
  + Raspberry Pi Pico, push forward = up, plus a dead-man and a stored-position button): see
  [`../lift_stick/`](../lift_stick/README.md).
- **The pinch:** thumb and fingers sit **38 mm apart closed and 84 mm apart fully open (40°)**, the comfortable pinch
  range, with 46 mm of finger travel for the robot's 0–90 mm.
- **Made for sitting at a desk.** The arm is 28.7 cm above the desk; with the gripper hanging straight down it stays
  22 mm above it.

Made by `mobile_manipulator_description/generator/leader.py` (shape), `leader_check.py` (collision checks), `leader_cables.py` (plugs, cable spaces, board, cable lengths),
`leader_export.py` (these files) and `leader_render.py` (the preview). v1 (AS5600 encoders) and v2 (hanging servo
stack, 0.4 scale) are in the git history.

## Printed parts

| File | What it is | Mono | Prototype | Volume (cm³) |
|---|---|---|---|---|
| `01_shoulder_hub.stl` | fixed hub on the pedestal: the shoulder servo's horn screws onto it from below (through its hollow tube); 4 × M3 onto the pedestal; support ring under link 1 + stop rib | 1 | 2 | 88 |
| `02_link1.stl` | upper arm (91 mm): clamps on the shoulder and elbow servos' rear ends, stop peg under the beam, cable clip on the elbow clamp, board socket on the tail behind the shoulder servo | 1 | 2 | 32 |
| `03_link2.stl` | forearm (91 mm): fork around the elbow servo (turned over: idler prong underneath), clamp on the wrist-pan servo, cable arch under the clamp | 1 | 2 | 34 |
| `04_wrist.stl` | fork around the wrist-pan servo (turned over) + clamp holding the pitch servo, cable clip on its −Y side | 1 | 2 | 33 |
| `05_pitch_fork.stl` | fork around the pitch servo + plate holding the roll servo | 1 | 2 | 32 |
| `06_gripper_frame.stl` | on the roll horn: holds the gripper servo (horn down, body forward), cable notch in the back plate, cable arch on top | 1 | 2 | 20 |
| `07_gripper_fixed_jaw.stl` | the fixed jaw with its finger loop, bolted under the frame | 1 | 2 | 33 |
| `08_gripper_jaw.stl` | the moving jaw on the gripper servo's horn, with its finger loop | 1 | 2 | 27 |
| `09_pedestal.stl` | hollow column from the desk to the shoulder hub | 1 | 2 | 194 |
| `10_mono_foot.stl` | foot for one leader (mono) | 1 | – | 174 |
| `11_proto_foot_left.stl`, `12_proto_foot_right.stl` | the two halves of the prototype foot, with the column stand-in | – | 1 each | 205 each |

Print like the follower (0.2 mm, 4 walls, 15 % cubic). Orientations are set in the STLs: the links, wrist and
the wrist print on their side, the gripper frame and the moving jaw upside down, the fixed jaw standing on its loop. The pedestal's cap overhangs its column by 8 mm, now as a ring around a 44 mm hole (use tree supports there if your printer sags).
Sizes are in `printed_parts.csv`; screws in `fasteners.csv` (per arm module).

## Purchased parts (one arm module)

| Item | Qty | Notes |
|---|---|---|
| STS3215 servo, leader version | 6 | see below |
| Servo horns | 6 + 3 | output horns on all six; idler horns on the elbow, wrist pan and pitch (the forks) |
| Waveshare bus servo board + power supply | 1 | the same as the follower's, on the SO-101 mounting plate (print `../SCARA_arm/SO100/WaveShare_Mounting_Plate_SO101.stl`), in link 1's socket |
| Servo cables (3-pin 5264) | 6 | **all stock 20 cm** (the SO-100's): lengths in **Cables** below |
| Gamepad (USB / Bluetooth) | 1 | drives the base and the lift between picks (one for both leaders) |
| USB foot pedal (optional) | 1 | clutch, if you'd rather keep it off the gamepad |
| M2 × 6 self-tapping | 23 | servo cases (4 per servo, 3 on the gripper servo): **the SO-100's own case screws** |
| M3 × 6 | 36 | every horn, output and idler (the shoulder's 4 go up from below into the hub-side horn): **the SO-100's own horn screws**. The heads sit in Ø 6.2 pockets, 3.5 mm
  of plastic over the horn (the SO-100's parts leave 3.4 mm); on the idler side the pockets are deep inside the tall boss |
| M3 × 20 + nut | 4 + 4 | shoulder hub to pedestal (nuts in the pedestal cap's underside) |
| M3 × 30 + 2 nuts | 2 | fixed jaw to the gripper frame (nuts slide into the frame's side wall) |
| M5 × 12 + nut | 3 + 3 | pedestal to foot (nuts in the hex pockets under the foot; the bolt ends flush) |
| C-clamps | 2 | hold the foot on the desk |

**Screws = the SO-100 set (2026-11-17).** Every servo screw is one the SO-100 uses (M2 × 6 self-tapping, M3 × 6). Only the
base and jaw bolts (M3 × 20, M3 × 30, M5 × 12 and their nuts) are extra. The idler bosses are about a third weaker with
the short screws in deep pockets, still ~16 × under PLA's strength across the layers at a 10 N push on the gripper.

**Which servo goes where.** Gravity acts only on the wrist pitch, so put the stiffest servo there.

| Joint | Servo |
|---|---|
| Shoulder, elbow, wrist pan | C044 (1:191) or C046 (1:147) |
| Wrist pitch | the stiffest you have (C044) |
| Wrist roll, gripper | C046 (1:147) |

Never put a C044 or C046 in the follower: it can't hold the follower's load.

## Assembly

1. **Shoulder:** slide the shoulder servo's rear end into link 1's rear clamp, sockets up, and screw it (4 × M2;
   the 2 underneath are covered once the hub is on). Then the hub under its output horn: 4 × M3 × 6 **up** through the
   hub's hollow tube into the horn, **before** the hub goes on the pedestal. Turn link 1 both ways by hand to the stops
   (±174°): its beam rides 0.3 mm over the support ring (it may touch once the arm's weight is on it) and must not rub
   anywhere else.
2. Elbow and wrist-pan servos are **turned over** (horn up, sockets down): slide each one's rear end into the clamp at
   the end of the link before it and screw it (4 × M2). **Plug in both of its cables now** (the sockets face down, into
   the gap above the next fork's bottom prong). Then fit the next fork: top prong on the output horn (4 × M3 × 6),
   bottom prong on the idler (4 × M3 × 6, up the deep pockets in its boss: a magnetic or ball-end driver helps).
3. Pitch servo: rear end up into the wrist's clamp (4 × M2); plug in its cables; pitch fork on its two horns
   (4 × M3 × 6 on each); roll servo into the fork's back plate (4 × M2, **before** the fork goes on the pitch servo;
   the two inner screws need a stubby driver), its cables through the slot in the plate.
4. **Frame first:** gripper frame on the roll horn (4 × M3 × 6; the gripper servo would cover these screws). Then slide
   the gripper servo into the frame's clamp **sideways from the open side** (horn down, body forward) and screw it
   (3 × M2: 2 on top, 1 underneath on the fixed jaw's side). Fixed jaw under the frame (2 × M3 × 30 into the nuts in
   the frame's side wall; its hole lets you reach that bottom case screw later). Moving jaw on the gripper servo's horn
   (4 × M3 × 6) with the blades just touching.
5. Pedestal onto the foot (3 × M5 × 12 into the nuts under the foot).
6. Shoulder hub onto the pedestal (4 × M3 × 20, nuts in the cap's underside), **turned like the follower's arm**
   (holes every 45°; on the prototype foot each arm at the follower's ±135°).
7. Route the cables as in **Cables** below; slide the board's plate down into link 1's socket and plug in the shoulder
   servo's cable. The board's USB and power cables hang from link 1 in a loose loop down to the desk.

## Joint ranges (checked in CAD)

Every joint reaches at least the follower's limit + 5°, except the roll (±90°: enough for nearly every grasp, and a
larger twist is hard to do with your hand anyway):

| Joint | Leader limit | Clear to | Follower (mono / prototype) |
|---|---|---|---|
| Shoulder | ±173° (mono; end stop at ±174°), ±120° (prototype, in software) | the stop | ±180° (plans use ±170°) / ±115° |
| Elbow | ±120° | ±138° (link 2 meets the support ring) | ±115° / ±125° |
| Wrist pan | ±127° | ±146° | ±122° |
| Wrist pitch | −20° … +120° | −25° … +125° | −15° … +115° |
| Wrist roll | **±90°** (in software; no end stop; the roll cable is sized for it) | turns freely | ±120° |
| Gripper | 0 … 40° (thumb-to-fingers 38 → 84 mm) | 0° (the blades touch: closed stop) … 45° | 0 … 90 mm |

**Where it can't go** (and the follower can't either): with the elbow folded to its limit and the wrist turned back,
a level hand points into the shoulder hub. The follower's hand hits its own column and carriage in those poses, much
more. On the prototype base the two leaders meet when both reach far inward, as the two follower arms do.

**Limits only software can set.** These can't be printed as stops, so the teleoperator code must clamp them:
- **Wrist roll:** the roll servo's case reaches 36.8 mm from the axis and ends 1.2 mm before the gripper frame, so a
  peg and lug won't fit without a bigger wrist. Calibrate the roll's zero with the gripper level: the ±180° jump then
  sits with the gripper upside down, past any comfortable twist of the hand.
- **Wrist roll ±90°:** the roll cable is sized for a quarter turn either way; don't twist further.
- **The follower's narrower ranges:** the mono follower's elbow stops at ±115°, the leader's at ±120°. Clamp every
  joint to the follower's limits so the follower never chases a pose it can't reach.
- **The mono follower's column:** its arm turns around the lift column, the camera mast and the carriage. The leader
  has none of them (nothing fixed can stand above the shoulder servo), so it can be folded into poses the follower
  would collide in. The software must refuse those poses.

## Cables

Every servo's cable plug has room (checked over every joint's range), and each cable that crosses a joint has a free
space for its slack loop there, so turning the joint only moves the loop (`leader_cables.py` checks all of it). The
servos are on one bus: board → shoulder → elbow → wrist pan → pitch → roll → gripper.

| Cable | Route | Needed (+15 %) |
|---|---|---|
| board → shoulder | both ride on link 1: from the board behind the shoulder servo over to its socket on top | 10 cm |
| shoulder → elbow | along link 1's top, down the **clip on the elbow clamp's side**, under the clamp to the elbow servo's socket (no joint crossed) | 16 cm |
| elbow → wrist pan | a loop in the gap **under** the elbow servo, out beside the fork 60° from straight back, across link 2's short beam, through the **arch under the clamp** to the wrist-pan servo | 17 cm |
| wrist pan → pitch | a loop under the wrist-pan servo, out beside the fork, down the **clip on the wrist's −Y side** to the pitch servo | 15 cm |
| pitch → roll | a loop **inside the pitch fork's idler side**, along it to the back plate and through its slot to the roll servo | 15 cm |
| roll → gripper | up behind the back plate, over the roll servo, a quarter-turn **twist around the roll servo's top** (±90°), into the **notch in the gripper frame's back plate**, along the **arch on top** to the gripper servo | 20 cm (the tightest) |

All six are stock 20 cm cables. "Needed" is the path over the geometry plus each joint's worst position, 12 mm per plug
end, plus 15 % (`leader_cables.py`); the roll → gripper run uses the whole 20 cm, so dry-fit it first. No loose loops
outside the arm except the board's USB + power cables.

**The board** slides down into the dovetail socket on link 1's tail (the same SO-101 plate as on the follower, checked
with its real STL: held in every direction, slides out upward). Nothing else touches it in 1428 random mono poses and
524 prototype poses.

**Assumed, not measured:** the plug sticks 3.5 mm out of the servo and its wires need 5 mm before they bend; the board
is at most 16 mm tall; the cable is a flat 5 × 2.6 mm.

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
- Print one link and check the servo fit before printing the rest (the clamps hold the servo's rear end, as v2's
  stand did).
- **Servo fit:** the same as the follower and the SO-100: line-to-line around the case, 0.1 mm play over the
  servo's flat case face on the output side, M2 × 6 self-tapping case screws. If a servo is still too tight to slide
  into a clamp, sand the clamp lightly rather than forcing it (the clamp walls are thin).
