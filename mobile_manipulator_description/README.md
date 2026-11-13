# mobile_manipulator_description: full robot for simulation

This package is the full robot: a mecanum mobile base, the lift, the carriage, and two SCARA arms. Every joint moves.

The arm links use the real CAD parts from `../CAD/SCARA_arm/`, with their real masses. The base, column and carriage are simple shapes with estimated masses, the same as in the earlier concept model.

## Files

| Path | Use it for |
|---|---|
| `urdf/mobile_manipulator_prototype.urdf` | **Prototype layout** (the current design): Isaac Sim, PyBullet, MuJoCo (`compile`), Gazebo. Mesh paths are relative (`../meshes/...`). |
| `urdf/mobile_manipulator_prototype_ros.urdf` | The same robot for ROS 2 / RViz, with `package://mobile_manipulator_description/...` paths. |
| `urdf/mobile_manipulator_prototype_mono_arm.urdf` (+ `_ros`) | **Experiment:** one arm whose shoulder is a ring around the lift column. See [Mono-arm experiment](#mono-arm-experiment-prototype_mono_arm). |
| `meshes/visual/` | STL files in metres. Shared by both arms. |
| `meshes/collision/` | Convex pieces of every part, about 350 per arm. Each one is ready for PhysX. |
| `scripts/drives.py` | The motor gains, in one place. Both Isaac scripts use it. |
| `scripts/isaac_import_and_test.py` | Imports the robot into Isaac Sim, sets the motor drives, adds a floor, and plays a looping motion demo (`--duration S` stops it, `--save` writes a USD). |
| `scripts/isaac_joint_test.py` | Tests every joint one at a time, or holds poses under gravity (`--hold`). |
| `scripts/isaac_base_test.py` | Drives the mecanum base forward, sideways and turning, and measures how well it moves. |
| `scripts/isaac_grasp_test.py` | Each arm picks a cube from a table, lifts it, swings it and puts it back. |
| `scripts/isaac_shelf_task.py` | Full task: drive to shelf A, pick a product, put it in the tote, drive to shelf B, place it there (`--layout prototype` uses `shelf_task_prototype.py`). |
| `scripts/isaac_table_task.py` | Prototype: drive to table A, both arms pick a product each (top-down) at the same time into the tote, drive to table B, place both there (`--table_h`, `--sequential`). |
| `scripts/reach_test.py` | No simulator: maps which spots on tables and shelf boards each arm can reach (IK + collision checks), writes a CSV and a PNG. |
| `scripts/isaac_mono_task.py`, `isaac_mono_table_task.py`, `isaac_mono_shelf_task.py` | Mono-arm experiment: table → tote → table without driving, and the table and shelf tasks above done with one arm. |
| `scripts/mono_poses.py` | Mono-arm experiment: the planned poses and moves for those tasks. |
| `scripts/smooth_path.py` | Mono-arm experiment: blends the planned moves into smooth, time-optimal motions (TOPP-RA; `pip install toppra`). On by default; `--no_smooth` turns it off. |
| `scripts/proto_task.py`, `scripts/proto_poses.py` | Prototype task helpers (time-synchronised moves, IK lines, reporting) and the planned poses and routes. |
| `scripts/scara_ik.py` | Inverse kinematics for one arm plus the lift (gripper position, heading and tilt to joint angles). |
| `scripts/self_collision.py` | Turns on self-collision (`--self_collision` in the other scripts). |
| `launch/display.launch.py`, `rviz/display.rviz`, `package.xml`, `CMakeLists.txt` | ROS 2 package: `ros2 launch mobile_manipulator_description display.launch.py` opens RViz with the robot and a slider for every joint |
| `scripts/ros_joint_check.py` | ROS 2: checks joint limits, the zero pose and the direction of every joint, through TF |
| `arm_link_masses.json` | Mass and centre of mass of each arm link |
| `preview_prototype_dual.png`, `preview_prototype_mono.png` | Pictures of the two-arm prototype and of the mono-arm experiment (two views: facing the front / turned 150° toward the tote). `generator/render_robot.py` draws any layout from its URDF |
| `preview_piper_mono.png`, `preview_piper_dual.png` | Pictures only: our body with AgileX PiPER arms instead of the SCARA arms. Mono: one PiPER on a plate on our shoulder ring, so it turns around the column (facing the front / turned 150° toward the tote). Dual: one on each side bracket, each base on a 30° wedge so the arm leans out to its side, away from the lift, each in a different pose (≥ 49 mm from the column and carriage, 136 mm between the arms). PiPER model from agilexrobotics/piper_ros (MIT); not part of this package |

## Layout (prototype)

One layout since 2026-11-09: the older v0 layout (column at the rear, arms forward on carriage beams) was deleted.
The scripts still accept `--layout prototype` (the only choice).

| | `prototype` |
|---|---|---|
| Lift column | 0.18 m in front of the base centre, on the **working face** (the robot's front, +x) |
| Arms | right beside the column at y = ±0.15; zero pose 45° toward the rear tote (±135°) |
| Lift | OpenBuilds C-Beam XLarge, 1000 mm beam (80 mm open face to the rear) on a 70 mm riser, **0–0.885 m** |
| Carriage | XLarge gantry plate (125 × 125 mm) behind the beam + two 8 mm arm brackets beside it, about 1.0 kg |
| Tote | behind the column, 0.35 × 0.48 m |
| Scene camera | on a carriage bracket in front of the column (moves with the lift), looking at the working face; a pan mount is a planned upgrade |
| Lift moment, arms parked / both reaching with 0.25 kg | **0.1 / 14.2 N·m** |
| Tote floor reached, either arm / both | 0.094 / 0.049 m² |
| Arms colliding when each works its own half of the tote | **8 %** of target pairs |
| Reach past the front bumper, hand level / hand down | 0.41 m / 0.22 m |
| Stow pose inside the 50 × 50 cm base | left (30°, 105°, −120°, hand level), right mirrored; clear at any lift |

The moment, tote and collision numbers come from the layout study of 2026-11-06 (180 / 150 links); the reach and stow rows are for the 130 / 130 links.

## Isaac Sim

```
cd mobile_manipulator_description
<isaac-sim>/python.sh scripts/isaac_import_and_test.py
#   --save mobile_manipulator.usd   converts to USD and exits
#   --headless                 no window
```

`<isaac-sim>/python.sh` is for a standalone Isaac Sim install. If Isaac Sim is installed with pip in a conda
environment (as in `env_isaaclab`), use `python` from that environment instead:
`conda activate env_isaaclab && python scripts/isaac_import_and_test.py`.

**Use the script (or a USD it saved), not the GUI import.** The URDF importer makes *acceleration* drives with
its own stiffness (625 per degree) and ignores our gains. The script replaces them with the force drives from
`scripts/drives.py`. If you do import by hand (File → Import → `urdf/mobile_manipulator_prototype.urdf`), use:
fix base off, merge fixed joints off, self-collision off, collider type convex hull (the collision meshes are
already convex pieces), joint drive position. Then set every drive to type *force* with the gains in the table
below, and set the 36 `*_roller_*` joints to stiffness 0 and damping 0.

### Motor gains (`scripts/drives.py`)

These are force drives: torque = kp × position error + kd × speed error, capped at the max force.
The max force and the speed limit come from the URDF `<limit>` values.

| Joints | kp | kd | Max force | Speed limit |
|---|---|---|---|---|
| STS3215: shoulder (swings the whole arm) | 40 N·m/rad | 1.5 N·m·s/rad | 1.9 N·m | 4.7 rad/s |
| STS3215: elbow, wrist pan, wrist pitch, wrist roll | 20 N·m/rad | 0.8 N·m·s/rad | 1.9 N·m | 4.7 rad/s |
| Fingers (each jaw) | 2000 N/m | 100 N·s/m | 80 N | 0.056 m/s |
| Lift | 100000 N/m | 2000 N·s/m | 300 N | 0.10 m/s (full travel takes 7 s; was 0.05 until 2026-11-06) |
| Wheels (speed control) | 0 | 2 N·m·s/rad | 6 N·m | 25 rad/s |
| Mecanum rollers | 0 | 0 | 0 | passive |

USD stores angular gains per degree; `drives.py` converts. Change gains only in `drives.py`.

**Ramp your commands.** Don't jump a joint target further than its speed limit allows in one step: use
`rate_limit()` from `drives.py` on every command (teleop, policies, demos). Real servos and the lead-screw
lift ramp too. It also avoids a PhysX problem: if the lift is asked to move faster than its speed limit
while the base is free, it crawls at about 5 mm/s instead (found with the demo; the arm joints are not affected).

## Testing each joint

```
<isaac-sim>/python.sh scripts/isaac_joint_test.py                          # every joint, base fixed
<isaac-sim>/python.sh scripts/isaac_joint_test.py --joint left_wrist --slow  # just one arm's wrist, slowly
<isaac-sim>/python.sh scripts/isaac_joint_test.py --wheels                  # also spin the wheels (base free)
<isaac-sim>/python.sh scripts/isaac_joint_test.py --hold                    # hold 3 poses for 30 s each
<isaac-sim>/python.sh scripts/isaac_joint_test.py --usd mobile_manipulator.usd   # the same tests on a saved USD
<isaac-sim>/python.sh scripts/isaac_base_test.py                            # drive the mecanum base
<isaac-sim>/python.sh scripts/isaac_grasp_test.py                           # pick, lift, swing and place a cube
<isaac-sim>/python.sh scripts/isaac_shelf_task.py                           # shelf A -> tote -> shelf B (about 2 min)
<isaac-sim>/python.sh scripts/isaac_table_task.py                           # prototype: table A -> tote -> table B, both arms
python scripts/reach_test.py                                                # no Isaac: reach maps for tables and shelves
```

### Prototype tasks (2026-11-09, arm links 130 / 130, C-Beam XLarge lift)

Both run on the prototype layout. The left arm does the shelf work; in the table task each arm works its own half.
The arm that waits parks straight out to its side with the hand level. Both arms travel stowed.

| Task | Result (Isaac, headless, self-collision on) |
|---|---|
| `isaac_shelf_task.py` (shelf A 0.70 m -> tote -> shelf B 0.50 m) | **6/6 OK in 66 s** (was 90 s): item flat in the tote, placed on shelf B, upright |
| `isaac_table_task.py` (table A -> tote -> table B, 0.75 m, both arms at once) | **10/10 OK in 76 s** (was 93 s; 0.12 m vertical approach with the 0.885 m lift) |
| same with `--sequential` (one arm at a time) | 10/10 OK in 174 s (before the speed-up; not rerun) |

Since 2026-11-11 both use the faster moves the mono tasks use: full allowed speed after a 0.3 s ramp (instead of a
cosine profile), and a move ends once the joints stop (holding a product the wrist pitch sags a little and never got
within the old 2° tolerance). Same paths and speed limits as before.

Re-planned for the shorter links: the stow pose is now hand level (30°, 105°, −120°), inside the footprint and
clear at any lift; the shelf item sits at (0.66, 0.22) m in the robot frame; the shelf task uses a tote spot with
the shelf's elbow bend (an elbow flip while carrying slid the lying item 52 mm down the jaws).

Speed (2026-11-06): lift 0.10 m/s (was 0.05), straight hand moves timed by distance (0.20 m/s, never faster
than the lift allows), jaw waits 0.6–0.8 s, 0.1 s settle. The shelf task went from 123 s to 91 s, the single-table
task from 236 s to 136 s.
Both arms at once (default in the table task): the two products must be at the same height (one shared lift).
The arms unfold from stow one at a time (left, then right; folding the other way round) and use the tote spots
`TOTE2` (0.26 m apart); 48 two-arm moves were checked offline with arm-against-arm collisions.
The robot stops with the table edge 0.36 m from its centre (0.10 m past the bumper); the products sit 6 cm in from
the edge because the hand-down reach ends about 0.17 m past it.

How the moves were made safe:
- Every joint-space move is **time-synchronised** (all joints start and stop together), so the arm sweeps the
  straight joint-space line that was checked offline with the collision meshes, the other arm, and the held product.
- **Stow:** the fold sits next to the carriage; the only clear way in or out is the elbow alone (stow <-> "elbow open").
- **Tables:** raise the lift before swinging over the table and lower it only after swinging away. A move that
  lowers while it swings drags the product into the table (this happened in the first Isaac run).
- **Lift travel:** with 0.885 m of travel a hand-down pick works on tables up to about 1.0 m (0.12 m approach on
  0.75 m), and a level grasp reaches shelf boards from about 0.38 to 1.27 m.

### Reach test (`reach_test.py`, prototype, 2026-11-09: links 130 / 130, lift 0.885 m)

Table edge 0.30 m and shelf edge 0.55 m in front of the base centre; 3 cm grid; the other arm parked.

| Surface | Reachable (either arm) | Deepest past the edge | Width at 0.10 m past the edge |
|---|---|---|---|
| Table 0.45 / 0.60 / 0.75 / 0.90 m, hand down | 0.095 m² each | 0.18 m | 0.54 m |
| Shelf boards 0.70 and 1.10 m, hand level | 0.058 m² | 0.12 m | 0.36 m |
| Shelf boards 0.30 m and 1.40 m | none (lift travel) | | |

With the old 180 / 150 links: tables 0.159 m², 0.24 m deep; shelf 0.099 m², 0.18 m deep (and no 0.90 m table or
1.10 m shelf, with the 0.70 m lift).

**Each arm reaches only its own half in front** (left arm y ≈ 0.05…0.48, right arm mirrored); no spot in front is
reachable by both arms, and a strip about 10 cm wide in the middle by neither. The shoulder's ±115° range around
its 135° mount heading leaves its blind sector pointing forward and inward, toward the column. Two-arm work in
front (hand-overs, holding one item with both hands) would need a different mount angle; over the tote both arms
overlap.

Add `--self_collision` to any of them (and to `isaac_import_and_test.py`) to make the robot's own parts collide.
The grasp test has it on by default (`--no_self_collision` turns it off).

### Self-collision

Off by default in the import, as before. `--self_collision` turns it on with the same rules as
`generator/urdf_selfcol.py`: parts joined by fixed joints count as one body, a body never collides with
itself or with the body it is jointed to, and mecanum rollers ignore each other (822 ignored pairs).
Everything else collides: arm against arm, arm against column, carriage or tote, and one jaw against the other.
Use it for teleop and data collection, so a policy can't learn to move one arm through the other.

With self-collision on, the joint sweep (all other joints at 0) stops some pan joints early. That is the two
arms hitting each other, not an error. With both arms pointing forward, 230 mm apart, the first contact comes
early: about -9° (shoulder), -21° (elbow) and -39° (wrist pan), where the moving arm's jaw touches the other
hand (CAD check). In the Isaac sweep the moving arm then pushes the other arm aside (its servos only hold
1.9 N·m) and stops at about -65° / -42° / -79°. When the other arm is out of the way, the full range is
available.

The script moves one joint at a time, from 0 to its lower limit, then to its upper limit, then back to 0. Every other joint holds at 0 while it does. For each joint it prints:
- the range the joint actually reached
- the worst tracking error
- whether the joint returned to 0
- a status: `OK`, `SHORT OF LIMIT`, `DOES NOT RETURN`, `LAGS (tune gains)`, `LEFT FINGER DOES NOT FOLLOW` (the left jaw isn't tracking its command) or `BLEW UP`

The results are also saved to `joint_test_report.csv`. Moves are slowed down if needed so they stay under each
joint's speed limit (the lift needs about 20 s to go from 0 to 0.885 m and back).

`--hold` holds three poses (arms forward; lift at the top with the jaws open; arms folded) for 30 s each, under
gravity. It reports each joint's error from its target and its drift (how far it moved after the first second).
It passes when the drift is under 1° (1 mm for the lift).

`isaac_base_test.py` drives the base forward, backward, left, right and turning on the spot. It reports the
speed reached compared with the command, any unwanted motion in other directions, the bounce in height and the
tilt. Wheel speeds come from standard mecanum mixing (positive wheel speed rolls forward; `R` = 0.05 m,
`LX` = 0.17 m, `LY` = 0.222 m):

```
fl = (vx - vy - (LX+LY)·wz) / R      fr = (vx + vy + (LX+LY)·wz) / R
rl = (vx + vy - (LX+LY)·wz) / R      rr = (vx - vy + (LX+LY)·wz) / R
```

## Mono-arm experiment (`prototype_mono_arm`)

An experiment next to the prototype, not a replacement: **one arm, whose shoulder is a ring bearing around the lift
column**, so the arm can swing all the way round the robot (front, both sides, the tote behind). The rest of the arm
(elbow to gripper) is the same CAD as the prototype's. The prototype URDFs are not changed by it.

| | Prototype | Mono arm |
|---|---|---|
| Arms | 2, beside the column, zero pose 45° toward the tote | 1, shoulder ring around the column, zero pose forward |
| Shoulder | STS3215 in the arm base, ±115° | ring on a printed two-row ball bearing (2 × 57 steel balls, 6 mm) driven by an STS3215 through a 72 / 18 gear (4:1): 7.6 N·m, ±180° (cables in a reversing loop, no slip ring; needs the servo's multi-turn mode) |
| Elbow | ±125° | ±117° (the forearm meets the ring at 121°) |
| Lift travel | 0–0.885 m | 0–0.77 m (the ring can't pass the column's top cap) |
| Scene camera | carriage bracket, faces forward | carriage, faces forward: a post in a socket on the fixed clamp ring, in front of the beam, and a bar over the arm |

![mono-arm robot](preview_prototype_mono.png)

**The ring** is the **draft** printable design in `../CAD/mono_ring_draft/` (a printed two-row ball bearing with 6 mm steel balls, 4 : 1
gear, drive servo under the disc at the front left, a reversing-loop cable channel, a homing magnet). The URDF uses
those parts directly; only the lift's gantry plate it bolts to is estimated.

**Reach** (`reach_test.py --layout prototype_mono_arm`, hand down on tables / level into shelves):

| | Prototype, both arms together | Mono, one arm |
|---|---|---|
| Tables 0.45–0.90 m | 0.095 m², two separate patches (y ±0.42) with a dead strip in the middle | 0.070 m², one patch (y ±0.25) **including the middle** |
| Shelf boards 0.70 / 1.10 m | 0.058 m² | 0.045 m² |
| Tote floor (hand down) | 45 % | 36 % |
| Shelf board 0.30 m | no | no |

**Run it** (from this folder, with `conda activate env_isaaclab`):

```bash
python scripts/isaac_mono_task.py               # stationary: table in front -> tote behind -> table
python scripts/isaac_mono_table_task.py         # the table-to-table task, 2 products, one arm
python scripts/isaac_mono_shelf_task.py         # the shelf task, 1 product, one arm
python scripts/isaac_joint_test.py --layout prototype_mono_arm [--hold]
python scripts/isaac_import_and_test.py --layout prototype_mono_arm
python scripts/reach_test.py --layout prototype_mono_arm
```
Add `--headless` for no window. In RViz: `ros2 launch mobile_manipulator_description display.launch.py layout:=prototype_mono_arm`.

**Isaac Sim results (2026-11-10, headless):**

| Test | Result |
|---|---|
| Joint sweep | 7/7 OK (the geared shoulder lags 1.6° at most) |
| Hold, 3 poses × 30 s | 18/18 OK, no drift |
| Stationary table → tote → table | 7/7 OK, **49 s** (2026-11-12; product 0.2 mm from target). Was 4/5: with the table edge at 0.30 m the scene camera went under the table top when the lift dropped for the tote and tipped the robot back 4°. The table now stands at 0.36 m like the other table task |
| Table-to-table, 2 products | 6/6 OK, **121 s** (products 0 mm from target). Prototype: 76 s with both arms at once |
| Shelf, 1 product | 6/6 OK, **71 s** (0 mm from target). Prototype: 66 s |

The mono tasks use a faster move profile (full allowed speed after a 0.3 s ramp) and move the lift during the swings;
the two-arm tasks use the same profile since 2026-11-11. A move also counts as finished once the joints have stopped:
holding a product, the wrist pitch sags 2–3° and never gets within the normal 2° tolerance, so before this fix every
move with a product waited out an 8 s timeout (33 s of the shelf task). About a third of the time is the lift (0.10 m/s, set to
match the real lift) going 0.6 m down into the tote and back for every product.

**Things learned:**
- Isaac Sim starts the robot in its zero pose: with this arm pointing forward and low, its hand is inside a table
  placed in front of it, and the overlap throws the robot. The tasks place the table only after the arm is up.
- Move into shelves slowly (2.5 s for 80 mm): at normal speed the elbow and wrist servos lag about 8 mm sideways and an
  open fingertip knocks the product into the shelf. Open the jaws 35–40 mm for a 45 mm product.
- Coming back from a table, swing away first and lower afterwards; going to the tote, lowering during the swing is fine.

## ROS 2 (Jazzy, Ubuntu 24.04)

Use a terminal **without conda** (`conda deactivate`): ROS 2 uses the system Python 3.12. Build once:

```
mkdir -p ~/ros2_ws/src && ln -s <path>/mobile_manipulator_description ~/ros2_ws/src/
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

Then, in each new terminal:

```
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch mobile_manipulator_description display.launch.py     # RViz + a slider for every joint
python3 scripts/ros_joint_check.py                          # automated check (don't run it with the launch file open)
```

`ros_joint_check.py` checks that the joint limits match the table below, that both TCPs sit where `scara_ik.py`
puts them at the zero pose, and that every joint moves the right way (in each arm's own mount frame).
Rebuild the workspace (`colcon build --symlink-install`) whenever URDF files are added or renamed. It prints `0 failures` when everything is right.

The warning `KDL does not support a root link with an inertia` is harmless: `base_link` has a tiny 1 g mass so
that Isaac Sim doesn't give it 1 kg.

## Joints (71: 55 that move and 16 fixed frames)

| Joint | Type | Range | Drive |
|---|---|---|---|
| `wheel_fl/fr/rl/rr_joint` | continuous | – | velocity |
| `wheel_*_roller_0..8_joint` (36) | continuous | – | **passive** (no drive). These make the mecanum wheels slide sideways. |
| `lift` | prismatic, Z | 0 – 0.885 m | position, 300 N (real C-Beam XL: 115 N max; load ~55 N) |
| `left_/right_shoulder_pan` | revolute, Z | ±115° | STS3215, 1.9 N·m |
| `left_/right_elbow_pan` | revolute, Z | ±125° | STS3215, 1.9 N·m |
| `left_/right_wrist_pan` | revolute, Z | ±122° | STS3215, 1.9 N·m |
| `left_/right_wrist_pitch` | revolute, Y | -15° (up) … +115° (down) | STS3215, 1.9 N·m (see the payload note below) |
| `left_/right_wrist_roll` | revolute, X | ±120° (cable loop; was ±170°) | STS3215, 1.9 N·m |
| `left_/right_finger_right` | prismatic, +Y | 0 – 45 mm per finger (0 – 90 mm opening) | 80 N |
| `left_/right_finger_left` | prismatic, -Y | 0 – 45 mm | 80 N. Always command it to the same value as `finger_right`. |

The arm limits are the collision-free ranges measured on the CAD, minus a 5° margin.

**Payload (wrist pitch).** Every joint uses the STS3215, the same servo as the SO-100, so servos can move between the two
arms. Only the wrist pitch lifts against gravity. With the wrist level it needs 0.64 N·m for the hand alone, 1.16 N·m
with 0.25 kg in the gripper and 1.69 N·m with 0.5 kg, against 1.9 N·m available. Plan for items up to about
0.25–0.3 kg; a 1 kg item (2.75 N·m) is more than the servo can hold with the wrist level.

At zero, each arm points along its zero heading (±135°, toward the rear and past the base sides), the gripper is level, and the fingers are closed.

**Fixed frames you can use:**
- `left_/right_tcp`: between the fingertips, 12 mm in from the tip.
- `left_/right_wrist_camera` and `*_wrist_camera_optical`: the 32 × 32 mm OV camera on each wrist (frame at the default 35° tilt; the real tilt is set by hand, 0–60°).
- `scene_camera` and `scene_camera_optical`: the scene camera (on a carriage bracket in front of the column). The optical frames follow ROS conventions: z forward, x right, y down.

## Masses (per arm: 2.09 kg in the URDF, a conservative estimate)

| Link | Two-arm prototype (each arm) | One-arm experiment (shoulder ring) |
|---|---|---|
| `<side>_base` | 382 g | 512 g (ring base, drive servo, board) |
| `<side>_upper_arm` | 389 g | 482 g (with the ring rotor and the collar round it) |
| `<side>_forearm` | 389 g | 389 g |
| `<side>_wrist` | 369 g | 369 g |
| `<side>_tilt` | 251 g | 251 g |
| `<side>_hand` | 201 g | 201 g |
| `<side>_finger_right` | 58 g | 58 g |
| `<side>_finger_left` | 58 g | 58 g |
| **Total** | **2,094 g** | **2,318 g** |

About 1.27 kg of each prototype arm is printed PLA (1,627 cm³), 330 g is the six servos, and the rest is bearings,
rail, camera and fasteners. The same numbers per link are in `arm_link_masses.json`.

How the masses were worked out:
- **Printed parts:** PLA at 0.781 g/cm³ effective, scaling the former 0.80 PETG estimate by the filament density ratio 1.24 / 1.27. This preserves the old effective-fill assumption (about 60 %).
- **Purchased parts:**
  - STS3215 55 g (all six servos per arm)
  - 6808 bearing 25 g
  - MGN9 rail 65 g, MGN9H block 26 g
  - camera 18 g
- **Fasteners:** every screw and insert, in steel or brass.
- **Inertias:** worked out from the actual shapes.
- **Print-plan comparison:** the 0.781 g/cm³ effective density assumes about 60 % fill. The parts are printed with 4 walls and 15 % cubic infill, so the real printed parts will be lighter (probably 10–25 %). Weigh them and set `RHO_PRINT` in `generator/urdf_build.py`; domain randomization covers the gap until then.
- **Base:** the mobile base (19 kg), column, carriage and tote are estimates. Replace them once you weigh the real parts.

## Servo sockets (same as the SO-100)

Every servo is an STS3215 held exactly the way the SO-ARM100 holds it, so a servo moves between the SO-100 and this arm
with its own screws. Measured from the SO-100 leader print files (`../CAD/SCARA_arm/SO100/`):

- The pocket is line-to-line with the servo case: the horn face sits flush, the idler side has 0.1 mm play.
- Case screws are **M2×6 self-tapping** (the SO-100 ones) through a 2.0 mm wall into the servo's case holes:
  Ø2.0 hole, Ø4.0 head counterbore.
- Horn: 4 × M3 on the 9.9 mm square, as on the SO-100.

| Servo | Case holes used (servo frame) | Fit them |
|---|---|---|
| Shoulder, elbow, wrist pan, pitch | horn face (−32.8, ±10.25) + idler face (−29.0, ±10.25), as on the SO-100 arm joints | before the next link goes on: its fork covers them |
| Wrist roll | idler face (−29.0, ±10.25) and (−8.3, ±10.25), through the tilt-fork base | before the tilt fork goes onto the pitch servo |
| Gripper | idler face (−29.0, ±10.25), from behind through the yoke's back wall | any time: turn the roll 90° or 180° so the screwdriver clears the roll servo |

The generator code is `SO100` / `case_screw()` in `generator/joint.py`.

## Printing (Bambu Lab A1, PLA Pro+)

The files in `../CAD/SCARA_arm/print_ready_stl/` are already in the print orientation. Print 2 of each part for the two arms.

- **Start with a fit test:** print `01_base_top` and `02_base_bottom` first. They are small and have the same servo socket
  as the other joints. Check that a servo goes in and the M2 screws line up before you print the long links.
- **Plate:** every part fits the 256 × 256 mm bed. The upper-arm halves are 254 mm long: rotate them 45° on the plate.
- **Settings:** 0.4 mm nozzle, 0.20 mm layers (0.12 mm for `17_pinion_24T_m1`), 4 walls, 5 top / 5 bottom layers,
  15 % cubic infill (like the SO-100). Use the temperatures on your PLA Pro+ spool. Full notes: `../CAD/SCARA_arm/README.md`.
- **Supports:** tree (auto), threshold 30°, *not* "build plate only". A few small ceilings (cable slots) need support
  that starts on the part. Clean the servo-pocket ceilings well: the servo face sits on them.
- **Keep the fit exact:** elephant-foot compensation 0.15 mm (the Bambu default), because the pocket openings are on
  the bed face. Leave X-Y hole and contour compensation at 0, so the holes stay Ø2.0 / Ø4.0 as on the SO-100.
- **Heat-set inserts (M3, M5):** press them in with the iron at about 200 °C for PLA.

## Checks done

- **Structure:** the URDF parses, it forms a single tree from `base_link`, every mesh file exists, and every inertia is positive-definite and physically valid.
- **Independent check:** I rebuilt the robot from the URDF alone, using a separate parser and forward kinematics. The rendered poses match the CAD.
- **Collisions at rest:** at the home pose and in a test pose, no collision bodies overlap except joints that are directly connected. Those are ignored by default.
- **ROS 2 Jazzy** (`ros_joint_check.py`, 2026-11-03): 32/32 checks OK. All 15 limits match, both TCPs are at (0.700, ±0.20, 0.411) m at zero, and every joint moves the right way.

### Isaac Sim 5.1 results, prototype layout (2026-11-06, headless)

- **Joint sweep:** 13/13 OK (shoulder max tracking error 9.1°, every joint ends within 1.8°; lift reaches 699.5 mm).
- **Hold** (30 s per pose): 33/33 OK, drift 0. Wrist pitch sags 1.8° with the gripper level.
- **Grasp** (self-collision on): 150 g and 250 g cubes OK on both arms (slip ≤ 0.1 mm, placed within 1.1 mm).
- **Mecanum base:** 6/6 motions OK (91–96 % of the commanded speed, bounce ≤ 0.4 mm, tilt ≤ 0.07°).
- **ROS 2:** `ros_joint_check.py` 0 failures; `check_urdf` parses the URDF.
- Not run on the prototype yet: USD save / reload (`--save`, `--usd`).

### Isaac Sim 5.1 results with STS3215 on every joint (2026-11-06, headless, on the old v0 layout, deleted 2026-11-09)

- **Joint sweep:** 13/13 OK. Shoulder max tracking error 9.0° (with kp 20 it was 14°: LAGS), end error 0.2°.
- **Hold** (30 s per pose): 33/33 OK, drift 0. Wrist pitch sags 1.8° with the gripper level (was 0.9° on the STS3250).
- **Grasp:** 150 g and 250 g cubes OK (slip 0.1 mm). **1 kg FAILS:** the wrist pitch can't hold it (the cube rose
  54 mm for 200 mm of lift and slipped). This matches the payload note above.
- Base, USD and self-collision runs below are from 2026-11-03 and were not repeated; the servo change does not touch them.

### Isaac Sim 5.1 results (2026-11-03, headless, RTX 5060 Ti, STS3250 on shoulder and pitch)

- **Joint sweep** (`isaac_joint_test.py`): 13/13 OK with the base fixed, with the base free (`--wheels`), and on a
  saved USD (`--usd`). Every arm joint reaches its limits within 0.8°. Worst tracking error during the moves
  is 7.4° (elbow), and every joint returns to 0 within 0.9°. Lift reaches 699.3 of 700 mm. Both jaws reach 45 mm,
  and the left jaw follows the right exactly.
- **Hold** (`--hold`, 30 s per pose): 33/33 OK, on the URDF and on the saved USD. Drift is under 0.01° everywhere.
  The largest steady error is wrist pitch at 0.9° with the gripper level (gravity against kp 40).
- **Mecanum base** (`isaac_base_test.py`, 0.3 m/s and 0.6 rad/s): 6/6 OK. The base reaches 89–97 % of the
  commanded speed, with under 0.02 m/s of unwanted motion, under 0.5 mm of bounce and under 0.1° of tilt.
- **USD** saved with `--save`: one self-contained file (about 47 MB, 821 collision shapes, drives included).
- **Self-collision on** (`--self_collision`): nothing jitters or blows up. Hold 33/33 OK, mecanum 6/6 OK, and the
  closed jaws track while touching (within 0.1 mm). The joint sweep stops the pan joints where the arms meet
  (see above); wrist pitch, wrist roll, jaws and lift are unaffected. A USD saved with it on keeps it.
- **Grasp** (`isaac_grasp_test.py`, default friction 0.5): both arms OK with a 60 mm, 150 g cube, a 40 mm cube and
  a 1 kg cube. Slip in the gripper is 0.2 mm or less, and every cube is put back within 1 mm. The grip force is
  about 58 N per jaw on the 60 mm cube: the jaw drive (2000 N/m) times how far the closed target is beyond the
  cube. The real gripper (STS3215 with a 12 mm pinion driving two racks) gives up to about 79 N per jaw at stall,
  which matches the 80 N cap. With 1 kg the cube rises 189 mm for 200 mm of lift: the arm, mostly the wrist pitch,
  sags under the load; the grip doesn't slip.

## Known simplifications

- **Gripper pinion:** the pinion is shown fixed to the hand. The fingers move on their own prismatic joints, which is how the real rack and pinion behaves.
- **No mimic joint.** Both jaws are separate driven joints, and you send them the same target. A `<mimic>` joint on top of a position drive makes the jaws fight, and in Isaac Sim one of them can fly off. Franka's gripper in Isaac Sim is set up this way for the same reason.
- **Mecanum wheels:** they use 9 passive rollers per wheel, each a chain of 3 spheres. In the test above they drive smoothly. If you ever need to skip wheel physics (for example, navigation-only training), a holonomic base controller acting on `base_link` is fine, as long as the commands and observations use the same `vx, vy, wz` convention as the wheel mixing above.
- **Servo gains:** the values in `scripts/drives.py` pass every test, but they are not measured from the servos.
- **Joint friction:** the URDF `friction` values are meant as N·m (or N), but Isaac Sim reads them as a unitless PhysX friction coefficient. Setting them all to 0 changed the test results very little, so they are left as they are.
- **Frame links** (`*_tcp`, `*_wrist_camera`, `*_wrist_camera_optical`, `scene_camera_optical`, `base_link`) have a tiny 1 g mass. Without an explicit mass, Isaac Sim gives each one 1 kg, which put 3 kg of phantom mass on each hand.
