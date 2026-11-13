# Mono-arm shoulder ring

The one-arm robot's shoulder ring on the real C-Beam XLarge lift (layout B, chosen 2026-11-13). It replaces the earlier
draft with printed ball races and a collar on the arm; that draft is in git history.

- **Your own arm base sits on the ring.** Parts **01 base_top + 02 base_bottom** (with the shoulder servo, exactly as
  printed for the two-arm robot) bolt onto a **shelf** on the ring with their 4 × M5 inserts. The ring turns the whole
  arm around the column; the arm keeps its own shoulder joint, then **one 130 mm link**, then the wrist. The forearm is
  not used. (Layout study: more reach than the old mono arm, tables 0.095 vs 0.070 m², tote 43 vs 36 %.)
- **Two bought 6820-2RS / 61820-2RS bearings** (100 × 125 × 13 mm, sealed).
- **Rotor and pinion in PETG.** Everything else PLA.

![mono ring](preview_mono_ring.png)

Made by `mobile_manipulator_description/generator/mono_ring.py` (parts, checks, these files, the preview);
`mono_ring_strength.py` checks the parts' strength with the loads from the Isaac load test.

## How it fits together (heights above the gantry plate's top)

| Height (mm) | Part |
|---|---|
| 0 … 5 | **disc** (fixed) on the gantry plate's top edge, bolted to the L bracket behind the plate |
| 6 … 19 | **bearing 1**: inner ring on the hub, outer ring in the rotor |
| 19 … 27 | **inner spacer** (fixed) between the inner rings; the rotor's **lip** between the outer rings |
| 27 … 40 | **bearing 2** |
| 40 … 44 | **clamp** (fixed): 4 × M3 into the hub pull both inner rings and the spacer down onto the disc |
| 44 … 56 | **cable channel**: fixed drum (on the clamp) inside, moving tray (on the rotor) outside |
| 1 … 40 | **rotor** (PETG, turns): gear at the bottom (72 teeth), **shelf** at 12 … 24 with two side ribs |
| 64 | the arm's plane (the base's servo mid-plane); shoulder axis 161 mm from the column |

The rotor's lip presses bearing 1's outer ring down (the arm's weight) and bearing 2's up (anything lifting the arm),
so the two bearings hold the arm's tipping load between them, 21 mm apart.

**Holding the ring on the lift.** The disc sits on the gantry plate's top edge and on the L bracket behind the plate. Four
M5 bolts tie it to the bracket: 2 from below at 50 mm behind the column axis, and **2 long M5 × 40 from the hub top**
(33 mm behind the axis, under the clamp). The load test showed why the second pair is needed: with the arm pointing
**backward**, the ring tips about the bracket's rear edge, and the first pair alone sits only 10 mm in front of it
(~470 N per bolt, about what an M5 insert in PLA holds); with both pairs it is ~155 N. The bracket's leg is 12 mm thick and
bolts through 4 of the gantry plate's own holes.

## Parts

| # | File | What | Material | Print |
|---|---|---|---|---|
| M01 | `M01_ring_base.stl` | disc + hub (seat for both inner rings) + the shoulder under bearing 1 | PLA | disc down |
| M02 | `M02_inner_spacer.stl` | ring between the two inner rings | PLA | flat |
| M03 | `M03_clamp_ring.stl` | clamps the inner rings; cable channel's inner wall; scene-camera post socket | PLA | flat |
| M04 | `M04_gantry_bracket.stl` | L bracket: gantry plate (its own holes) to the disc + hub | PLA | on its side |
| M05 | `M05_drive_mount.stl` | holds the ring-drive STS3215 under the disc | PLA | ear down |
| M06 | `M06_drive_strap.stl` | closes the servo cradle | PLA | flat |
| M07 | `M07_ring_pinion.stl` | 18-tooth pinion on the drive servo's horn | **PETG** | gear down |
| M08 | `M08_board_mount.stl` | servo-board dovetail under the disc (SO-101 Waveshare plate) | PLA | on its back |
| M09 | `M09_rotor.stl` | both outer rings' seat + lip, 72-tooth gear, shelf + ribs for the arm base | **PETG** | upright, gear down; tree supports under the shelf; 5 walls |
| M10 | `M10_cable_tray.stl` | moving outer wall + floor of the cable channel | PLA | floor down |

`printed_parts.csv` has sizes and volumes; `fasteners.csv` has every screw, insert and the two bearings.
`assembly_stl/` has every part in place, plus `context/` (bearings, drive servo, board, the kit's plate / wheels / beam,
the arm base with its servo).

**To buy:** 2 × **6820-2RS** (= 61820-2RS, 100 × 125 × 13): from a local bearing shop (the online listings were out of
stock or unclear). Check the number on the side, measure 100 / 125 / 13, spin it by hand (smooth, no grit). Then the
screws in `fasteners.csv`: 4 × M5 × 14 (base to shelf), 2 × M5 × 40 (hub bolts), 2 × M5 × 10 (bracket to disc),
4 × M5 × 25 + nylocs (bracket to plate), 4 × M3 × 10 (clamp), and the smaller ones.

## Assembly

1. Heat-set the inserts: hub top (4 × M3), disc underside (2 × M5 × 4, 2 × M3 drive mount, 3 × M3 board), bracket top
   (2 × M5 × 8 for the hub bolts), rotor top (3 × M3 tray).
2. Push the **arm base (01 + 02)** onto the shelf and screw it from underneath (4 × M5 × 14; the heads sink into the shelf).
3. Bolt the **ring base** onto the **L bracket**: 2 × M5 × 10 from below, 2 × M5 × 40 down through the hub (heads sink
   into the hub top).
4. Slide **bearing 1** down the hub onto the disc's shoulder.
5. Lower the **rotor** over it (bearing 1's outer ring goes up against the lip). Put the **inner spacer** on the hub.
6. Slide **bearing 2** down the hub into the rotor, until it sits on the spacer and against the lip.
7. **Clamp** on the hub top, 4 × M3 × 10, tightened evenly: the rotor should turn freely with no axial play.
8. Cable tray onto the rotor top (3 × M3 × 6). Drive mount, servo, strap and pinion under the disc (slide the mount in
   its slots until the teeth just touch, then back off ~0.2 mm).
9. Fit the whole ring on the beam **before the top end mount goes on** (the ring can't pass the end mount; the motor is
   at the bottom), and bolt the bracket through the plate's 4 holes beside the beam (y ±50 mm, 12.5 and 72.5 mm below
   the plate's top, nylocs on the plate's front face).
10. Upper arm, wrist, hand: the prototype's parts. The upper arm's far end takes the **wrist** directly (it is the same
    part as the forearm, so it fits).

Cables: from the board (under the disc) up through the hub hole, into the channel (reversing loop, ±195°), out of the
tray toward the arm, into the base's rear cable slot. The base's board dovetail is not used here.

## Set the drive servo's torque limit

The gear teeth are fine for everything the arm does (load test: at most 3.4 N·m on the ring), but **not** for the
drive servo stalling at its full 1.9 N·m (7.6 N·m on the ring: the PETG pinion's teeth would see ~21 MPa). Set the
ring servo's torque limit to **60 %** (its EEPROM `Max_Torque_Limit` / RAM `Torque_Limit`): 4.6 N·m on the ring is still
more than the arm needs, and the pinion then stays at ~13 MPa even when something blocks the ring. Keep a spare pinion
(9.5 cm³ of PETG).

## Servo multi-turn test (do this first)

The 4 : 1 gear means the servo turns 4 times for one turn of the ring. In its normal mode an STS3215 covers only
one turn, which would give just **90° of ring**. ±180° needs the servo's **multi-turn mode** (both angle limits set to 0).
Test it on one servo before building the ring.

**You need:** one STS3215 (any of yours), the bus adapter board, its 7.4 V supply and the USB cable. Nothing on the
horn except a small tape flag, so you can count the turns.

1. Connect **only this one servo** to the board. Power on, plug in USB.
2. Find the port and allow access (after every reboot):
   ```bash
   ls /dev/ttyACM*                  # e.g. /dev/ttyACM0
   sudo chmod 666 /dev/ttyACM0
   ```
3. Run the test (ID 1 is the default; an SO-100 servo keeps the ID of its joint, 1 to 6):
   ```bash
   conda activate lerobot_env
   cd ~/Documents/Mobile-Manipulator-Robot/CAD/mono_ring/bench
   python servo_multiturn_test.py --port /dev/ttyACM0 --id 1
   ```
4. Follow the prompts: it turns **2 full turns forward** (count them on the flag, answer y/n), **3 back**, then asks you
   to unplug the servo power (keep USB), wait 3 s, plug it back in and press Enter. Paste the printed results to me.

The script sets the angle limits back to what they were at the end, even if you stop it with Ctrl+C.

| Result | What it means for the ring |
|---|---|
| Multi-turn works and keeps the turn count after power-off | Best case: ±180° and no homing needed |
| Multi-turn works, turn count lost after power-off (most likely) | ±180° works; the ring finds its turn at start-up with the homing magnet + hall sensor |
| Multi-turn doesn't work | The ring is limited to 90° with this servo |

## Checked

`generator/mono_ring.py`:
- No clashes between the fixed parts, or with the kit's gantry plate, V-wheels and beam.
- **Bearings:** fixed parts touch only the inner rings, turning parts only the outer rings, nothing touches the
  balls / seals.
- The rotor, tray, shelf and arm base turn all the way round (every 10°) without touching anything fixed; the shelf
  passes 1.5 mm over the pinion.
- Arm sweep (110 poses × 12 ring angles): the arm only touches the ring, the drive or the beam when it is folded back
  on itself (shoulder and wrist both near the same limit, hand pointing at the column): a limit for motion planning.
- Gear centre distance 90.00 mm (exact). Hub wall at the beam hole's corners 3.9 mm. Highest clear lift 0.815 m
  (the clamp meets the lift's top end mount at 0.820; the robot model stops at 0.81).

In Isaac Sim (the robot model with this ring):
- **Joint sweep:** 7/7 OK; the lift reached 809.7 of 810 mm, the ring ±180°.
- **Hold under gravity:** 18/18 OK, no drift.
- **Load tests:** 0 / 0.25 / 0.5 kg, all moves completed.
- **Tasks:** one-arm table→tote→table 7/7 (51 s), table-to-table 6/6 (125 s), shelf task 6/6 (79 s, item 1 mm from target).

**Strength:** see [Strength](#strength) below.

## Strength

Isaac Sim's parts are rigid, so the sim can't show a printed part failing. `scripts/isaac_mono_load_test.py`
records the **loads** through the ring while the arm works with 0 / 0.25 / 0.5 kg in the gripper:
- arm stretched out, hand level and hand down;
- the ring swinging 300° at full speed, then a hard stop;
- the shoulder swinging 220° at full speed, then a hard stop;
- the lift going 0.05 → 0.75 → 0.05 m.

Worst loads (0.5 kg, all at the fastest moves):

| | Load |
|---|---|
| Through the ring bearings | tipping 9.5 N·m (lift stopping at the bottom), axial 34 N |
| At the arm base's shoulder | tipping 5.1 N·m, axial 22 N |
| Ring drive (gear) | 3.5 N·m (hard stop); the arm's own shoulder servo 2.0 N·m |

`generator/mono_ring_strength.py` puts those loads on the parts' real cross-sections, cut from the CAD. Printed
strength used: PETG 30 MPa, PLA 30 MPa, PLA 15 MPa across the layers, all already reduced for 4–5 walls and 15 %
infill. The target is a safety factor (SF) of at least 3.

| Part | What | Stress / force | SF |
|---|---|---|---|
| rotor (PETG) | shelf + ribs where they leave the rotor (worst section, r 81) | 1.6 MPa | 18 |
| ring base (PLA) | hub tube above the disc | 0.2 MPa | 91 |
| bearings | radial load per bearing (rating 19.6 kN) | 466 N | 42 |
| hub / rotor | bearing seat pressure | 0.3–0.4 MPa | > 80 |
| ring to bracket | worst bolt (arm pointing back), M5 × 4 insert ~600 N | 155 N | 3.9 |
| *(earlier mount: 2 bolts only)* | *same case* | *474 N* | *1.3* |
| L bracket (PLA) | 12 mm leg bending | 3.5 MPa | 8.5 |
| arm base | its 4 M5 × 8 inserts | 14 N | 87 |
| pinion (PETG) | tooth root, worst load in use | 9.9 MPa | 3.0 |
| rotor gear (PETG) | tooth root, worst load in use | 7.1 MPa | 4.2 |
| pinion (PETG) | ring blocked, servo stalls at its **60 % limit** | 12.8 MPa | 2.3 |
| rotor gear (PETG) | same | 9.2 MPa | 3.3 |
| pinion (PETG) | ring blocked, servo stalls with **no limit** | 21.4 MPa | 1.4 |

**What this means:**
- **Nothing printed comes close to failing in normal use.** The shelf, ribs, hub, bearing seats and bracket carry the
  loads many times over.
- **The ring's mount needed the second pair of bolts:** without them, the worst bolt would see 474 N against ~600 N for
  an M5 × 4 insert in PLA.
- **The gear sets the limit, and only if the ring is blocked.** Set the drive servo's torque limit to 60 %. If something
  ever blocks the ring, the small pinion is the part that gives (SF 2.3 against its full strength, so not on a single
  stall), and it's the cheap one to keep a spare of.
- **Insert pull-out values are typical published figures, not measured.**
- **Deflection is not computed.** The bearings' own internal play (~0.07° of tilt) may move the gripper about 0.5 mm.

## Not done / to check

- **Bench rig** for this ring (the old rig's lever clamped the old collar).
- **Fits on printed parts:** the hub is 0.05 mm under the bearing bore and the rotor bore 0.05 mm over its outside. Print a
  short slice of each first to tune the fit.
- Measure the bearings when they arrive (bore, outside, how far the inner / outer ring faces reach, seal recess): the
  clamp, spacer and lip are sized from SKF's catalogue values.
