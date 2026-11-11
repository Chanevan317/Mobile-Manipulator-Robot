# Mono-arm shoulder ring: DRAFT

**Status: draft, not ready to print as a set.** The mechanism is worked out and checked for clashes. The lift's real
mounting plate hasn't been measured yet, so everything that touches it is an estimate (see [Estimates](#estimates)).

This is the shoulder for the **mono-arm experiment**: one arm whose shoulder turns *around* the C-Beam lift column
instead of beside it. It replaces the prototype's base parts (`01_base_top`, `02_base_bottom`) and the upper arm's
shoulder fork. Everything from the elbow outward (forearm, wrist, pitch, roll, gripper) is the same as the prototype's.

![preview](preview.png)

## How it works

```
        top cap (fixed) ──────────────┐   camera post socket, inner wall of the cable channel
   ┌── rotor top ring ────┐ ● row B   │   57 steel balls (6 mm) in grooves of the hub and the rotor
   │   rotor middle ring  │           ├── the rotor turns with the arm; the upper arm sits on the middle ring's lug
   └── rotor bottom ring ─┘ ● row A   │   57 more balls; the bottom ring carries the 72 T gear
        hub (fixed) = inner race ─────┘   part of the ring base; the beam passes through its 44 x 84 hole
        ring base disc (fixed) ── rests on the gantry plate's top edge, held by the L bracket behind it
   gear: 72 T on the rotor  <──>  18 T pinion on an STS3215 hanging under the drive mount  (4 : 1)
   cable channel on top: fixed inner wall (clamp ring) + moving outer wall (cable tray on the rotor)
```

1. **The fixed side** is the ring base: a disc sitting on top of the lift's gantry plate, with a hub tube that goes up
   around the beam. The L bracket bolts the disc to the back of the gantry plate.
2. **A printed two-row ball bearing (no bought bearing).** The hub is the inner race: two grooves, 20 mm apart.
   The rotor is the outer race, printed as **three rings** split through the middle of each groove. Two rows of 57
   steel balls (6 mm) run between them. Two rows spaced apart resist the arm's tipping load much better than one.
3. **Preload:** the rings are printed 0.15 mm apart at each split. Eight M3 bolts pull the stack together, which squeezes
   the balls into their grooves (each ball touches three points) and takes out the play. The grooves are 6.2 mm for
   6.0 mm balls (0.2 mm of play before tightening). Expect about 0.1–0.2 mm of play left after a careful assembly:
   at full reach that's 2–3 mm at the hand.
4. **The drive:** the 72-tooth gear (module 2) is part of the rotor. An STS3215 hangs under the drive mount, output
   up, with an 18-tooth pinion: 4 : 1, so about 7.6 N·m at stall and 1.2 rad/s. The servo body points **in toward the
   column**, under the disc: nothing fixed sticks out past 110 mm from the axis. With the body pointing out (to 128 mm),
   the hanging wrist hit the cradle when the elbow folded past 110°.
5. **Cables cross the joint in a reversing loop.** The drive servo and the control board stay on the carriage, so only
   two cables cross the turning joint: the **servo bus** (to the elbow servo and on down the arm) and the **wrist
   camera's USB**. They come up through a hole in the hub, lie in a channel on top of the rotor, run along the fixed
   inner wall, make a U-turn, and come back along the moving outer wall, which carries them straight into the end of
   the arm tube. As the arm turns, the U-turn travels round the channel at half the speed, so nothing is pulled or
   twisted. Range **±195°**, with about 0.25 m of each cable inside the channel; your 900 mm cables are long enough.
   The two cables lie on top of each other (bus at the bottom, USB on top).
6. **Homing:** a 5 × 3 mm magnet in the rotor's inner wall (at the arm's heading) passes a hall sensor in the rim of
   the ring base, 1 mm away. With the 4 : 1 gear the servo needs several turns for the full range, and after power-up
   it only knows its angle within one turn; one pass over the magnet tells which turn it's on. Home = arm pointing
   forward.
7. **The upper arm** is the prototype's upper arm with its shoulder fork cut off (from 70 mm out). Its tube sits in a
   saddle on the rotor's lug, with 4 × M3 from inside the tube. A new bolt pair at 76 mm holds the two halves together
   at that end, where the fork's bolts used to be.

**Why everything is so low:** the forearm's lower fork prong hangs 33.6 mm under the arm, and when the elbow passes
over a spot it sweeps everything between 92 and 168 mm from the column. So everything fixed out there stays under the
pinion's top (30.5 mm on the carriage scale; the prong is at 31.9).

## Parts

| # | File | What | Turns? | Print |
|---|---|---|---|---|
| M01 | `M01_mono_upper_arm_bottom.stl` | Upper arm bottom half, fork cut off | yes | split face down (as the prototype) |
| M02 | `M02_mono_upper_arm_top.stl` | Upper arm top half, fork cut off | yes | split face down |
| M03 | `M03_ring_base.stl` | Disc + hub = inner race (2 ball grooves) | no | disc down |
| M04 | `M04_clamp_ring.stl` | Top cap: camera-post socket, inner wall of the cable channel | no | flat |
| M05 | `M05_rotor_bottom.stl` | Outer race (lower half of row A) + 72 T gear + homing magnet | yes | gear down |
| M06 | `M06_rotor_middle.stl` | Outer race (rows A / B) + lug under the upper arm | yes | upside down; **supports under the lug only** |
| M07 | `M07_rotor_top.stl` | Outer race (upper half of row B) | yes | upside down (groove up) |
| M08 | `M08_cable_tray.stl` | Moving outer wall + floor of the cable channel | yes | floor down |
| M09 | `M09_drive_mount.stl` | Ear + cradle for the drive servo | no | ear down |
| M10 | `M10_drive_strap.stl` | Holds the servo in the cradle from below | no | flat |
| M11 | `M11_ring_pinion.stl` | 18 T pinion on the servo horn | no (spins) | gear face down |
| M12 | `M12_gantry_bracket.stl` | L bracket to the gantry plate | no | on its side |
| M13 | `M13_board_mount.stl` | Holds the Waveshare servo board (on your SO-101 plate) under the ring base | no | on its back |

Sizes and volumes are in `printed_parts.csv`; screws and inserts in `fasteners.csv`.
Print like the rest of the arm (0.2 mm, 4 walls, 15 % cubic), except: **6 walls** for the ring base and the three
rotor rings (the ball grooves and gear teeth must be solid plastic) and for the pinion. PETG wears better than PLA
in the ball grooves if you have it; PLA works for a prototype.

## To buy

| Item | Qty | Notes |
|---|---|---|
| Steel balls 6 mm (chrome steel, e.g. bicycle bearing balls) | 120 | 114 in the races + spares; airsoft BBs work but wear |
| Lithium grease | a little | for the ball grooves |
| STS3215 (7.4 V) | 1 | the shoulder servo moves here from the prototype base |
| Neodymium magnet 5 × 3 mm | 1 | homing, glued into the rotor |
| Hall sensor (SS49E analog or A3144 switch, TO-92) | 1 | homing, in the ring base rim; wires down through the disc |
| Flexible USB cable for the wrist camera | 1 | high-flex type if you can find it: it bends in the loop |
| Screws, inserts, nuts | see `fasteners.csv` | M2 / M3 / M5, from the same kit as the arm |

## Wiring

Everything electrical stays on the **carriage** except the arm itself:

```
 base ── drag chain up the lift: power pair (18 AWG) + one USB cable ──┐
                                                                       ▼
 carriage:  Waveshare board (on M13, front right under the ring base) + small USB hub
              ├─ servo bus ─> ring drive servo (front left, under the disc)
              ├─ servo bus ─> up the hub hole ─> cable loop ─> elbow servo ─> wrist pan ─> pitch ─> roll ─> gripper
              └─ USB hub ─> scene camera (on the top cap)  +  wrist camera USB (back down through the cable loop)
                           + homing hall sensor via a small microcontroller (Pico / ESP32: the board has no input for it)
```

- **Why the board rides the lift:** a servo cable running ~1.5 m up the lift would lose about 1.5 V of the 7.4 V when the
  arm moves (its wires are thin), and the bus data would share it with the motor current. Thick power wires and USB
  carry that distance easily, and the servo bus stays short.
- The board slides into the M13 dovetail from the right side, facing forward: plugs reachable from the front, and
  the bus cable to the hub hole is a few centimetres. Tie the drag-chain cables to the two slots under the board.
- Nothing here reaches past 85 mm from the column, so the hanging wrist and hand can't hit it (checked at every lift).

## Assembly (draft)

1. Heat-set the inserts (M03 hub top and disc underside, M05 top face, M06 lug top and tray holes, M09 ear,
   M02 at the 76 mm bolts). Glue the magnet into M05's inner wall and the hall sensor into the ring base rim (front).
2. Grease both hub grooves. Slide the **bottom rotor ring** (M05) down over the hub, hold it about 1 mm above the
   disc (two 1 mm shims), and drop **57 balls** into row A, all the way round. They sit in the hub groove and the ring's
   half-groove.
3. Put the **middle ring** (M06) on, lug toward the arm's heading. Drop **57 balls** into row B. Put the **top ring**
   (M07) on.
4. Insert the 8 × M3 × 25 stack bolts and tighten them a little at a time, crosswise, turning the rotor as you go.
   **Stop as soon as the tilt play is gone and it still turns smoothly** (the 0.15 mm gaps may not fully close).
   Pull out the shims. Screw on the top cap (4 × M3).
5. Lay the two cables: up through the hub hole, out through the slot in the clamp ring's wall, along the inner wall,
   U-turn, and back along the outer wall to the exit facing the arm (bus at the bottom, USB on top). Check the loop
   with the arm at both ends of its range (±180°) before closing anything. Screw on the cable tray (3 × M3, from the side).
6. Put the servo in the cradle from below, close it with the strap, and screw the pinion onto the horn (4 × M3).
7. Screw the drive mount under the disc (2 × M3 in radial slots). **Set the gear backlash:** slide the mount in until
   the teeth just touch, then back it off about 0.2 mm before tightening.
8. Fit the ring base onto the gantry plate with the L bracket (positions to be checked on the real plate).
9. Bolt the upper arm's bottom half onto the rotor lug, then fit the elbow servo and the top half as on the prototype.

## Checked (`generator/mono_ring.py`)

- No clashes between the fixed parts, or with the estimated gantry plate, V-wheels and beam.
- The rotor and upper arm turn all the way round (every 10°) without touching anything fixed.
- The forearm folded at −117 … +117° clears the drive, the ring base and the clamp at every shoulder angle.
- Gear: centre distance 90.00 mm = ideal (module 2, 72 + 18 teeth).
- The balls touch nothing but their grooves (0.1 mm clearance each side before preload); the three rotor rings
  don't overlap (0.15 mm preload gap at each split).
- The cable tray clears the upper arm, the rotor and the balls, and turns with the arm without touching the fixed
  inner wall. Channel 10 mm wide × 12 mm tall.

## Estimates

- **Gantry plate:** 125 × 125 × 6 behind the beam, top level with the disc's underside, V-wheels between it and the
  beam. The L bracket's 4 × M5 holes into it are **guessed** (y ±20 mm, 8 and 45 mm below the top): drill or move them
  to match the real plate.
- **Lead-screw nut block:** assumed to stay below the gantry plate's top.
- **Gear strength:** at full stall (7.6 N·m) the teeth see about 25 MPa, which PLA takes but not for long. Limit the
  servo's torque in its settings; swinging the arm needs far less.

## Bench test first: can the servo do several turns?

The 4 : 1 gear means the servo turns 4 times for one turn of the shoulder. In its normal mode an STS3215 covers only
one turn, which would give just **90° of shoulder**. A ±180° shoulder needs the servo's **multi-turn mode** (both
angle limits set to 0). Test it on one servo before building the ring.

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
   cd ~/Documents/Mobile-Manipulator-Robot/SCARA_arm_CAD/mono_ring_draft/bench
   python servo_multiturn_test.py --port /dev/ttyACM0 --id 1
   ```
4. Follow the prompts:
   - The servo turns **2 full turns forward**. Count them on the flag and answer y/n.
   - It turns **3 full turns back** (ending 1 turn before where it started). Count and answer.
   - It asks you to **unplug the servo power** (keep USB), wait 3 s, plug it back in, and press Enter.
5. Read the verdict and paste the printed results here.

The script sets the angle limits back to what they were at the end, even if you stop it with Ctrl+C. If it ever prints
"could not restore", tell me the numbers it shows.

**What the results mean:**

| Result | What it means for the ring |
|---|---|
| Multi-turn works and keeps the turn count after power-off | Best case: ±180° and no homing needed |
| Multi-turn works, turn count lost after power-off (most likely) | ±180° works; the ring needs a homing mark (magnet + hall sensor) to find its turn at start-up |
| Multi-turn doesn't work | The ring is limited to 90° with this servo: go back to the side-mounted shoulder |

## Not done yet

- The mono URDF uses these parts since 2026-11-11 (fixed parts on `left_base`, rotor / tray / upper arm on
  `left_upper_arm`). The Isaac tasks haven't been rerun with them yet.
- The reversing loop is checked only on paper: a floppy round cable can buckle when it's pushed. Try it with your
  real cables first. If it buckles, a thin plastic strip taped along the cables (or a flat cable) stiffens the loop.
- No hard stop: the range is kept by software limits after homing. A mechanical stop for more than one turn needs a
  floating stop ring; add it if the software limits ever fail.
- The scene-camera post and bar (the clamp ring has the socket for it).
- Regenerate everything: `cd mobile_manipulator_description/generator && python3 mono_ring.py ../../SCARA_arm_CAD/mono_ring_draft`
