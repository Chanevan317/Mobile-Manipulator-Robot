# Ring bench rig

Tests the **riskiest part of the mono arm, the shoulder ring**, on its own on a table, before you build the rest:

- the printed ball races: do they turn smoothly, and how much play is left after preloading?
- the gear and drive: backlash, and whether the servo can do several turns (multi-turn mode)
- homing: does the hall sensor see the magnet?
- the cable loop: do your real cables follow ±180° without buckling?
- the load: does the ring hold the arm's tipping load (about 6 N·m)?

![rig](preview_ring_bench_rig.png)

## What to print

Two new parts, plus the ring parts from `../print_ready_stl/` **except** the upper arm halves (M01, M02): the test lever
takes their place.

| # | File | What | Print |
|---|---|---|---|
| R01 | `R01_rig_foot.stl` | Lift stand-in: base plate, a stand-in for the kit's gantry plate, at its real place 1.5 mm behind the beam (the disc sits on its top edge, the L bracket bolts through its 4 holes beside the beam) and a hollow 40 × 80 beam stub | flat; 185 × 180 × 211 mm, 383 cm³ |
| R02 | `R02_test_lever.stl` | Stands in for the upper arm: the same collar round the rotor as the arm (6 radial M3); weight holes at 150 / 200 / 250 / 290 mm | upside down (top face on the bed) |

From the ring: M03 ring base, M04 top cap, M05–M07 rotor rings, M08 cable tray, M09–M11 drive mount, strap, pinion,
M12 L bracket, M13 board mount (optional, for the real cable path). Plus the 114 steel balls, the drive STS3215,
the magnet and the hall sensor.

## Set up

1. Assemble the ring as in `../README.md` (Assembly, steps 1–7): balls, rotor rings, stack bolts, top cap, drive.
2. Lower the ring base over the beam stub until the disc sits on the stand-in's top edge. Bolt the L bracket (M12) to
   the disc (2 × M5) and to the stand-in (4 × M5 + nyloc nuts on the front face). This also checks the bracket.
3. Slide the test lever's collar down over the rotor (the cable tray's tabs in its pockets) and screw it on
   (6 × M3 × 10, radial); then the cable tray through the collar's small holes (3 × M3 × 8).
4. C-clamp the foot's rear corners to a sturdy table with the lever's side (**+x**) over the edge, so weights can
   hang free.
5. Plug the drive servo into the Waveshare board (on M13, or loose on the table), board power 7.4 V, USB to the PC.

## Tests

Write the numbers down; paste them to me afterwards.

| # | Test | How | Good result |
|---|---|---|---|
| 1 | **Smoothness** | Servo unplugged. Turn the lever slowly by hand, a full turn each way | Even resistance, no notchy spots or grinding. If it's notchy, back off the stack bolts a quarter turn each |
| 2 | **Play** | Measure the lever tip's height above the table (calipers, at the notch at 290 mm). Push the tip up, then down, with a finger (~2 N) | Under about 0.5 mm of difference. More: tighten the stack bolts a little and repeat test 1 |
| 3 | **Tipping load** | Hang **2.4 kg** (e.g. a 2 L water bottle and a bit) at the **250 mm** hole. Measure the tip height again | Sags under ~3 mm and returns when unloaded; nothing creaks. Leave it 10 min: no creep |
| 4 | **Gear backlash** | Servo on and holding. Wiggle the lever gently by hand | Barely any free movement (under ~1 mm at the tip). Too much: slide the drive mount in a bit (radial slots) |
| 5 | **Multi-turn** | Run `../bench/servo_multiturn_test.py` with the servo in the rig (lever on, no weight): it turns the servo ±2 turns = **±180° of the lever** | The lever goes half a turn each way and stops where commanded |
| 6 | **Homing** | Hall sensor wired to a Pico / ESP32 (or a multimeter on an SS49E's output). Turn the lever past straight ahead (+x) | A clear jump in the output when the magnet passes, at the same lever angle every time |
| 7 | **Cable loop** | Lay your real servo cable and a USB cable through the loop (as in the ring README). Turn the lever to +180°, back to −180°, ten times | The U-turn travels round smoothly; the cables never climb out, kink or catch |
| 8 | **Drive under load** | Weight on (2.4 kg at 250 mm), command the servo through ±90° at normal speed | Moves without stalling or skipping teeth |

If test 5 fails (the servo can't do several turns), the ring can only turn 90°: stop and tell me; that changes the
design. If tests 1–3 fail even after adjusting the stack bolts, the printed races need rework (or PETG).

Regenerate: `cd mobile_manipulator_description/generator && python3 mono_ring_rig.py`
