# Lift joystick case

A small desk box for moving the robot's lift while teleoperating: a **KY-023 thumbstick** (only its front-back
axis is used: **push forward = up, pull back = down, let go = stop**) and a **Raspberry Pi Pico** as the USB device.
The lid's slot lets the stick move only front-back.

It stands on the desk beside the leader, not on the leader's foot: anything fixed near the pedestal would sit where
the leader's hanging gripper sweeps. Put it by your free hand (mono), or give it to the second person (dual).

Made by `mobile_manipulator_description/generator/lift_stick.py`.

| File | What | Print |
|---|---|---|
| `S1_stick_case.stl` | box with standoffs for both boards, USB opening, two ears for M4 screws or rubber feet | flat, 117 × 55 × 29 mm |
| `S2_stick_lid.stl` | lid with the front-back slot around the stick's shaft and an "up" arrow | top down |

## Dimensions: exact vs. assumed

- **Pico: exact** (datasheet: 51 × 21 mm, 4 × Ø2.1 holes on 47.0 × 11.4 mm, micro-USB on a short end).
  Pico W and Pico 2 have the same footprint.
- **KY-023: assumed.** Listings disagree (34 × 26, 34 × 32, 40 × 27 mm). Before printing, **measure your module**
  and tell me:
  1. board length and width;
  2. hole diameter and the spacing between hole centres (both directions);
  3. where the stick's centre is relative to the holes;
  4. height of the stick's round dome above the board;
  5. shaft diameter just above the dome, and the height of the stock cap's underside.

  Values 4 and 5 set the lid's height and slot width. The lid is the quick reprint if they're off.

## Assembly

1. Pico onto its 4 standoffs (M2 self-tapping), USB end at the opening.
2. KY-023 onto its 4 standoffs (M3 self-tapping), stick's front-back axis along the slot.
3. 5 short wires KY-023 → Pico (power, ground, the two axis outputs, the button).
4. Lid on (4 × M2.5 self-tapping), slot around the shaft. Refit the stock cap.
5. Screw the ears to the desk (M4) or stick on rubber feet.

The Pico will need its program (USB joystick or keyboard) at the software stage.
