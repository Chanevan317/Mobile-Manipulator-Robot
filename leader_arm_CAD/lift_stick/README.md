# Lift joystick case

A small **handheld** remote for moving the robot's lift while teleoperating: a **KY-023 thumbstick** (only its
front-back axis is used: **push toward the tip = up, pull back = down, let go = stop**) and a **Raspberry Pi Pico**
as its own USB device. The lid's slot lets the stick move only front-back.

It's a **wedge**, like a game-controller grip: 14 mm thick at the back where it sits in your palm, 29 mm at the front
where the stick needs room, with every edge rounded. Use it two ways: **lying on the table** (flat bottom, rubber feet,
tip pointing away from you), or **in your hand**: palm along the thin back, fingers underneath, thumb on the stick, the
USB cable leaving the back end toward your wrist. It's a separate USB device,
independent of the leader: hold it in your free hand (mono), or give it to the second person (dual).

**On the table:** keep its centre at least **40 cm from the leader's pedestal** (its axis). Within 31 cm, the
leader's gripper and wrist can come lower than the box (about 50 mm with the cap) in hand-down and tilted poses
(checked over 610 poses). Further out the leader only passes high above it. For mono, 40 cm to the left of the
pedestal is about where your free hand rests anyway.

Made by `mobile_manipulator_description/generator/lift_stick.py`.

| File | What | Print |
|---|---|---|
| `S1_stick_tray.stl` | bottom tray: standoffs for the Pico (back) and the KY-023 (front), USB opening, 3 countersunk screw holes underneath, 4 rubber-feet recesses | flat |
| `S2_stick_top.stl` | the wedge-shaped top shell: front-back slot around the stick's shaft, "up" arrow to the tip; no screws on top | upside down |

Overall 104 × 37 × 29 mm (14 mm at the back), about 28 cm³ of plastic.

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

  Values 4 and 5 set the top shell's height over the stick and the slot width. The top shell is the reprint if
  they're off.

## Assembly

1. Pico onto its 4 standoffs at the back (M2 self-tapping), USB end at the opening.
2. KY-023 onto its 4 standoffs at the front (M3 self-tapping), with the axis you'll use along the case's length.
3. 5 short wires KY-023 → Pico (power, ground, the two axis outputs, the button).
4. Top shell on, slot around the shaft; 3 × M2.5 countersunk from underneath into its bosses. Refit the stock cap.
5. Stick 4 rubber feet (10 mm) into the recesses underneath.

The Pico will need its program (USB joystick or keyboard) at the software stage.
