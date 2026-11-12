# Lift joystick (controller grip)

A one-hand **controller** for moving the robot's lift while teleoperating: a **KY-023 thumbstick** that moves freely in
every direction (the software reads only its front-back axis: **push toward the front = up, pull back = down, let go =
stop**) and a **Raspberry Pi Pico**
as its own USB device. Hold it like a game-controller grip: palm along the back, **thumb on the stick**, **index and
middle finger on the two buttons** on the front face. The bottom is flat, so it can also lie on the table.

![lift joystick](preview_lift_stick.png)

Made by `mobile_manipulator_description/generator/lift_stick.py` (shape, checks, these files, the preview).

| File | What | Print |
|---|---|---|
| `S1_stick_tray.stl` | bottom tray: the Pico's slide-in mount (ridge, side walls with lips, back stops; room for headers), holes for 2 optional Pico screws, USB opening, 4 countersunk M2.5 holes underneath, 4 rubber-feet recesses | flat, bottom on the bed |
| `S2_stick_top.stl` | the grip: round stick hole and "up" arrow on the head, 4 posts for the joystick bracket, 2 button holes with guide sleeves + 2 posts for the button bracket on the front, 2 fins that stop the Pico | upside down (head top on the bed); tree supports under the sloping tail |
| `S3_stick_bracket.stl` | holds the joystick module under the head: 4 short standoffs at its holes; screws up into the top shell | flat |
| `S4_button_bracket.stl` | **optional, with the buttons:** holds both switches from behind (pockets, slots for their legs) | on its back |
| `S5_button_cap.stl` | **optional, print 2 with the buttons:** the wide button you press | flange down (no supports) |

Overall 116 × 42 × 44 mm (30 mm high at the back; the stick's cap reaches 58 mm), about 53 cm³ of plastic (+9 cm³ with
the buttons).

## The Pico: slides in, no screws needed

- **With or without headers.** A bare Pico: solder the wires into its holes from the top. A Pico with headers
  (pins pointing down, like the Pico H): solder the wires onto the pin tops. Either way you only solder on the top side.
- **To take it out:** remove the 4 screws underneath and lift the top off. Slide the Pico toward the front (about
  5 cm) until it leaves its rails, and lift it out. Solder, then slide it back in, USB end first, until it stops.
- **What holds it:** a ridge under its middle (between the two pin rows), walls on both sides with lips over its
  edges, two blocks behind it, and the top shell's two fins just in front of it once the top is on. Under both pin
  rows a channel stays free down to the floor, so a header's plastic strip and pins slide through.
- **Optional, e.g. while testing with the top off:** 2 × M2 × 6 self-tapping screws from above through its two front
  holes (the end away from the USB) into the ridge.
- The USB opening (13.5 × 9 mm) goes right up to the socket, so a normal micro-USB cable plugs in fully, and so would
  a USB-C one if you switch to a board of the same size with USB-C (both plug sizes are checked). A cable with a slim
  plug end takes less strain when you move your hand.

## Wiring

**Power the KY-023 from the Pico's 3.3 V, never from VBUS (5 V):** the Pico's analogue inputs take at most 3.3 V.

| KY-023 / button | Pico pin |
|---|---|
| +5V (the module's label; give it 3.3 V) | 3V3(OUT), pin 36 |
| GND | AGND, pin 33 |
| VRx / VRy (whichever runs front-back, check yours) | GP26 / ADC0, pin 31 (the other axis, if wired: GP27 / ADC1, pin 32) |
| SW (stick press) | GP22, pin 29 |
| index button (upper: stored positions) | GP20, pin 26 (its other leg to GND) |
| middle-finger button (lower: dead-man) | GP21, pin 27 (its other leg to GND) |

## Dimensions: exact vs. assumed

- **Pico: exact** (datasheet: 51 × 21 × 1 mm, 4 × Ø2.1 holes on 47.0 × 11.4 mm, micro-USB on a short end).
  Pico W and Pico 2 have the same footprint.
- **Joystick: the Prakti PS2 dual-axis module** (Amazon ASIN B0BTXGX8R9, `PK-JOY-PS2DUAL`; listed 40 × 26 × 20 mm =
  the standard KY-023 board with its bent pins, height without the cap). Its dimensions come from the one **dimensioned
  drawing** found for this standard board (Aus Electronics Direct TA0051): board 34.0 × 26.5 × 1.6 mm, **4 × Ø3.0
  holes on 26.2 × 19.9 mm**, header bent flat 4.9 mm past the end, stick 0.9 mm off the hole pattern's centre, pivot
  6.2 mm above the board, moving dome **Ø26.2**, cap Ø20 (top 27.5 mm above the board), **±33°** travel every way.
  Assumed: the dome's top height (~18.7 mm above the board, read off the drawing). **Check when it arrives:** the hole
  spacing (26.2 × 19.9) and the dome size; if they differ, only the top shell and the joystick bracket change.

  The dome passes right over the corner holes, so the module is held from **below**: it screws onto the joystick
  bracket's four short standoffs (**4 × M2.5 × 8 self-tapping**; the Ø3.0 holes are too tight for M3), and the bracket
  screws up into four posts in the top shell (**4 × M2 × 8 self-tapping**). The module then lifts away with the top
  shell, and the Pico can slide out of the tray. The cap, tilted fully any way, stays 1 mm above the case top (checked
  in 12 directions, dome included); the dome comes up 4.9 mm through a Ø27 hole.
- **Buttons: Robu "12x12x7.3mm Tactile Push Button Switch" (SKU 618210, 10 pcs, sold in India).** 7.3 mm overall,
  square stem (fits 3.4 × 3.8 caps), **1.05 mm stroke**, 3–5 N, rated 300,000 presses. Checked: the caps move freely
  through the full stroke (+0.25 mm) and still stand 1.95 mm out of the front when fully pressed. Assumed: how the
  7.3 mm splits into body (3.6) and stem (3.7); measure one when they arrive (if it's 7.0 overall, the cap gets 0.3 mm
  longer). Taps and long holds are told apart in the Pico's code (by timing, after a few ms of debounce).

## Buttons (optional, add later)

Two **wide** push buttons on the middle of the front face, one above the other, like a bumper over a trigger: the
**upper** one for the **index** finger, the **lower** one for the **middle** finger. The layout is symmetric, so it
works the same in either hand. Both caps are the same part, and both sit fully on the flat part of the front face.
Plain push buttons are enough: the robot only needs on / off from them (the dead-man and the stored positions below).

- Each **cap** (24 × 10 mm face, rounded corners) sticks out **3 mm** and slides in a guide sleeve in the front wall;
  a flange inside keeps it from falling out, and the switch's own spring pushes it back out. One switch sits behind the
  middle of each cap: pressed near an end, the cap tips a little in its 0.2 mm play and still clicks it.
- The **bracket** holds both switches from behind (their stems touch the caps) and screws onto two posts beside the
  buttons: 2 × M2 × 10 self-tapping.
- Until you add them, the two holes stay open; print the caps alone to close them if you like.

## What the buttons will do (plan for the software)

Nothing here changes the case; it's what the Pico's program and the robot side should do.

- **Lower button (middle finger) = dead-man (hold-to-run).** The robot moves **only while it's held**: the lift
  from the stick, and any move to a stored position. Let go and everything stops at once, including a move that's
  halfway there.
- **Upper button (index finger) = stored positions, chosen by how you press it.** Each pattern is mapped to a
  named position of the whole robot (arm joints + lift), e.g. **home**, **rest**, tote, table, shelf. The table lives
  in a settings file, so patterns and positions can be added or changed without touching the case:

  | Pattern (example) | Position (example) |
  |---|---|
  | 1 click | home |
  | 2 clicks | rest |
  | 3 clicks | tote |
  | long press (≥ 1 s) | table |
  | click + long press | shelf |
  | very long press (≥ 3 s) | save the current position into the last-used slot |

  A pattern counts once the button has been released for ~0.35 s (so a double click isn't read as two single
  clicks). Choosing a position only **selects** it; the robot moves there while the dead-man is held.
- **Stick press (its own switch):** a third input, free for e.g. "stop and clear the selected position".
- **If the controller goes quiet** (USB unplugged, the Pico hangs), the robot must stop by itself: the Pico sends its
  state continuously (~50 times a second), and the robot stops if nothing arrives for ~0.1–0.2 s. Without that, a
  dead-man button isn't a safety.
- For a real product, a 3-position **enabling switch** (as on industrial robot pendants: it also stops when squeezed
  hard in a panic) would replace the tactile switch.

## Assembly

1. Joystick module onto the joystick bracket's standoffs, its pin header toward the back (the Pico's end) (4 × M2.5 × 8
   self-tapping from above). Then the bracket into the top shell, the stick up through the round hole (4 × M2 × 8
   self-tapping up into the posts).
2. Solder the wires (table above) to the Pico; slide it into the tray's rails, USB end first, until it stops.
3. Buttons (optional): from inside the top shell, drop a cap into each button hole, put the switches into the
   bracket's pockets (stems to the front, legs through the slots), solder their wires, and screw the bracket onto the
   two posts (2 × M2 × 10).
4. Top shell on (the fins drop in front of the Pico); 4 × M2.5 × 16 countersunk from underneath into its bosses (10 mm through the tray, 6 into the boss).
   Refit the stock cap on the stick.
5. Stick 4 rubber feet (10 mm) into the recesses underneath.

**On the table:** keep its centre at least **40 cm from the leader's pedestal** (its axis). Within 33 cm, the
leader's gripper and wrist can come lower than the controller's top (58 mm) in hand-down and tilted poses (checked
over 1200 poses); at 40 cm even its end nearest the pedestal stays outside that.

The Pico will need its program (USB joystick or keyboard) at the software stage.
