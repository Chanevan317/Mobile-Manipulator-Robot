# Scene camera mount

An **Orbbec Gemini 2** (colour 86° × 55°, depth 91° × 66°, active stereo) on a **Feetech STS3215 7.4 V** pitch servo, on a printed mast standing on the **carriage**
in front of the C-Beam. It rides up and down with the arms (as the old scene camera did) and tilts to look ahead or
down at the work. Fits both robots: on the two-arm robot the mast bolts to the dual holder's front bridge, on the
one-arm robot to the mono holder's bar over the arm (turned 90° so the camera looks forward, its tilt axis **60 mm
behind and 15 mm left of the shoulder**: further back the camera sees more of the work), on a short 103 mm mast (C02).
Both put the lens 0.33 m over the carriage.

**Camera since 2026-11-24: Orbbec Gemini 2** (the OAK-D Pro W it was designed for was sold out in India; the Gemini 2
bought for the reBot arm is reused, held by 2 screws so it can move between the robots). Its USB-C port is in the
camera's **right end** (seen from behind), which faces the lift column. So the head is mirrored compared with the
OAK-D version: the **servo is on the camera's left end**, the **pivot arm on the right end, 40 mm out**, leaving room
for the USB plug and for the cable to turn back. The cable runs through a slot in the cradle's back plate and is held
by a cable tie on a bridge behind it, so it moves with the camera and never pulls on the port.

![scene camera mount](preview_scene_cam.png)

Made by `mobile_manipulator_description/generator/scene_cam.py` (`CAMERA = 'gemini2'`; `MM_SCENE_HW=oakd` builds the
OAK-D version). The Gemini 2's dimensions come from Orbbec's drawing and STEP model ([CAD download page, "Gemini 2 &
Gemini 335", V2.0](https://doc.orbbec.com/documentation/Orbbec%20Gemini%20330%20Series%20Documentation/Download%20CAD%20Files%20(Gemini%20330%20Series))).
Check the M3 hole spacing (45 mm) and the USB-C end on the real camera before printing the cradle.

## Swappable parts

Every joint between parts is the **same flange**: 40 × 44 × 6 mm, 4 × M3 from above into M3 × 8 heat-set inserts in
the part below, on a 28 × 34 mm pattern. So a different set-up is one replacement part:

| # | File | What | Swap it to… |
|---|---|---|---|
| C01 | `C01_mast_244.stl` | two-arm robot: 36 × 26 hollow mast, 3 mm walls (cables inside), 244 mm flange to flange | **change the height**: the lens sits at **mount + L + 62 mm** over the carriage origin. Two-arm: 0.33 m with L 244 (L 194 → 0.28 m, L 294 → 0.38 m) |
| C02 | `C02_mast_103_mono.stl` | one-arm robot: the same, 103 mm (it stands on the mono holder's bar, 165 mm over the plate top) | one-arm: 0.33 m with L 103 |
| C03 | `C03_mast_178_mono_ring_parked.stl` | only for the PARKED ring robot: the same, 178 mm. Not needed now | — |
| C04 | `C04_head.stl` | flange, crossbar reaching 23 mm forward, the servo's cradle (output face toward the camera, body hanging down) and the pivot upright | a forward offset or a different tilt mechanism: a new head |
| C05 | `C05_servo_strap.stl` | holds the servo in the head (2 case screws on its idler face + 2 M2 into the head) | — |
| C06 | `C06_camera_cradle.stl` | holds the camera by its two back M3 holes on 2 mm standoffs (the cooling fins stay open); one arm on the servo horn (4 × M3), the other on an M5 pivot bolt 40 mm past the USB end; a slot for the cable and a cable-tie bridge on the back | another camera: a new cradle |

The mounts are part of the robot's own prints (re-exported with this change):
- two-arm: `../dual_holder/D01 / D02`: a 2 mm pocket across the front bridge top for the flange, 4 inserts in its floor;
- one-arm: `../mono_holder/H01_holder.stl`: the same 2 mm pocket on its bar over the arm, 4 inserts in its floor (the
  parked ring robot used a pedestal on its clamp, `../mono_ring/M03_clamp_ring.stl`, with the 173 mm mast).

All PETG. Print orientations: masts **lying on their front face** (the box front is flush with both flanges, the cavity
is peaked at the back, so no supports; bending then runs along the layers, which is stiffer and stronger than across
them), head flange down, cradle back plate down, strap flat. The masts fit the A1's 256 mm bed lying down at any length
up to about 250 mm; a longer mast still fits diagonally.

## Mast stiffness (2026-11-16)

The mast was 24 × 24 mm with 2.5 mm walls, printed standing, so bending ran across the layers. Now 36 (front-back) ×
26 mm with 3 mm walls (with 4 perimeters per surface the wall prints solid), printed lying: about 3.3× stiffer front to
back and 2× sideways by section alone, before the gain from the layer direction. Front-back matters most: the camera
sits forward of the mast, so the lift starting and stopping rocks it that way. The box can't grow further: its back is
6 mm in front of the beam / end mounts (x 20), and its sides stay inside the flange screw heads (y ±17).
The arm sweep against the bigger mast found no new contacts: the same 91 of 396 poses touch the holder or mast as before.

## Why this height (lens 0.33 m over the carriage origin, 80 mm in front of the column)

The camera moves with the lift, so the lift height for each job was taken from where the arm works (hand down 0.12 m
over a table item, hand level 35 mm over a shelf board), and the camera's height **above the carriage** was varied.
Coverage of the OAK-D Pro AF (colour camera 69° × 55°, depth minimum 0.35 m, ideal 0.7–3 m) at the best tilt (this study was
made for the AF; the Gemini 2's colour camera, 86° × 55°, is wider, and its depth works from 0.15 m, so the height stays):

| Camera over the carriage origin | Tables 0.45–0.90 m: seen / in the ideal depth range | Shelves 0.30–1.10 m seen | Floor 0.6–3 m (lift 0.2) |
|---|---|---|---|
| 0.19 m (old camera) | 100 % / 57 % | 97–98 % | 99 % |
| 0.25 m | 100 % / 79 % | 100 % | 99 % |
| **0.30–0.35 m** | **100 % / 97–100 %** | **100 %** | **99–100 %** |
| 0.40 m | 100 % / 100 % | 100 % | 99 % |

Shelves are always closer than the ideal 0.7 m (the arm works right in front of them), so they are seen but not in the
ideal depth range at any height. Moving the camera further forward (0.14 m) lowered shelf coverage to 68–97 %. The tilt
does the rest: about 0° for driving, 40–45° for the floor ahead, 50–60° for tables, 15–35° for shelves.
Range −10° (looking slightly up) to +80° (down).

## Camera facts used (Orbbec Gemini 2: datasheet v1.1, drawing V2.0 20241107, STEP model)

| | |
|---|---|
| Size, weight | 89.46 × 25 × 30 mm, rounded ends; 97 g; under 2 W on average (7 W peak) over the USB-C cable |
| Mounting | 2 × M3 on the back (the finned face), **45 mm apart** on the centre line, **max 3 mm deep**; 1/4"-20 in the bottom (max 5 mm, not used) |
| USB-C | in the camera's **right end** seen from behind (the end away from the colour lens), centred; 2 × M2 cable-lock threads 15 mm apart (max 3 mm). The 8-pin sync port is in the other end (not used) |
| Lenses | IR pair ±25 mm (50 mm baseline), colour camera 11 mm left of centre (seen from behind), IR projector at the centre; all on the body's centre line |
| Views | colour 86° × 55° (16:9, 1920 × 1080, rolling shutter); depth 91° × 66°, active stereo IR, 0.15–10 m (best 0.2–5 m), aligned to colour on the camera |

## Assembly

1. Heat-set the inserts: 4 × M3 × 8 in each mast top, 2 × M2 in the head (strap), 1 × M5 × 6 in the cradle's pivot arm
   (and the 4 × M3 × 8 in the holder's front bridge or the ring clamp's pedestal).
2. Mast on its mount (two-arm: into the holder's pocket): 4 × M3 × 10 (heads sunk 3.2 mm in the flange; a ball-end key
   reaches them past the top flange).
3. Head on the mast: 4 × M3 × 14 through the access holes in the crossbar.
4. Camera onto the cradle: 2 × M3 × 8 through the back plate and standoffs into the camera's back holes (2 mm of
   thread: the holes are only 3 mm deep, so don't use longer screws).
5. Servo into the head's cradle, output face toward the middle; strap on its back (2 × M2 × 6 case screws + 2 × M2 × 6
   into the head).
6. Cradle onto the servo horn (4 × M3 × 8), the other arm on the pivot: M5 × 16 through the upright into the arm's
   insert, with a washer (the bolt turns in the upright).
7. USB cable: plug into the camera's right end, turn the cable back through the slot in the cradle's back plate, down
   the back, and tie it to the bridge there (one cable tie). Leave a loose loop down to the mast so the camera can tilt
   from −10° to +80°. Then the USB and servo cables into the window at the top of the mast's front face, down inside it
   and out of the opening at its bottom.

## Checked (`generator/scene_cam.py` and the robot model)

- No clashes between the parts, nor with the beam / end mounts (at any lift), the kit's gantry plate and V-wheels, the
  dual holder and both arm bases, the mono holder, its arm base and the base tote, or the (parked) ring's fixed parts; the ring's rotor and cable tray turn all the way round
  past the pedestal.
- Tilt: free from −40° to +120° (the range used is −10° to +80°), including the USB plug and the cable down to its tie.
- Lens 0.330 m over the carriage origin. One-arm robot: the whole tilt sweep also clears the lift's top end mount at
  full lift, the holder and the arm base (`mono_holder.py` check).
- Arms (500 random poses each, lift 0.1–0.7): every pose where an arm touches the new mast or camera was already
  blocked by the carriage, column or its own base, so the mast takes away no reach (the old post: 20 such poses on the
  two-arm robot, the new one 26, all blocked anyway; one-arm 0 / 1).
- The robot model (`urdf/`): `scene_mast` fixed to `carriage`, joint `scene_camera_pitch` (−10° … +80°, + = down),
  `scene_camera` (origin on the tilt axis) and `scene_camera_optical` (the colour lens).
