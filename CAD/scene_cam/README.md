# Scene camera mount

A **Luxonis OAK-D Pro (AF)** on a **Feetech STS3215 7.4 V** pitch servo, on a printed mast standing on the **carriage**
in front of the C-Beam. It rides up and down with the arms (as the old scene camera did) and tilts to look ahead or
down at the work. Fits both robots: on the two-arm robot the mast bolts to the dual holder's front bridge, on the
one-arm robot to a pedestal on the ring clamp.

![scene camera mount](preview_scene_cam.png)

Made by `mobile_manipulator_description/generator/scene_cam.py`. The camera shape comes from Luxonis' own STEP files
([OAK-D Pro, depthai-hardware repo](https://github.com/luxonis/depthai-hardware/tree/master/DM9098_OAK-D-Pro/3D_Models));
a mesh of it is in `generator/oakd/`.

## Swappable parts

Every joint between parts is the **same flange**: 40 × 44 × 6 mm, 4 × M3 from above into M3 × 8 heat-set inserts in
the part below, on a 28 × 34 mm pattern. So a different set-up is one replacement part:

| # | File | What | Swap it to… |
|---|---|---|---|
| C01 | `C01_mast_240.stl` | two-arm robot: 36 × 26 hollow mast, 3 mm walls (cables inside), 240 mm flange to flange | **change the height**: the lens sits at **mount + L + 67 mm** over the carriage origin. Two-arm: 0.33 m with L 240 (L 190 → 0.28 m, L 290 → 0.38 m) |
| C02 | `C02_mast_173.stl` | one-arm robot: the same, 173 mm (its mount is 64.5 mm higher) | one-arm: 0.33 m with L 173 |
| C03 | `C03_head.stl` | flange, crossbar reaching 23 mm forward, the servo's cradle (output face toward the camera, body hanging down) and the pivot upright | a forward offset or a different tilt mechanism: a new head |
| C04 | `C04_servo_strap.stl` | holds the servo in the head (2 case screws on its idler face + 2 M2 into the head) | — |
| C05 | `C05_camera_cradle.stl` | holds the camera by its two back M4 holes on 3 mm standoffs (the cooling fins stay open); one arm on the servo horn (4 × M3), the other on an M5 pivot bolt; the bottom (USB-C plug) left open | another camera: a new cradle |

The mounts are part of the robot's own prints (re-exported with this change):
- two-arm: `../dual_holder/D01 / D02`: a 2 mm pocket across the front bridge top for the flange, 4 inserts in its floor;
- one-arm: `../mono_ring/M03_clamp_ring.stl`: a pedestal rising from inside the cable drum to z 90 (over the turning
  cable tray and rotor ribs), 4 inserts on top.

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
Coverage of the OAK-D Pro (colour camera 69° × 55°, depth minimum 0.35 m, ideal 0.7–3 m) at the best tilt:

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

## Camera facts used (from Luxonis' STEP file and datasheet)

| | |
|---|---|
| Size, weight | 97 × 29.7 × 23.4 mm (with the front bezel), 91 g |
| Mounting | 2 × M4 on the back, 75 mm apart (VESA 75), on two flat pads in the cooling fins; 1/4"-20 tripod thread in the bottom near the left end (not used) |
| USB-C | in the **bottom**, 13.5 mm right of the centre: the plug and cable leave downward (the cradle leaves it open). Tilted down, the plug swings back toward the beam: the tilt axis sits 23 mm in front of the mast so a straight plug with its strain relief (28 mm) clears the beam at 80° |
| Power | 4–7.5 W; the IR dot projector and flood LED need Luxonis' USB Y-adapter for extra power |
| Lenses | stereo pair 75 mm apart, colour camera in the middle, 4.8 mm above the body's centre line |

## Assembly

1. Heat-set the inserts: 4 × M3 × 8 in each mast top, 2 × M2 in the head (strap), 1 × M5 × 6 in the cradle's pivot arm
   (and the 4 × M3 × 8 in the holder's front bridge or the ring clamp's pedestal).
2. Mast on its mount (two-arm: into the holder's pocket): 4 × M3 × 10 (heads sunk 3.2 mm in the flange; a ball-end key
   reaches them past the top flange).
3. Head on the mast: 4 × M3 × 14 through the access holes in the crossbar.
4. Camera onto the cradle: 2 × M4 × 12 through the back plate into the camera's back holes (about 5 mm of thread).
5. Servo into the head's cradle, output face toward the middle; strap on its back (2 × M2 × 6 case screws + 2 × M2 × 6
   into the head).
6. Cradle onto the servo horn (4 × M3 × 8), the other arm on the pivot: M5 × 16 through the upright into the arm's
   insert, with a washer (the bolt turns in the upright).
7. USB and servo cables into the window at the top of the mast's front face, down inside it and out of the opening at
   its bottom (-y side). The window and the cavity (30 × 20 mm) pass the USB-C plug.

## Checked (`generator/scene_cam.py` and the robot model)

- No clashes between the parts, nor with the beam / end mounts (at any lift), the kit's gantry plate and V-wheels, the
  dual holder and both arm bases, or the ring's fixed parts; the ring's rotor and cable tray turn all the way round
  past the pedestal.
- Tilt: free from −40° to +120° (the range used is −10° to +80°), including the USB-C plug under the camera.
- Lens 0.330 m over the carriage origin, 80 mm in front of the column axis (both robots).
- Arms (500 random poses each, lift 0.1–0.7): every pose where an arm touches the new mast or camera was already
  blocked by the carriage, column or its own base, so the mast takes away no reach (the old post: 20 such poses on the
  two-arm robot, the new one 26, all blocked anyway; one-arm 0 / 1).
- The robot model (`urdf/`): `scene_mast` fixed to `carriage`, joint `scene_camera_pitch` (−10° … +80°, + = down),
  `scene_camera` (origin on the tilt axis) and `scene_camera_optical` (the colour lens).
