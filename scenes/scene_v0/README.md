# scene_v0: store scene for the one-arm (mono) and two-arm (dual) robots

![scene_v0: room, table with the robot, racks, shelves](preview_scene_v0.png)

A small store room for testing the mono (one-arm) robot in Isaac Sim 5.1, built from free
[NVIDIA SimReady](https://huggingface.co/datasets/nvidia/simready-assets) assets (CC BY 4.0, see `CREDITS.md`).

- **Room:** three walls (behind the table, the racks and the shelves), open at the back, grey floor; store lighting
  (8 ceiling strips + soft fill). In the viewport keep **Lighting > Stage Lights** (the "Default" rig ignores the scene's lights).
- **Table** (0.85 m) in front of the robot's start: the work / staging area, with a crate and a mixed back row.
- **Store sections:** blue rack 1 *breakfast and dry goods* (cereal, crackers, boxed food), blue rack 2 *snacks and
  canned* (canned food, chocolate, coffee cups) on the robot's left; grey shelf 1 *dairy* (milk, yogurt), grey shelf 2
  *sauces* (ketchups) on its right. Each board holds runs of one product side by side, like shelf facings. Which
  product goes where comes from the **product database** (section 6).
- **15 task products** placed where the one arm can reach them, each in its own section (0.03-0.25 kg, our weights;
  the packages' own weights are wrong).
- **Filler products:** the rest of each run, every run stocked at random from a few items to full (higher boards too).
  The ones the arm can reach move with physics; the others are fixed in place (they cost no simulation time).
  Each task product keeps at least 50 mm free on both sides, so the open jaws fit.

All commands from the repository root, in the `env_isaaclab` conda environment
(add `OMNI_KIT_ACCEPT_EULA=YES` for Isaac Sim).

The scripts that build, check and save this scene are in `scripts/scenes/scene_v0/` (run them from the repository
root); this folder holds the scene's data: the saved `scene_v0.usda`, `stations.json`, the preview and the credits.

## 1. Download the assets (once, ~500 MB)

```bash
python scripts/scenes/scene_v0/fetch_assets.py            # into scenes/assets/simready/ (not in git)
python scripts/scenes/scene_v0/fetch_assets.py --check    # what is there
```

## 2. Look at it

```bash
python scripts/scenes/scene_v0/check_scene_mono.py --no_grasp
```

Opens a full-screen window with the robot, checks the scene (below), then pauses physics so the view moves smoothly
(Play in the window resumes it). Or open the saved file `scene_v0.usda` in Isaac Sim (File > Open): the scene alone, no robot.

Options: `--fill 0.5` (fewer filler products; `0` = only the 15 task products), `--seed 3` (another random layout),
`--res 1920x1080`.

## 3. Check it

```bash
python scripts/scenes/scene_v0/check_scene_mono.py --headless --no_grasp
```

- reach: the arm can pick each task product from some spot in front of its furniture (arm geometry only);
- scene: every product lands and stands (tilt < 5 deg, moved < 10 mm), our weights, furniture stays put, crate walls up,
  every filler with physics stands, >= 50 mm free beside each task product. Last result: reach 15/15, scene 23/23.

Without `--no_grasp` it also picks up each task product with the arm (not run with the mono robot yet).

## 4. Save it

```bash
python scripts/scenes/scene_v0/save_scene.py              # -> scenes/scene_v0/scene_v0.usda (also --fill, --seed, --out)
```

The file refers to the downloaded assets by relative path; it holds none of NVIDIA's files.

## 5. Robot stations (scripted driving for arm training)

This is not real navigation. The robot is driven to **fixed, marked spots** in front of the shelves and the table,
and the arm is trained there. Each station is a base position and heading `(x, y, heading)` in metres and degrees.
The world frame is the robot's start spot: x points to the table, y to the blue racks.

- `home`: the start spot, the middle of the room (x -1.375, y -0.05), facing the table. Every script that puts the robot
  in the store starts it here (`check_scene_mono.py` excepted: its grasp test needs the robot at the table).
- One station per product run on every board the arm reaches, named `<board>_<product>`, for example
  `rack_b0_box_cracker_a01` or `shelf2_b1_ketchup_c01`. The base stops facing the board, and the arm can reach that
  run's middle product.
- `table_<product>`: one for each of the 4 task products on the table.

They are on the floor as flat marks: **blue** = home, **green** = a task product's station, **yellow** = another
product run. The small **orange** square is the station's turning point.

```bash
python scripts/scenes/scene_v0/stations_mono.py              # works out the stations -> stations.json (no simulator; rerun after a layout change)
python scripts/scenes/scene_v0/drive_stations_mono.py --list                       # names and coordinates
python scripts/scenes/scene_v0/drive_stations_mono.py --route home,rack_b0_box_cracker_a01,shelf_b1_milk_b01,home
python scripts/scenes/scene_v0/drive_stations_mono.py --tour                       # every station once (add --headless for no window)
python scripts/scenes/scene_v0/random_drive_mono.py                       # random stations, 3 s at each, until you close the window
python scripts/scenes/scene_v0/random_drive_mono.py --stay 5 --only task --seed 1   # options: stay time, which stations, sequence
```

How the robot drives: it backs straight out to the station's turning point, crosses the aisle (through the middle of
the aisle when a direct line isn't clear), turns on the way, and drives straight in. The arm stays folded while driving.
At a station, the wheels hold the base still while the arm works.

What `stations_mono.py` checks: the arm can reach the product from the station (joint limits), and the robot's outline
(the base plus the camera in front, + 2 cm) clears the furniture and walls at the station, on the way in and out, and
while turning. Not checked: the arm against the furniture (the grasp test does that).

## 5b. The dual-arm robot: two products at the same time

Every robot script here comes in a `_mono` and a `_dual` version. The dual robot always works with **both arms at once,
on two different products**: the left arm takes one, the right arm the one beside it.

```bash
python scripts/scenes/scene_v0/stations_dual.py                     # pair stations -> stations_dual.json (no simulator)
python scripts/scenes/scene_v0/pick_to_tote_dual.py                  # window: "Dual pick" panel + scene and both wrist cameras + scene depth
python scripts/scenes/scene_v0/pick_to_tote_dual.py --headless --all --place   # every pair: pick both, put both back
python scripts/scenes/scene_v0/station_menu_dual.py                  # window: drive to pair stations, pick / place both
python scripts/scenes/scene_v0/drive_stations_dual.py --route home,shelf_b1_milk_b01,home   # driving only
python scripts/scenes/scene_v0/random_drive_dual.py
python scripts/scenes/scene_v0/check_scene_dual.py --headless       # pair reach + scene check with the dual robot
```

- **Pair stations** (`stations_dual.py`): two task products on one board, 0.12-0.75 m apart (0.20 before the arm mounts
  turned to ±90° on 2026-11-25: the two arms' reach now meets in the middle), and both arms share **one lift**, so both products are gripped from the side at **one height** (the lower of their usual grip heights).
  5 pairs: four on blue rack 1's bottom board and the two ketchups on grey shelf 2. The table's products are only
  0.12 m apart: no base pose fits both arms beside the table. The driving scripts still use the mono stations (`stations_mono.py`).
- **The pick** (`lib/pick_place_dual.py`): drive there folded; unfold both arms while the lift rises; both hands in front of
  their products; both jaws open (fully, before going in); both straight in; both close; both lift 2 cm and back out
  of the shelf to a point from which the swing can't slide a held box along the board; elbow-open beside the robot;
  hands down beside the robot (the boxes turn to lying); over the two tote spots, high enough to clear the tote walls
  in any orientation; down; release; up just clear of the products; fold both arms. The lift stays where it is for the
  drive (at most 0.50 m), so the next pick starts near its grip height instead of from 0.20 m.
- **Asynchronous arms (2026-11-25):** between the shared lift moves each arm runs its own chain of moves (in, jaws, out,
  back out, elbow-open, hands down; on the way back the same) on its own clock, without waiting for the other
  (`proto_task.Task.parallel`); each jaw stops when it closes on its product. A pair takes 18-23 s to pick and 19-20 s
  to put back (was 29-32 s), all 5 pairs PASS (also on the 0.45 m base). `MM_DUAL_SYNC=1` = the old lock-step sequence.
  The pair beside the tall unit (rack 1's last pair) takes ~44 s: its hands must go high before they swing out.
- **The lift moves only while both hands are inside the robot's outline** (folded, over the tote) or straight in front
  of the shelf. Next to the table (rack 1's last pair) the hands swing out above the table and the crate on it
  (`side_min_z` in `stations_dual.json`), and the held boxes' length is added while they hang.
- **Place back:** the same path backwards. Each hand re-grips its product where it lies now (it slides and drops a little
  after release). Both hands are at one height, so the two boxes may be held a little deeper or shallower than when
  they were dropped. Each hand then goes that much further in or less far into the shelf, so the box lands on its spot.
- **Scene camera depth window** (in `pick_to_tote_mono.py`, `pick_to_tote_dual.py`, `station_menu_dual.py`; `cameras.depth_window()`):
  the scene camera's depth image, live, also while a pick runs: red = 0.2 m, through yellow and green, to blue = 2 m and
  beyond; black = no depth (outside the camera's range, as the real Gemini 2 reports). The line under it gives the nearest
  and median distance. `MM_DEPTH_SNAP=file.png` saves one frame.
- **Window layout** (`cameras.arrange()`, the same three scripts): the camera windows (scene, scene depth, wrist cameras)
  float in one even row along the bottom, under the main viewport, and the script's menu is a tab next to Render
  Settings on the right. Drag them anywhere after; `MM_UI_SNAP=file.png` saves a picture of the whole app window.
- `DUAL_DEBUG=1` prints, after every step, both products, both hands, the jaw gaps, the worst joint error and anything
  else that moved.
- Not yet: collision planning for two arms (mono uses `collide.py`'s planner; the dual moves are fixed patterns, checked
  in Isaac), table pairs, the training pipeline for two arms.

## 6. Locations and the product database

Every board is cut into fixed-width **locations**, each with an ID:

| ID | Meaning |
|---|---|
| `R1-B1-03` | blue rack 1 (R1; R2 = blue rack 2, S1 / S2 = grey shelves 1 / 2), board 1 (the lowest), slot 3 (counted from the left, as seen from the aisle) |
| `T1-01` … `T1-04` | the four pick spots on the table |

Racks have 10 slots of 0.206 m per board, grey shelves 6 of 0.23 m: 124 locations in all.

The **product database** `products.json` lists each product: id (`P01`…), a plain name (what it looks like, e.g.
"Milk carton with cap"; NVIDIA's variant codes like b01 are only in the package name), package (the asset), weight, size and
the **locations it covers**. One product can cover one location or many, on one or several boards. Neighbouring
locations on a board form one run of that product. The scene is built from this file: fillers stock each run, and the
task products sit in theirs. To change the store layout, edit the locations in `products.json`.

```bash
python3 scripts/scenes/scene_v0/store_db.py                        # check products.json, write locations.json, print the catalogue
python3 scripts/scenes/scene_v0/store_db.py --product cereal       # where a product is (id, package or part of its name)
python3 scripts/scenes/scene_v0/store_db.py --at S1-B2-05          # what is at a location, and which station serves it
```

To drive there from inside Isaac Sim instead of the terminal: `python scripts/scenes/scene_v0/station_menu_mono.py` opens the
scene with a **Robot stations** panel: the product (grouped by section, with its size, weight and friction), the
location (nearest, or unit / board / slot), the station that serves them, and Go / Home / Stop / Random. **View**
switches the viewport to the robot's scene or wrist camera (real specs, `scripts/common/cameras.py`). The **Up / Down** arrow keys move
the lift while held (at its real speed, 0–0.81 m).

The check fails if a location is unknown or claimed by two products, a task product has no location on its board, or
a table spot is empty. After an edit: `store_db.py`, then `stations_mono.py` (each station lists the locations it serves),
then `check_scene_mono.py` and `save_scene.py` in Isaac Sim.

## Files

| File | What |
|---|---|
| `scene_v0.usda` | the saved scene (refers to the downloaded assets by relative path) |
| `stations.json` | the robot stations: name, x, y, heading, turning point, product, how it is grasped, locations it serves |
| `products.json` | the product database: id, name, package, weight, size, the locations each product covers (edit this) |
| `locations.json` | every location ID: unit, board, slot, height, position, reachable, product, station (written by `store_db.py`) |
| `CREDITS.md` | attribution for the NVIDIA assets |
| `preview_scene_v0.png` | picture of the scene (rendered with `check_scene_mono.py --shot`, 4 views) |

Scripts, in `../../scripts/scenes/scene_v0/`:

| Script | What |
|---|---|
| `lib/scene.py` | builds the scene (layout, sections, walls, lights, products, station marks, every physics fix) |
| `lib/assets.py` | the asset list, sizes and our product weights |
| `lib/grasp_plan_mono.py` | where and how the mono arm can grasp each product (no simulator) |
| `fetch_assets.py` | downloads the assets |
| `check_scene_mono.py` | reach + scene checks with the robot; viewing window |
| `save_scene.py` | writes `scene_v0.usda` |
| `pick_to_tote_mono.py` | a panel in the Isaac window: choose a product (the 16 that can be picked), the robot drives to its station, picks it (side grasp or from above) and puts it in the tote; the scene camera (Orbbec Gemini 2 by default, `--tilt 40`) and the wrist camera show side by side. Moves are checked offline against the furniture and the products (`scripts/common/collide.py`; plans cached in `~/.cache/mobile_manipulator`) |
| `station_menu_mono.py` | opens the scene with a "Robot stations" panel: pick a product and / or a location, Go drives the robot to the station that serves it (Home, Stop too) |
| `store_db.py` | checks the product database, writes `locations.json`, looks up products and locations (no simulator) |
| `stations_mono.py` | works out the robot stations and routes -> `stations.json` (no simulator) |
| `lib/scene_robot.py` | the robot in the scene and one drive between stations (used by the two below) |
| `drive_stations_mono.py` | drives the robot along a route of stations (`--route`, `--tour`) |
| `random_drive_mono.py` | drives to random stations and stays at each for 3 s (`--stay`, `--count`, `--only`, `--seed`) |

## Changes made to the assets in our scene (the downloaded files stay untouched)

- the shelves' and racks' own collision shapes tipped bottles over: replaced by invisible flat boxes;
- products use a finer collision shape (convex decomposition): one convex hull left two products leaning 5 deg;
- furniture fixed in place; the crate's folding walls locked upright; the chocolate bar laid flat;
- product weights set to realistic values within the arm's ~0.25-0.3 kg payload.
