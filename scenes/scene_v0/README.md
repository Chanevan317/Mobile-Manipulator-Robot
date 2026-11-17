# scene_v0: store scene for the one-arm prototype

![scene_v0: room, table with the robot, racks, shelves](preview_scene_v0.png)

A small store room for testing the mono (one-arm) robot in Isaac Sim 5.1, built from free
[NVIDIA SimReady](https://huggingface.co/datasets/nvidia/simready-assets) assets (CC BY 4.0, see `CREDITS.md`).

- **Room:** three walls (behind the table, the racks and the shelves), open at the back, grey floor; store lighting
  (8 ceiling strips + soft fill). In the viewport keep **Lighting > Stage Lights** (the "Default" rig ignores the scene's lights).
- **Table** (0.85 m) in front of the robot's start: the work / staging area, with a crate and a mixed back row.
- **Store sections:** blue rack 1 *breakfast and dry goods* (cereal, crackers, boxed food), blue rack 2 *snacks and
  canned* (canned food, chocolate, coffee cups) on the robot's left; grey shelf 1 *dairy* (milk, yogurt), grey shelf 2
  *sauces* (ketchups) on its right. Each board holds runs of one product side by side, like shelf facings.
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
python scripts/scenes/scene_v0/check_scene.py --no_grasp
```

Opens a full-screen window with the robot, checks the scene (below), then pauses physics so the view moves smoothly
(Play in the window resumes it). Or open the saved file `scene_v0.usda` in Isaac Sim (File > Open): the scene alone, no robot.

Options: `--fill 0.5` (fewer filler products; `0` = only the 15 task products), `--seed 3` (another random layout),
`--res 1920x1080`.

## 3. Check it

```bash
python scripts/scenes/scene_v0/check_scene.py --headless --no_grasp
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

- `home`: the start spot.
- One station per product run on every board the arm reaches, named `<board>_<product>`, for example
  `rack_b0_box_cracker_a01` or `shelf2_b1_ketchup_c01`. The base stops facing the board, and the arm can reach that
  run's middle product.
- `table_<product>`: one for each of the 4 task products on the table.

They are on the floor as flat marks: **blue** = home, **green** = a task product's station, **yellow** = another
product run. The small **orange** square is the station's turning point.

```bash
python scripts/scenes/scene_v0/stations.py              # works out the stations -> stations.json (no simulator; rerun after a layout change)
python scripts/scenes/scene_v0/drive_stations.py --list                       # names and coordinates
python scripts/scenes/scene_v0/drive_stations.py --route home,rack_b0_box_cracker_a01,shelf_b1_milk_b01,home
python scripts/scenes/scene_v0/drive_stations.py --tour                       # every station once (add --headless for no window)
python scripts/scenes/scene_v0/random_drive.py                       # random stations, 3 s at each, until you close the window
python scripts/scenes/scene_v0/random_drive.py --stay 5 --only task --seed 1   # options: stay time, which stations, sequence
```

How the robot drives: it backs straight out to the station's turning point, crosses the aisle (through the middle of
the aisle when a direct line isn't clear), turns on the way, and drives straight in. The arm stays folded while driving.
At a station, the wheels hold the base still while the arm works.

What `stations.py` checks: the arm can reach the product from the station (joint limits), and the robot's outline
(the base plus the camera in front, + 2 cm) clears the furniture and walls at the station, on the way in and out, and
while turning. Not checked: the arm against the furniture (the grasp test does that).

## Files

| File | What |
|---|---|
| `scene_v0.usda` | the saved scene (refers to the downloaded assets by relative path) |
| `stations.json` | the robot stations: name, x, y, heading, turning point, product, how it is grasped |
| `CREDITS.md` | attribution for the NVIDIA assets |
| `preview_scene_v0.png` | picture of the scene (rendered with `check_scene.py --shot`, 4 views) |

Scripts, in `../../scripts/scenes/scene_v0/`:

| Script | What |
|---|---|
| `lib/scene.py` | builds the scene (layout, sections, walls, lights, products, station marks, every physics fix) |
| `lib/assets.py` | the asset list, sizes and our product weights |
| `lib/grasp_plan.py` | where and how the mono arm can grasp each product (no simulator) |
| `fetch_assets.py` | downloads the assets |
| `check_scene.py` | reach + scene checks with the robot; viewing window |
| `save_scene.py` | writes `scene_v0.usda` |
| `stations.py` | works out the robot stations and routes -> `stations.json` (no simulator) |
| `lib/scene_robot.py` | the robot in the scene and one drive between stations (used by the two below) |
| `drive_stations.py` | drives the robot along a route of stations (`--route`, `--tour`) |
| `random_drive.py` | drives to random stations and stays at each for 3 s (`--stay`, `--count`, `--only`, `--seed`) |

## Changes made to the assets in our scene (the downloaded files stay untouched)

- the shelves' and racks' own collision shapes tipped bottles over: replaced by invisible flat boxes;
- products use a finer collision shape (convex decomposition): one convex hull left two products leaning 5 deg;
- furniture fixed in place; the crate's folding walls locked upright; the chocolate bar laid flat;
- product weights set to realistic values within the arm's ~0.25-0.3 kg payload.
