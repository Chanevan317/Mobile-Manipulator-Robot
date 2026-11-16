# scene_v0: store scene for the one-arm prototype

![scene_v0: room, table with the robot, racks, shelves](preview_scene_v0.png)

A small store room for testing the mono (one-arm) robot in Isaac Sim 5.1, built from free
[NVIDIA SimReady](https://huggingface.co/datasets/nvidia/simready-assets) assets (CC BY 4.0, see `CREDITS.md`).

- **Room:** three walls (behind the table, the racks and the shelves), open at the back; store lighting
  (8 ceiling strips + soft fill). In the viewport keep **Lighting > Stage Lights** (the "Default" rig ignores the scene's lights).
- **Table** (0.85 m) in front of the robot's start, with a crate. **Two blue racks** on its left, **two grey shelves** on its right.
- **15 task products** placed where the one arm can reach them (0.03-0.25 kg, our weights; the packages' own weights are wrong).
- **Filler products:** one row at the front of each board, each board filled at random from nearly empty to full.
  The ones the arm can reach move with physics; the others are fixed in place (they cost no simulation time).

All commands from the repository root, in the `env_isaaclab` conda environment
(add `OMNI_KIT_ACCEPT_EULA=YES` for Isaac Sim).

## 1. Download the assets (once, ~500 MB)

```bash
python scenes/scene_v0/fetch_assets.py            # into scenes/assets/simready/ (not in git)
python scenes/scene_v0/fetch_assets.py --check    # what is there
```

## 2. Look at it

```bash
python scenes/scene_v0/check_scene.py --no_grasp
```

Opens a full-screen window with the robot, checks the scene (below), then pauses physics so the view moves smoothly
(Play in the window resumes it). Or open the saved file `scene_v0.usda` in Isaac Sim (File > Open): the scene alone, no robot.

Options: `--fill 0.5` (fewer filler products; `0` = only the 15 task products), `--seed 3` (another random layout),
`--res 1920x1080`.

## 3. Check it

```bash
python scenes/scene_v0/check_scene.py --headless --no_grasp
```

- reach: the arm can pick each task product from some spot in front of its furniture (arm geometry only);
- scene: every product lands and stands (tilt < 5 deg, moved < 10 mm), our weights, furniture stays put, crate walls up,
  every filler with physics stands. Last result: reach 15/15, scene 22/22.

Without `--no_grasp` it also picks up each task product with the arm (not run with the mono robot yet).

## 4. Save it

```bash
python scenes/scene_v0/save_scene.py              # -> scenes/scene_v0/scene_v0.usda (also --fill, --seed, --out)
```

The file refers to the downloaded assets by relative path; it holds none of NVIDIA's files.

## Files

| File | What |
|---|---|
| `scene.py` | builds the scene (layout, walls, lights, products, every physics fix) |
| `assets.py` | the asset list, sizes and our product weights |
| `grasp_plan.py` | where and how the mono arm can grasp each product (no simulator) |
| `fetch_assets.py` | downloads the assets |
| `check_scene.py` | reach + scene checks with the robot; viewing window |
| `save_scene.py` | writes `scene_v0.usda` |
| `CREDITS.md` | attribution for the NVIDIA assets |
| `preview_scene_v0.png` | picture of the scene (rendered with `check_scene.py --shot`, 4 views) |

## Changes made to the assets in our scene (the downloaded files stay untouched)

- the shelves' and racks' own collision shapes tipped bottles over: replaced by invisible flat boxes;
- products use a finer collision shape (convex decomposition): one convex hull left two products leaning 5 deg;
- furniture fixed in place; the crate's folding walls locked upright; the chocolate bar laid flat;
- product weights set to realistic values within the arm's ~0.25-0.3 kg payload.
