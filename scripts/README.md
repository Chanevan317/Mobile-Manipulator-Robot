# scripts

Every simulation script of the project, by topic. Run them **from the repository root**. The Isaac Sim ones need the
`env_isaaclab` conda environment:

```bash
OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/<folder>/<script>.py [--headless]
```

The robot itself (URDF, meshes, the CAD generator that makes them) is in `mobile_manipulator_description/`. The scene
data (saved USD, robot stations, downloaded assets) is in `scenes/`.

| Folder | What is in it |
|---|---|
| `common/` | shared code, not run on its own: `repo_paths.py` (repo locations, import path), `drives.py` (motor gains), `scara_ik.py` (arm + lift inverse kinematics), `self_collision.py`, `urdf_variants.py`, `collide.py` (offline collision check + move planning around furniture / products), `proto_task.py` (task helpers: moves, driving, reports), `smooth_path.py` (TOPP-RA motion blending), `cameras.py` (the scene + wrist cameras at the real specs, added in every Isaac script; `MM_CAMERAS=0` turns them off), `sim_app.py` (starts Isaac in a window at the screen's full resolution in every script; `--res WxH` in the scene checks) |
| `robot_tests/` | Isaac tests of the robot model: `isaac_joint_test.py` (every joint, `--hold`), `isaac_base_test.py` (mecanum drive), `isaac_grasp_test.py`, `isaac_import_and_test.py` (looping demo, `--save` a USD), `isaac_mono_load_test.py` (ring robot) / `isaac_dual_load_test.py` (forces for the strength checks) |
| `viewers/` | just look at the robot: `isaac_view.py` (both layouts side by side), `isaac_arm_view.py` (one arm on a stand), `camera_view.py` (drive to a station, save what the robot's cameras see) |
| `tasks/mono/` | one-arm tasks made for the PARKED ring robot (`prototype_mono_ring`, their default since 2026-11-23): `isaac_mono_task.py` (table → tote → table), `isaac_mono_table_task.py`, `isaac_mono_shelf_task.py`; their plans in `lib/mono_poses.py` (which also holds the one-arm prototype's travel fold and tote spot) |
| `tasks/dual/` | two-arm layout tasks: `isaac_table_task.py`, `isaac_shelf_task.py`; helpers in `lib/` (`shelf_task_prototype.py`, `proto_poses.py`) |
| `planning/` | no simulator: `reach_test.py` (reach maps for tables, shelves, tote) |
| `ros/` | ROS 2: `ros_joint_check.py` (joint limits, zero pose, directions through TF; run with ROS sourced, conda off) |
| `scenes/scene_v0/` | the store scene: `fetch_assets.py`, `check_scene_mono.py`, `save_scene.py`, `store_db.py` (location IDs + product database), `stations_mono.py` (robot stations), `drive_stations_mono.py` (drive a route), `random_drive_mono.py` (random stations, stay 3 s each), `station_menu_mono.py` (pick a product / location in a panel in the Isaac window, the robot drives there), `pick_to_tote_mono.py` (one pick from a station into the tote, scene + wrist camera views side by side); helpers in `lib/` (`scene.py` builder, `assets.py`, `grasp_plan_mono.py`, `scene_robot.py`, `pick_place_mono.py` = the planner-driven pick / place used by `pick_to_tote_mono.py` and `train_pipeline/record`). **Dual (two arms at once):** `stations_dual.py` (pair stations: two task products on one board, one per arm -> `stations_dual.json`), `drive_stations_dual.py`, `random_drive_dual.py`, `pick_to_tote_dual.py` (both arms pick two products into the tote at the same time and put them back; `lib/pick_place_dual.py`), `station_menu_dual.py` (drive to pair stations, pick / place both, three camera views), `check_scene_dual.py` (pair reach + scene check). Guide: `scenes/scene_v0/README.md` section 5b |

The scripts you run sit directly in their topic folder; code they only import sits in that folder's `lib/` (and in
`common/` when several topics share it). Each script's first lines say how to run it and what it reports.

Adding a script: put it in the folder of its topic and start it with

```python
import os, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', 'common')); import repo_paths as RP   # ('..', '..', 'common' two levels down)
```

Every script folder is then importable (`import scara_ik`, `import scene`, ...), and `RP.urdf()`, `RP.PKG`,
`RP.scene_dir()` point into the repository wherever it is cloned.
