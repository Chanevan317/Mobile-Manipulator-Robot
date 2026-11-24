# Mobile-Manipulator-Robot

A wheeled mobile manipulator for restocking retail shelves: a mecanum base, a C-Beam lift and 3D-printed SCARA
arms (STS3215 servos), with a small leader arm for teleoperation. Work in progress: the arms are being printed, and
the robot is tested in simulation (Isaac Sim, ROS 2).

**One-arm prototype** (the official prototype: the lift column shifted to the right, the carriage facing the centre with
one arm on it, the tote on the base behind the arm):

![one-arm robot](mobile_manipulator_description/preview_prototype_mono.png)

**Two-arm layout** (both arms beside the column):

![two-arm robot](mobile_manipulator_description/preview_prototype_dual.png)

## Folders

| Folder | What |
|---|---|
| `CAD/SCARA_arm/` | the follower arm: print files, STEP, assembly, fastener lists |
| `CAD/mono_holder/` | one-arm robot: printed holder for the arm base and the scene camera mast on the C-Beam carriage |
| `CAD/dual_holder/` | two-arm robot: printed holder for both arm bases on the C-Beam carriage (and the scene camera mast) |
| `CAD/scene_cam/` | scene camera: OAK-D Pro W on a pitch servo, on a swappable printed mast on the carriage |
| `CAD/mono_ring/` | PARKED (replaced by the mono holder on 2026-11-23): the shoulder ring around the lift column |
| `CAD/leader_arm/` | the leader arm for teleoperation |
| `CAD/lift_stick/` | one-hand controller for the lift: thumbstick, dead-man and stored-position buttons |
| `mobile_manipulator_description/` | the robot model (URDF, meshes) and the CAD generator |
| `scenes/` | simulation scenes: `scene_v0/` store scene for the one-arm prototype (saved USD, robot stations), downloaded assets in `assets/` (not in git) |
| `scripts/` | every simulation script, by topic: robot tests, viewers, tasks, planning, ROS, scenes (`scripts/README.md`) |
| `train_pipeline/` | learning from demonstrations with LeRobot: record episodes in the store scene, check, convert to a LeRobotDataset, train ACT (`train_pipeline/README.md`) |

Large files (STL, STEP, PNG, ...) are stored with Git LFS: run `git lfs install` before cloning.
