# Mobile-Manipulator-Robot

A wheeled mobile manipulator for restocking retail shelves: a mecanum base, a C-Beam lift and 3D-printed SCARA
arms (STS3215 servos), with a small leader arm for teleoperation. Work in progress: the arms are being printed, and
the robot is tested in simulation (Isaac Sim, ROS 2).

![robot](mobile_manipulator_description/preview_prototype_dual.png)

## Folders

| Folder | What |
|---|---|
| `CAD/SCARA_arm/` | the follower arm: print files, STEP, assembly, fastener lists |
| `CAD/mono_ring_draft/` | draft shoulder ring for the one-arm version, with a bench test rig |
| `CAD/leader_arm/` | the leader arm for teleoperation |
| `CAD/lift_stick/` | handheld joystick for the lift |
| `mobile_manipulator_description/` | the robot model (URDF, meshes), the CAD generator and the simulation scripts |

Large files (STL, STEP, PNG, ...) are stored with Git LFS: run `git lfs install` before cloning.
