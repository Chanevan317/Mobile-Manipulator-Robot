"""Open Isaac Sim (5.x) with ONE arm alone (no base, lift or carriage), and slowly open and close its gripper.

The arm (left_base and everything below it, cut out of the robot URDF at run time; urdf/ is not touched) stands on a
fixed point 0.30 m over the floor, pointing along +x. Every arm joint holds zero; only the two jaws move, both with the
same target (no mimic): closed -> fully open (45 mm each, 90 mm gap) -> closed, one cycle every --period seconds.
Close the window to quit.

Run from this package folder:
    <isaac-sim>/python.sh scripts/viewers/isaac_arm_view.py
    (this machine: OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/viewers/isaac_arm_view.py)
Options:
    --period S        seconds per open + close cycle (default 20: about 4.5 mm/s per jaw at the fastest)
    --layout prototype | prototype_mono_arm   which robot's arm (default prototype; the mono one includes its ring)
"""
import argparse, math, os, sys
ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap.add_argument('--period', type=float, default=20.0)
ap.add_argument('--layout', default='prototype', choices=('prototype', 'prototype_mono_arm'))
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({'headless': False})

import numpy as np
import omni.kit.commands, omni.usd
from pxr import UsdLux, Sdf
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
sys.path.insert(0, here)
from drives import apply_drives
import urdf_variants

CTRL_HZ = 60
JAW_MAX = 0.045                      # each jaw's travel (URDF limit)
world = World(stage_units_in_meters=1.0, physics_dt=1 / 240.0, rendering_dt=1 / CTRL_HZ)
world.scene.add_ground_plane()
stage = omni.usd.get_context().get_stage()
UsdLux.DistantLight.Define(stage, Sdf.Path('/DistantLight')).CreateIntensityAttr(3000)

urdf = urdf_variants.arm_only(urdf_variants.layout_urdf(args.layout), side='left', name='arm')
_, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
cfg.merge_fixed_joints = False
cfg.fix_base = True                  # the stand is pinned to the world
cfg.make_default_prim = True
cfg.self_collision = False
cfg.import_inertia_tensor = True
cfg.convex_decomp = False
cfg.distance_scale = 1.0
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
_, robot_path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=urdf,
                                          import_config=cfg, get_articulation_root=True)
print('arm imported at', robot_path, '-', len(apply_drives(stage, robot_path, args.layout)), 'joints set up')

robot = world.scene.add(Articulation(prim_path=robot_path, name='arm'))
world.reset()
robot.initialize()
import cameras; cams = cameras.add()                 # the robot's cameras (MM_CAMERAS=0: off)
names = list(robot.dof_names)
print('joints:', names)
jaws = np.array([names.index('left_finger_right'), names.index('left_finger_left')])
set_camera_view(eye=np.array([0.85, -0.45, 0.45]), target=np.array([0.50, 0.0, 0.225]))  # at the gripper (TCP x 0.515 / mono 0.546, z 0.225)

k = 0
while app.is_running():
    t = k / CTRL_HZ
    g = 0.5 * JAW_MAX * (1.0 - math.cos(2.0 * math.pi * t / args.period))     # 0 -> 45 mm -> 0, smooth at both ends
    robot.apply_action(ArticulationAction(joint_positions=np.array([g, g]), joint_indices=jaws))
    world.step(render=True)
    k += 1
    if k % (2 * CTRL_HZ) == 0:
        q = robot.get_joint_positions()
        print(f't {t:5.1f} s  jaw target {g * 1000:4.1f} mm  right {q[jaws[0]] * 1000:4.1f}  left {q[jaws[1]] * 1000:4.1f} mm')
app.close()
