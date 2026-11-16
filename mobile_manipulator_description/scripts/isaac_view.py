"""Open Isaac Sim (5.x) with the robot standing on the floor, and do nothing else.

By default both robots stand side by side: the two-arm robot (prototype) and the one-arm robot (prototype_mono_arm,
1.2 m to its left). No motion is commanded: every drive holds its zero pose (arms along their mount heading, lift at
the bottom, jaws closed, scene camera level). Close the window to quit.

Run from this package folder:
    <isaac-sim>/python.sh scripts/isaac_view.py
    (this machine: OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/isaac_view.py)
Options:
    --layout both | prototype | prototype_mono_arm   which robot(s) (default both)
    --paused             open with the simulation stopped (press Play in the window to start physics)
    --fix_base           pin each base_link to the world
    --self_collision     robot parts collide with each other (see self_collision.py)
See isaac_arm_view.py for one arm alone.
"""
import argparse, os, sys
ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--layout', default='both', choices=('both', 'prototype', 'prototype_mono_arm'))
ap.add_argument('--paused', action='store_true')
ap.add_argument('--fix_base', action='store_true')
ap.add_argument('--self_collision', action='store_true')
args = ap.parse_args()
LAYOUTS = ['prototype', 'prototype_mono_arm'] if args.layout == 'both' else [args.layout]
OFFSET_Y = 1.2                       # the one-arm robot stands this far to the left (+y) of the two-arm one

from isaacsim import SimulationApp
app = SimulationApp({'headless': False})

import numpy as np
import omni.kit.commands, omni.usd
from pxr import UsdLux, UsdGeom, Sdf, Gf
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
sys.path.insert(0, here)
from drives import apply_drives
import urdf_variants

world = World(stage_units_in_meters=1.0, physics_dt=1 / 240.0, rendering_dt=1 / 60.0)
world.scene.add_ground_plane()
stage = omni.usd.get_context().get_stage()
UsdLux.DistantLight.Define(stage, Sdf.Path('/DistantLight')).CreateIntensityAttr(3000)

for k, layout in enumerate(LAYOUTS):
    name = 'dual_arm' if layout == 'prototype' else 'mono_arm'
    urdf = urdf_variants.renamed(urdf_variants.layout_urdf(layout), name)
    _, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
    cfg.merge_fixed_joints = False      # keep camera / tcp frames as prims
    cfg.fix_base = args.fix_base
    cfg.make_default_prim = (k == 0)
    cfg.self_collision = False          # set below with the filtered pairs when --self_collision is given
    cfg.import_inertia_tensor = True
    cfg.convex_decomp = False           # collision meshes are already convex pieces
    cfg.distance_scale = 1.0
    cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    _, robot_path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=urdf,
                                              import_config=cfg, get_articulation_root=True)
    top = '/' + robot_path.strip('/').split('/')[0]
    if layout == 'prototype_mono_arm' and len(LAYOUTS) == 2:
        UsdGeom.XformCommonAPI(stage.GetPrimAtPath(top)).SetTranslate(Gf.Vec3d(0.0, OFFSET_Y, 0.0))
    # Force drives with the gains from drives.py (per layout); without this the importer's acceleration drives leave the arms limp.
    print(layout, 'imported at', top, '-', len(apply_drives(stage, robot_path, layout)), 'joints set up')
    if args.self_collision:
        from self_collision import setup_self_collision
        print('  self-collision on,', setup_self_collision(stage, urdf), 'pairs ignored')

cy = OFFSET_Y / 2 if len(LAYOUTS) == 2 else 0.0
set_camera_view(eye=np.array([2.4, cy - 1.6, 1.6]), target=np.array([0.2, cy, 0.6]))
if args.paused:
    print('paused: press Play to start physics')
    while app.is_running():
        app.update()
else:
    world.reset()
    while app.is_running():
        world.step(render=True)
app.close()
