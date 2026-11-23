"""Open Isaac Sim (5.x) with the robot standing on the floor, and do nothing else.

By default both robots stand side by side: the two-arm robot (prototype) and the one-arm robot (prototype_mono_arm,
1.2 m to its left). No motion is commanded: every drive holds its zero pose (arms along their mount heading, lift at
the bottom, jaws closed, scene camera level). Close the window to quit.

Each robot is converted to its OWN USD file (temp folder) and referenced into the scene. Importing two URDFs straight
into one stage does not work: the importer puts every robot's meshes and collision shapes in the same top-level
/visuals and /colliders folders, and the two robots share link names, so the second import replaced the first
robot's collision shapes (it was thrown off the floor) (2026-11-16).

Run from this package folder:
    <isaac-sim>/python.sh scripts/viewers/isaac_view.py
    (this machine: OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/viewers/isaac_view.py)
Options:
    --layout both | prototype | prototype_mono_arm   which robot(s) (default both)
    --paused             open with the simulation stopped (press Play in the window to start physics)
    --fix_base           pin each base_link to the world
    --self_collision     robot parts collide with each other (see self_collision.py)
    --headless --duration S   test run: no window, S seconds, then prints how far each base moved
See isaac_arm_view.py for one arm alone.
"""
import argparse, os, sys, tempfile
ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap.add_argument('--layout', default='both', choices=('both', 'prototype', 'prototype_mono_arm'))
ap.add_argument('--paused', action='store_true')
ap.add_argument('--fix_base', action='store_true')
ap.add_argument('--self_collision', action='store_true')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--duration', type=float, default=0.0, help='stop after S seconds (0 = until the window is closed)')
args = ap.parse_args()
LAYOUTS = ['prototype', 'prototype_mono_arm'] if args.layout == 'both' else [args.layout]
OFFSET_Y = 1.2                       # the one-arm robot stands this far to the left (+y) of the two-arm one

from isaacsim import SimulationApp
import sim_app                                        # noqa: E402  (window at the screen's resolution)
app = sim_app.start(args.headless)

import numpy as np
import omni.kit.commands, omni.usd
from pxr import UsdLux, UsdGeom, UsdPhysics, Sdf, Gf
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
sys.path.insert(0, here)
from drives import apply_drives
import urdf_variants

world = World(stage_units_in_meters=1.0, physics_dt=1 / 240.0, rendering_dt=1 / 60.0)
world.scene.add_ground_plane()
stage = omni.usd.get_context().get_stage()
UsdLux.DistantLight.Define(stage, Sdf.Path('/DistantLight')).CreateIntensityAttr(3000)
tmp = tempfile.mkdtemp(prefix='mm_view_')

robots = []
for layout in LAYOUTS:
    name = 'dual_arm' if layout == 'prototype' else 'mono_arm'
    urdf = urdf_variants.renamed(urdf_variants.layout_urdf(layout), name)
    _, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
    cfg.merge_fixed_joints = False      # keep camera / tcp frames as prims
    cfg.fix_base = args.fix_base
    cfg.make_default_prim = True        # the robot is the default prim of its own USD file
    cfg.self_collision = False          # set below with the filtered pairs when --self_collision is given
    cfg.import_inertia_tensor = True
    cfg.convex_decomp = False           # collision meshes are already convex pieces
    cfg.distance_scale = 1.0
    cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    usd = os.path.join(tmp, name, name + '.usd'); os.makedirs(os.path.dirname(usd), exist_ok=True)
    omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=urdf, import_config=cfg, dest_path=usd)
    # each robot under its own parent Xform that carries its place on the floor (setting the translate on the referenced
    # robot prim itself silently did nothing: its own xform ops win, so both robots spawned in one spot and pushed apart)
    slot = UsdGeom.Xform.Define(stage, Sdf.Path(f'/{name}_slot'))
    dy = OFFSET_Y if (layout == 'prototype_mono_arm' and len(LAYOUTS) == 2) else 0.0
    slot.AddTranslateOp().Set(Gf.Vec3d(0.0, dy, 0.0))
    top = f'/{name}_slot/{name}'
    add_reference_to_stage(usd_path=usd, prim_path=top)
    roots = [str(p.GetPath()) for p in stage.Traverse()
             if str(p.GetPath()).startswith(top + '/') and p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError(f'{layout}: expected one articulation root under {top}, found {roots}')
    # Force drives with the gains from drives.py (per layout); without this the importer's acceleration drives leave the arms limp.
    n = len(apply_drives(stage, top, layout))
    print(f'{layout}: {top} from {usd}, articulation {roots[0]}, {n} joints set up')
    if args.self_collision:
        from self_collision import setup_self_collision
        print('  self-collision on,', setup_self_collision(stage, urdf), 'pairs ignored')
    robots.append((layout, world.scene.add(Articulation(prim_path=roots[0], name=name))))

if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    cy = OFFSET_Y / 2 if len(LAYOUTS) == 2 else 0.0
    set_camera_view(eye=np.array([2.4, cy - 1.6, 1.6]), target=np.array([0.2, cy, 0.6]))

if args.paused and not args.headless:
    print('paused: press Play to start physics')
    while app.is_running():
        app.update()
else:
    world.reset()
    import cameras; cams = cameras.add()             # every robot's cameras (MM_CAMERAS=0: off)
    p0 = {lay: np.array(r.get_world_pose()[0], dtype=float) for lay, r in robots}
    k, steps = 0, int(args.duration * 60)
    while app.is_running() and (steps <= 0 or k < steps):
        if args.headless:
            for _ in range(4):
                world.step(render=False)     # 4 physics steps = 1/60 s
        else:
            world.step(render=True)
        k += 1
    if steps > 0:
        for lay, r in robots:
            p = np.array(r.get_world_pose()[0], dtype=float); q = r.get_joint_positions()
            print(f'{lay}: base moved {np.linalg.norm(p - p0[lay]) * 1000:.1f} mm in {k / 60:.1f} s '
                  f'(now at {np.round(p, 3)}), joints finite: {bool(np.all(np.isfinite(q)))}, '
                  f'largest joint angle {np.max(np.abs(q)):.3f}')
app.close()
