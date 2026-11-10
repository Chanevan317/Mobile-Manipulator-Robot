"""Drive the mecanum base in Isaac Sim and measure how well it moves.

The base is free on the ground, the arms hold the zero pose and the lift sits at 0.35 m.
For each motion (forward, backward, left, right, turn left, turn right) the script
commands body speeds through standard mecanum wheel mixing for a few seconds, then reports:

  speed    achieved body speed / commanded (1.00 is perfect), measured after a 1 s ramp
  cross    unwanted motion in the other directions (m/s or rad/s)
  bounce   peak-to-peak height of base_link (mm)
  tilt     largest roll/pitch of the base (deg)

Run with Isaac Sim's Python, from the package folder:
    <isaac-sim>/python.sh scripts/isaac_base_test.py --headless
    <isaac-sim>/python.sh scripts/isaac_base_test.py --speed 0.2 --seconds 5
"""
import argparse, math, os, sys

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--layout', default='prototype', choices=('prototype',), help='robot layout (urdf/mobile_manipulator_<layout>.urdf)')
ap.add_argument('--urdf', default=None, help='URDF path (overrides --layout)')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--speed', type=float, default=0.3, help='linear speed, m/s')
ap.add_argument('--turn', type=float, default=0.6, help='turn rate, rad/s')
ap.add_argument('--self_collision', action='store_true', help='robot parts collide with each other (see self_collision.py)')
ap.add_argument('--seconds', type=float, default=4.0, help='seconds per motion')
args = ap.parse_args()
if args.urdf is None:
    args.urdf = os.path.join(here, '..', 'urdf', f'mobile_manipulator_{args.layout}.urdf')

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless})

import numpy as np
import omni.kit.commands, omni.usd
from pxr import UsdLux, Sdf
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.types import ArticulationAction
sys.path.insert(0, here)
from drives import apply_drives

CTRL_HZ = 60
world = World(stage_units_in_meters=1.0, physics_dt=1 / 240.0, rendering_dt=1 / CTRL_HZ)
world.scene.add_ground_plane()
stage = omni.usd.get_context().get_stage()
UsdLux.DistantLight.Define(stage, Sdf.Path('/DistantLight')).CreateIntensityAttr(3000)

_, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
cfg.merge_fixed_joints = False
cfg.fix_base = False
cfg.make_default_prim = True
cfg.self_collision = False
cfg.import_inertia_tensor = True
cfg.convex_decomp = False
cfg.distance_scale = 1.0
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
_, art_path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=os.path.abspath(args.urdf),
                                        import_config=cfg, get_articulation_root=True)
apply_drives(stage, art_path)
if args.self_collision:
    from self_collision import setup_self_collision
    print('self-collision on,', setup_self_collision(stage, os.path.abspath(args.urdf)), 'pairs ignored')
robot = world.scene.add(Articulation(prim_path=art_path, name='mobile_manipulator'))
world.reset()
robot.initialize()

names = list(robot.dof_names)
n = len(names)
SUBSTEPS = max(1, round((1.0 / CTRL_HZ) / world.get_physics_dt()))


def tick():
    """One control period (1/60 s). world.step(render=False) is only one physics step."""
    if args.headless:
        for _ in range(SUBSTEPS):
            world.step(render=False)
    else:
        world.step(render=True)


# ---------------------------------------------------------------- mecanum kinematics
R_WHEEL = 0.05                    # wheel radius (m), from urdf_build.py
LX, LY = 0.17, 0.222              # wheel centre offsets from the base centre (m)
wheel = {w: names.index(f'wheel_{w}_joint') for w in ('fl', 'fr', 'rl', 'rr')}
hold_q = np.zeros(n)
hold_q[names.index('lift')] = 0.35
arm_idx = [i for i, nm in enumerate(names) if 'roller' not in nm and not nm.startswith('wheel')]


def wheel_speeds(vx, vy, wz):
    """Body speeds (m/s, m/s, rad/s) -> wheel speeds (rad/s), X-pattern rollers, +wheel = roll forward."""
    k = LX + LY
    return {'fl': (vx - vy - k * wz) / R_WHEEL, 'fr': (vx + vy + k * wz) / R_WHEEL,
            'rl': (vx + vy - k * wz) / R_WHEEL, 'rr': (vx - vy + k * wz) / R_WHEEL}


def yaw_rp(qwxyz):
    w, x, y, z = qwxyz
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return yaw, roll, pitch


def drive(vx, vy, wz, seconds):
    vel = np.zeros(n)
    for w, s in wheel_speeds(vx, vy, wz).items():
        vel[wheel[w]] = s
    log = []
    for k in range(int(seconds * CTRL_HZ)):
        robot.apply_action(ArticulationAction(joint_positions=hold_q[arm_idx], joint_indices=np.array(arm_idx)))
        robot.apply_action(ArticulationAction(joint_velocities=vel[list(wheel.values())],
                                              joint_indices=np.array(list(wheel.values()))))
        tick()
        p, q = robot.get_world_pose()
        log.append((k / CTRL_HZ, np.array(p, dtype=float), yaw_rp(np.array(q, dtype=float)),
                    bool(np.all(np.isfinite(robot.get_joint_positions())))))
        if not app.is_running():
            break
    return log


# settle on the ground first
drive(0, 0, 0, 2.0)

S, T = args.speed, args.turn
motions = [('forward', (S, 0, 0)), ('backward', (-S, 0, 0)), ('left', (0, S, 0)), ('right', (0, -S, 0)),
           ('turn left', (0, 0, T)), ('turn right', (0, 0, -T))]
print(f'\nMecanum base test: {args.seconds:.1f} s per motion, speed {S} m/s, turn {T} rad/s\n')
print(f"{'motion':11s} {'speed':>6s} {'cross':>14s} {'bounce mm':>10s} {'tilt deg':>9s}  status")
results = []
for label, (vx, vy, wz) in motions:
    log = drive(vx, vy, wz, args.seconds)
    if not log:
        break
    finite = all(l[3] for l in log)
    steady = [l for l in log if l[0] >= 1.0]
    t0, p0, (y0, _, _), _ = steady[0]
    t1, p1, (y1, _, _), _ = steady[-1]
    dt = t1 - t0
    dyaw = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))
    # body-frame average velocity over the steady part (heading taken at the midpoint)
    ym = y0 + dyaw / 2
    dw = p1[:2] - p0[:2]
    bvx = (math.cos(ym) * dw[0] + math.sin(ym) * dw[1]) / dt
    bvy = (-math.sin(ym) * dw[0] + math.cos(ym) * dw[1]) / dt
    bwz = dyaw / dt
    meas = np.array([bvx, bvy, bwz]); cmd = np.array([vx, vy, wz])
    main = int(np.argmax(np.abs(cmd)))
    ratio = meas[main] / cmd[main]
    cross = np.delete(meas, main)
    z = np.array([l[1][2] for l in log]); bounce = (z.max() - z.min()) * 1000.0
    tilt = max(max(abs(l[2][1]), abs(l[2][2])) for l in log) * 180 / math.pi
    status = 'BLEW UP' if not finite else (
        'OK' if (0.8 <= ratio <= 1.2 and np.all(np.abs(cross) < 0.05) and bounce < 5 and tilt < 2) else 'POOR')
    results.append((label, ratio, cross, bounce, tilt, status))
    print(f'{label:11s} {ratio:6.2f} {cross[0]:+6.3f} {cross[1]:+6.3f} {bounce:10.1f} {tilt:9.2f}  {status}')
    drive(0, 0, 0, 1.0)

print('\nPass: speed 0.8-1.2, cross < 0.05, bounce < 5 mm, tilt < 2 deg.')
print(f"{sum(r[5] == 'OK' for r in results)}/{len(results)} motions OK.")
if not args.headless:
    print('Window stays open; close it to exit.')
    while app.is_running():
        world.step(render=True)
app.close()
