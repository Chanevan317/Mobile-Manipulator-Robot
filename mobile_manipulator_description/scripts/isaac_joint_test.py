"""Test every joint of the picking robot in Isaac Sim, one at a time.

For each joint: hold all other joints at zero, move this one 0 -> lower limit -> upper limit -> 0,
and record how closely it follows (tracking error), the range it actually reached, and whether
anything blew up (NaN / runaway). Prints a table and writes joint_test_report.csv.

Run with Isaac Sim's Python, from the package folder:
    <isaac-sim>/python.sh scripts/isaac_joint_test.py                 # all joints, base fixed
    <isaac-sim>/python.sh scripts/isaac_joint_test.py --joint left_wrist_roll --slow
    <isaac-sim>/python.sh scripts/isaac_joint_test.py --headless
    <isaac-sim>/python.sh scripts/isaac_joint_test.py --wheels      # also spin the 4 wheels, base free
    <isaac-sim>/python.sh scripts/isaac_joint_test.py --hold        # hold 3 poses 30 s each, report drift
    <isaac-sim>/python.sh scripts/isaac_joint_test.py --usd robot.usd   # same tests on a saved USD
    <isaac-sim>/python.sh scripts/isaac_joint_test.py --self_collision  # robot parts collide with each other
"""
import argparse, csv, math, os, sys

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--layout', default='prototype', choices=('prototype',), help='robot layout (urdf/mobile_manipulator_<layout>.urdf)')
ap.add_argument('--urdf', default=None, help='URDF path (overrides --layout)')
ap.add_argument('--joint', default='', help='test only joints whose name contains this text')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--slow', action='store_true', help='4 s per move instead of 1.5 s, easier to watch')
ap.add_argument('--wheels', action='store_true', help='also test the wheel velocity drives (base not fixed)')
ap.add_argument('--hold', action='store_true', help='instead of the sweep: hold 3 poses for 30 s each under gravity, report drift')
ap.add_argument('--hold_s', type=float, default=30.0, help='seconds per pose for --hold')
ap.add_argument('--self_collision', action='store_true', help='robot parts collide with each other (see self_collision.py)')
ap.add_argument('--usd', default='', help='test a USD saved by isaac_import_and_test.py --save instead of the URDF '
                                         '(drives are NOT re-applied; the base is free, as saved)')
ap.add_argument('--out', default='joint_test_report.csv')
args = ap.parse_args()
if args.urdf is None:
    args.urdf = os.path.join(here, '..', 'urdf', f'mobile_manipulator_{args.layout}.urdf')

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp
app = SimulationApp({'headless': args.headless})

import numpy as np
import omni.kit.commands, omni.usd
from pxr import UsdLux, Sdf, Gf
try:                                                         # Isaac Sim 4.5 / 5.x
    from isaacsim.asset.importer.urdf import _urdf
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation as Articulation
    from isaacsim.core.utils.types import ArticulationAction
except ImportError:                                          # Isaac Sim 4.0 / 4.2
    from omni.importer.urdf import _urdf
    from omni.isaac.core import World
    from omni.isaac.core.articulations import Articulation
    from omni.isaac.core.utils.types import ArticulationAction

# ---------------------------------------------------------------- import
world = World(stage_units_in_meters=1.0, physics_dt=1 / 240.0, rendering_dt=1 / 60.0)
world.scene.add_ground_plane()
stage = omni.usd.get_context().get_stage()
UsdLux.DistantLight.Define(stage, Sdf.Path('/DistantLight')).CreateIntensityAttr(3000)

if args.usd:
    # Load the saved robot as it is: its own drives, masses and (free) base.
    from pxr import UsdPhysics
    prim = stage.DefinePrim('/mobile_manipulator', 'Xform')
    prim.GetReferences().AddReference(os.path.abspath(args.usd))
    art_path = next(str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI))
    print('loaded', args.usd, 'articulation root', art_path)
else:
    _, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
    cfg.merge_fixed_joints = False
    cfg.fix_base = not args.wheels
    cfg.make_default_prim = True
    cfg.self_collision = False
    cfg.import_inertia_tensor = True
    cfg.convex_decomp = False
    cfg.distance_scale = 1.0
    cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    _, art_path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=os.path.abspath(args.urdf),
                                            import_config=cfg, get_articulation_root=True)
    # gains: force drives from drives.py
    sys.path.insert(0, here)
    from drives import apply_drives
    apply_drives(stage, art_path)
    if args.self_collision:
        from self_collision import setup_self_collision
        print('self-collision on,', setup_self_collision(stage, os.path.abspath(args.urdf)), 'pairs ignored')

robot = world.scene.add(Articulation(prim_path=art_path, name='mobile_manipulator'))
world.reset()
robot.initialize()

names = list(robot.dof_names)
n = len(names)
props = robot.dof_properties
lower = np.array(props['lower'], dtype=float); upper = np.array(props['upper'], dtype=float)
vmax = np.array(props['maxVelocity'], dtype=float)

driven = [i for i, nm in enumerate(names) if 'roller' not in nm and not nm.startswith('wheel') and not nm.endswith('finger_left')]
mimic_followers = [i for i, nm in enumerate(names) if nm.endswith('finger_left')]
if args.joint:
    driven = [i for i in driven if args.joint in names[i]]
wheels = [i for i, nm in enumerate(names) if nm.startswith('wheel') and 'roller' not in nm]

print(f'\n{n} DOFs found. Testing {len(driven)} driven joints' + (f' and {len(wheels)} wheels' if args.wheels else '') + '.\n')
missing = [j for j in ('lift', 'left_shoulder_pan', 'left_elbow_pan', 'left_wrist_pan', 'left_wrist_pitch', 'left_wrist_roll',
                       'left_finger_right', 'right_shoulder_pan', 'right_elbow_pan', 'right_wrist_pan', 'right_wrist_pitch',
                       'right_wrist_roll', 'right_finger_right') if j not in names]
if missing:
    print('WARNING: joints missing after import:', missing)

home = np.zeros(n)
CTRL_HZ = 60
SUBSTEPS = max(1, round((1.0 / CTRL_HZ) / world.get_physics_dt()))


def tick():
    """Advance the simulation by one control period (1/60 s), with or without a window.

    world.step(render=True) runs a whole rendering frame (1/60 s = 4 physics steps), but
    world.step(render=False) runs a single physics step (1/240 s). Without this, headless runs
    went 4x faster than intended and every move hit the joint velocity limits.
    """
    if args.headless:
        for _ in range(SUBSTEPS):
            world.step(render=False)
    else:
        world.step(render=True)


move_t = 4.0 if args.slow else 1.5
settle_t = 0.5


def step_to(target_all, seconds, idx, log):
    """Ease the commanded position of joint idx toward target_all[idx]; others hold. Logs (cmd, actual)."""
    start = robot.get_joint_positions().copy()
    # The eased move peaks at pi/2 * distance / time; keep that under 80% of the joint's velocity limit.
    dist = abs(target_all[idx] - start[idx])
    if np.isfinite(vmax[idx]) and vmax[idx] > 0:
        seconds = max(seconds, math.pi / 2 * dist / (0.8 * vmax[idx]))
    steps = max(1, int(seconds * CTRL_HZ))
    for k in range(steps + int(settle_t * CTRL_HZ)):
        a = min(1.0, (k + 1) / steps)
        s = 0.5 - 0.5 * math.cos(math.pi * a)
        cmd = home.copy()
        cmd[idx] = start[idx] + (target_all[idx] - start[idx]) * s
        for f in mimic_followers:                      # the left jaw always gets the same command as the right
            cmd[f] = cmd[names.index(names[f].replace('finger_left', 'finger_right'))]
        robot.apply_action(ArticulationAction(joint_positions=cmd, joint_indices=np.arange(n)))
        tick()
        q = robot.get_joint_positions()
        log.append((cmd[idx], q[idx], bool(np.all(np.isfinite(q))), q.copy()))
        if not app.is_running():
            return False
    return True


def hold_test():
    """Hold three poses for args.hold_s seconds each; report error and drift per arm joint and the lift.

    error = |actual - command| at the end; drift = how far the joint moved after the first second.
    Pass: drift < 1 deg (1 mm for the lift) and error within the sweep tolerance (3 deg / 2 mm).
    """
    def pose(**kw):
        q = np.zeros(n)
        for side in ('left', 'right'):
            for j, v in kw.items():
                k = f'{side}_{j}' if f'{side}_{j}' in names else j
                if k in names:
                    q[names.index(k)] = v
        for f in mimic_followers:
            q[f] = q[names.index(names[f].replace('finger_left', 'finger_right'))]
        return q
    poses = {
        'zero (arms forward, gripper level)': pose(),
        'stretched (lift top, jaws open)':    pose(lift=0.70, finger_right=0.045),
        'folded (elbow bent, wrist down)':    pose(lift=0.35, elbow_pan=2.0, wrist_pan=-2.0, wrist_pitch=1.5),
    }
    watch = [i for i in driven if not names[i].endswith('finger_right')]
    out = []
    for label, q in poses.items():
        robot.set_joint_positions(q); robot.set_joint_velocities(np.zeros(n))
        q1 = None; peak = np.zeros(n); finite = True
        for k in range(int(args.hold_s * CTRL_HZ)):
            robot.apply_action(ArticulationAction(joint_positions=q, joint_indices=np.arange(n)))
            tick()
            a = robot.get_joint_positions()
            finite = finite and bool(np.all(np.isfinite(a)))
            if k == CTRL_HZ:
                q1 = a.copy()
            if q1 is not None:
                peak = np.maximum(peak, np.abs(a - q1))
            if not app.is_running():
                return out
        a = robot.get_joint_positions()
        print(f'\nHold {args.hold_s:.0f} s: {label}')
        for i in watch:
            lin = names[i] == 'lift'
            scale, unit, tol_d, tol_e = (1000.0, 'mm', 1.0, 2.0) if lin else (180 / math.pi, 'deg', 1.0, 3.0)
            err = abs(a[i] - q[i]) * scale; drift = peak[i] * scale
            status = 'BLEW UP' if not finite else ('DRIFTS' if drift > tol_d else ('SAGS' if err > tol_e else 'OK'))
            print(f'  {names[i]:22s} {unit:3s} target {q[i]*scale:7.1f}  error {err:6.2f}  drift {drift:6.3f}  {status}')
            out.append(dict(pose=label, joint=names[i], unit=unit, target=round(q[i] * scale, 1),
                            error=round(err, 3), drift=round(drift, 4), status=status))
    return out


rows = []
if args.hold:
    rows = hold_test()
    driven = []
for i in driven:
    nm = names[i]
    lo, hi = lower[i], upper[i]
    if not np.isfinite(lo) or not np.isfinite(hi) or hi - lo > 100:
        lo, hi = -1.0, 1.0
    robot.set_joint_positions(home); robot.set_joint_velocities(np.zeros(n))
    log = []
    ok = True
    for tgt in (lo, hi, 0.0):
        t = home.copy(); t[i] = tgt
        ok = step_to(t, move_t, i, log) and ok
    cmd = np.array([l[0] for l in log]); act = np.array([l[1] for l in log]); finite = all(l[2] for l in log)
    err = np.abs(cmd - act)
    lin = (names[i] == 'lift') or ('finger' in nm)
    unit, scale = ('mm', 1000.0) if lin else ('deg', 180.0 / math.pi)
    reached_lo, reached_hi = act.min() * scale, act.max() * scale
    max_err = err.max() * scale
    end_err = abs(act[-1] - 0.0) * scale
    tol = 2.0 if lin else 3.0
    status = 'OK'
    follow = ''
    if nm.endswith('finger_right'):                   # the left finger should mirror this one
        f = names.index(nm.replace('finger_right', 'finger_left'))
        gap = max(abs(l[3][f] - l[3][i]) for l in log) * 1000.0
        follow = f'  left finger off by up to {gap:.1f} mm'
        if gap > 2.0:
            status = 'LEFT FINGER DOES NOT FOLLOW'
    if not finite:
        status = 'BLEW UP'
    elif reached_lo > lo * scale + tol or reached_hi < hi * scale - tol:
        status = 'SHORT OF LIMIT'
    elif end_err > tol:
        status = 'DOES NOT RETURN'
    elif max_err > 4 * tol:
        status = 'LAGS (tune gains)'
    rows.append(dict(joint=nm, unit=unit, limit_lo=round(lo * scale, 1), limit_hi=round(hi * scale, 1),
                     reached_lo=round(reached_lo, 1), reached_hi=round(reached_hi, 1),
                     max_tracking_error=round(max_err, 2), final_error=round(end_err, 2), status=status))
    print(f"{nm:28s} {unit:3s} limit [{lo*scale:8.1f},{hi*scale:8.1f}]  reached [{reached_lo:8.1f},{reached_hi:8.1f}]  "
          f"max err {max_err:6.2f}  end err {end_err:5.2f}  {status}{follow}")
    if not app.is_running():
        break

if args.wheels and app.is_running():
    print('\nWheels: 3 s at +5 rad/s each, then the base should have moved.')
    p0 = robot.get_world_pose()[0]
    for i in wheels:
        for _ in range(180):
            v = np.zeros(n); v[i] = 5.0
            robot.apply_action(ArticulationAction(joint_velocities=v, joint_indices=np.arange(n)))
            tick()
        print(f'  {names[i]:12s} speed now {robot.get_joint_velocities()[i]:6.2f} rad/s')
    print('  base moved', np.round(robot.get_world_pose()[0] - p0, 3), 'm')

with open(args.out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ['joint']); w.writeheader(); w.writerows(rows)
bad = [r for r in rows if r['status'] != 'OK']
print(f"\n{len(rows) - len(bad)}/{len(rows)} joints OK. Report: {os.path.abspath(args.out)}")
for r in bad:
    print('  needs a look:', r['joint'], '-', r['status'])
if not args.headless:
    print('Window stays open; close it to exit.')
    while app.is_running():
        world.step(render=True)
app.close()
