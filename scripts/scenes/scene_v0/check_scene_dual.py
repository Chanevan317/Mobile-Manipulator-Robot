"""Check scene_v0 (scene.py) with the DUAL-arm prototype: the dual version of check_scene_mono.py.

Part 0, reach (no physics, scara_ik 'prototype'): the pair stations (stations_dual.py): two task products on one board,
the left arm picking one and the right arm the other from ONE base pose at ONE lift height (both arms on one carriage);
kinematics + joint limits + footprint + what stands beside the robot (low obstacles: the arms swing out above them).
Part 1, scene: everything is dropped from 3 mm and runs 3 s (as the mono check), with the dual robot standing there.
Part 2, grasps: the dual robot's real test is both arms picking two products at once and putting them back, which needs
the robot to drive between stations (this check holds the base fixed), so it is its own script:
    OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/scenes/scene_v0/pick_to_tote_dual.py --headless --all --place

Run (Isaac Sim 5.1; ask the user before running, CLAUDE.md):
    OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 conda run --no-capture-output -n env_isaaclab python scripts/scenes/scene_v0/check_scene_dual.py --headless
    ... --only ketchup,chocolate      only products whose name contains one of these (Part 1)
    (with a window it stays open to look at, physics paused; --res WxH to override the window size)
"""
import argparse, math, os, sys
ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap.add_argument('--urdf', default=RP.urdf('prototype'))
ap.add_argument('--headless', action='store_true')
ap.add_argument('--no_grasp', action='store_true')
ap.add_argument('--only', default='')
ap.add_argument('--no_self_collision', action='store_true')
ap.add_argument('--physics_hz', type=int, default=240, help='PhysX steps per second (scene_robot.SIM uses the arm-training rate)')
ap.add_argument('--res', default='', help='render / window size WxH (default: the screen size, from xrandr)')
ap.add_argument('--fill', type=float, default=1.0, help='filler-product density scale (0 = only the 15 task products)')
ap.add_argument('--seed', type=int, default=0, help='filler layout')
ap.add_argument('--shot', default='', help='save viewport pictures to PATH_<view>.png after the scene check, then quit')
args = ap.parse_args()


import sim_app                                        # noqa: E402  (window at the screen's resolution, --res)
app = sim_app.start(args.headless, args.res)

import numpy as np
import omni.usd
from pxr import UsdGeom, Gf
from isaacsim.core.prims import SingleXFormPrim
sys.path.insert(0, here)
from proto_task import Task
import assets as SA
import scene as SC
import grasp_plan_mono as G
import mono_poses as MP

LAYOUT = 'prototype'
task = Task(app, args.urdf, args.headless, fix_base=True, self_collision=not args.no_self_collision, layout=LAYOUT,
            physics_hz=args.physics_hz)
task.profile, task.settle_mode = 'trap', 'still'
stage = omni.usd.get_context().get_stage()
items = SC.build(stage, fill=args.fill, seed=args.seed)
# plain prims, NOT physics objects: any rigid-body view on these bodies (SingleRigidPrim / RigidPrim, registered or not,
# even deleted again) made every physics step ~5-7x slower (2026-11-17). Poses are read / written through USD (PhysX
# writes its poses back and takes our writes as teleports), velocities zeroed with PhysX sleep / wake, masses from
# PhysX's property query.
bodies = {n: [SingleXFormPrim(b) for b in it['bodies']]
          for n, it in items.items() if it['role'] in ('product', 'basket') or it.get('dynamic')}
FURN = [n for n, it in items.items() if it['role'] in ('shelf', 'table')]
task.start()
# Isaac resets to the zero pose, where the arms point out under the table top: start with both arms folded
import scene_robot as R
task.set_pose(R.stow_joints(LAYOUT))
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([-4.6, -0.4, 2.3]), target=np.array([0.0, 0.0, 0.8]))   # from the room's open side


def mat(p, q):
    m = Gf.Matrix4d(); m.SetRotate(Gf.Quatd(float(q[0]), Gf.Vec3d(float(q[1]), float(q[2]), float(q[3])))); m.SetTranslateOnly(Gf.Vec3d(*map(float, p)))
    return m


def pq(m):
    t = m.ExtractTranslation(); r = m.ExtractRotationQuat(); i = r.GetImaginary()
    return np.array([t[0], t[1], t[2]]), np.array([r.GetReal(), i[0], i[1], i[2]])


xc = UsdGeom.XformCache()
S0 = {n: xc.GetLocalToWorldTransform(stage.GetPrimAtPath(it['slot'])) for n, it in items.items()}
F0 = {n: xc.GetLocalToWorldTransform(stage.GetPrimAtPath(items[n]['bodies'][0])).ExtractTranslation() for n in FURN}
REL = {n: [mat(*b.get_world_pose()) * S0[n].GetInverse() for b in bodies[n]] for n in bodies}    # body in its slot frame


def slot_now(n):
    """Where the slot frame is now, from the first body's physics pose."""
    return REL[n][0].GetInverse() * mat(*bodies[n][0].get_world_pose())


def bottom_and_tilt(n):
    S = slot_now(n); lo, hi = items[n]['range']
    p = S.Transform(Gf.Vec3d((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]))
    z = S.TransformDir(Gf.Vec3d(0, 0, 1)); tilt = math.degrees(math.acos(max(-1.0, min(1.0, z[2] / z.GetLength()))))
    return np.array([p[0], p[1], p[2]]), tilt


from pxr import PhysicsSchemaTools
from omni.physx import get_physx_simulation_interface, get_physx_property_query_interface
STAGE_ID = omni.usd.get_context().get_stage_id()


def body_id(b):
    return PhysicsSchemaTools.sdfPathToInt(b.prim_path)


def teleport(n, xy, yaw, z_bottom):
    S = SC.slot_matrix(xy, yaw, z_bottom, items[n]['range'])
    sim = get_physx_simulation_interface()
    for b, rel in zip(bodies[n], REL[n]):
        p, q = pq(rel * S); b.set_world_pose(position=p, orientation=q)
        sim.put_to_sleep(STAGE_ID, body_id(b)); sim.wake_up(STAGE_ID, body_id(b))     # sleeping zeroes the velocities


def mass(b):
    """The mass PhysX uses for this body (property query: no physics view needed)."""
    from omni.physx.bindings._physx import PhysxPropertyQueryMode
    out = {}
    get_physx_property_query_interface().query_prim(
        stage_id=STAGE_ID, prim_id=body_id(b), query_mode=PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,
        rigid_body_fn=lambda r: out.setdefault('m', r.mass), timeout_ms=2000)
    return out['m']


products = [n for n, it in items.items() if it['role'] == 'product']

# ---------------------------------------------------------------- part 0: reach
print('\n=== Part 0: dual-arm pair stations (kinematics, footprint, what stands beside the robot) ===')
import stations_dual as SD
pst, tried = SD.compute()
for n_, s_ in pst.items():
    if n_ == 'home':
        continue
    side = f", arms swing out above {s_['side_min_z']:.2f} m" if s_['side_min_z'] else ''
    print(f"{n_:62s} base ({s_['x']:+.2f}, {s_['y']:+.2f}) facing {s_['yaw']:+.0f} deg, grip z {s_['grip_z']:.3f}{side}  OK")
for a_, b_, why in tried:
    print(f'  not a pair: {a_} + {b_}: {why}')
print(f'reach: {len(pst) - 1} pair station(s) for {sum(1 for x in items.values() if x["role"] == "product")} task products')
import scara_ik as K
K.use(LAYOUT)

# ---------------------------------------------------------------- part 1: scene
task.wait(2.5)
P25 = {n: bottom_and_tilt(n)[0] for n in products}
task.wait(0.5)
print('\n=== Part 1: scene (dropped from 3 mm, 3 s) ===')
ok1 = []
for n in products:
    it = items[n]; z, x, y, _ = it['rest']; p, tilt = bottom_and_tilt(n)
    m = sum(mass(b) for b in bodies[n]); want = SA.PRODUCTS[it['pkg']]['mass']
    v = np.linalg.norm(p - P25[n]) / 0.5          # over the last 0.5 s (PhysX reports ~8 mm/s on resting bodies)
    ok = abs(p[2] - z) < 0.005 and tilt < 5 and math.hypot(p[0] - x, p[1] - y) < 0.010 and abs(m - want) < 0.01 * want + 1e-4 and v < 0.002
    ok1.append(ok)
    print(f'{n:28s} on {z:.3f}: bottom {p[2]:.4f} tilt {tilt:4.1f} deg moved {math.hypot(p[0]-x, p[1]-y)*1000:5.1f} mm '
          f'speed {v*1000:4.1f} mm/s  mass {m:.3f} kg (want {want:.3f})  {"OK" if ok else "FAIL"}')
for n in FURN:
    d = (UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(items[n]['bodies'][0])).ExtractTranslation() - F0[n]).GetLength()
    print(f'{n:28s} static (rigid body disabled), body moved {d*1000:.1f} mm  {"OK" if d < 1e-4 else "FAIL"}'); ok1.append(d < 1e-4)
p, tilt = bottom_and_tilt('crate')
walls = [pq(mat(*b.get_world_pose()) * mat(*bodies['crate'][0].get_world_pose()).GetInverse())[1] for b in bodies['crate']]
rel0 = [pq(r * REL['crate'][0].GetInverse())[1] for r in REL['crate']]
wdev = max(2 * math.degrees(math.acos(min(1.0, abs(float(np.dot(a, b)))))) for a, b in zip(walls, rel0))
okc = abs(p[2] - SC.TABLE_TOP) < 0.005 and tilt < 2 and wdev < 3; ok1.append(okc)
print(f'{"crate":28s} bottom {p[2]:.4f} (table {SC.TABLE_TOP}), tilt {tilt:.1f} deg, walls off upright by up to {wdev:.1f} deg  {"OK" if okc else "FAIL"}')
fill_dyn = [n for n, it in items.items() if it['role'] == 'filler' and it['dynamic']]
fill_bad = []
for n in fill_dyn:
    z, x, y, _ = items[n]['rest']; p, tilt = bottom_and_tilt(n)
    if abs(p[2] - z) > 0.005 or tilt > 5 or math.hypot(p[0] - x, p[1] - y) > 0.010:
        fill_bad.append(f'{n} (tilt {tilt:.0f} deg, moved {math.hypot(p[0]-x, p[1]-y)*1000:.0f} mm, bottom {p[2]:.3f} on {z:.3f})')
n_fix = sum(1 for it in items.values() if it['role'] == 'filler' and not it['dynamic'])
print(f'fillers: {len(fill_dyn) - len(fill_bad)}/{len(fill_dyn)} with physics stand where placed ({n_fix} more fixed in place)'
      f'  {"OK" if not fill_bad else "FAIL"}'); ok1.append(not fill_bad)
for b in fill_bad[:10]:
    print('   ', b)
gaps = []                                          # task product <-> nearest filler on the same board (planned spots)
for n, it in items.items():
    if it['role'] != 'product' or it['rest'][3] == 0.0:
        continue
    z, x, y, hd = it['rest']; wt = min(SA.size(it['pkg'])[:2]) / 1000
    for m, f in items.items():
        if f['role'] == 'filler' and abs(f['rest'][0] - z) < 0.01 and f['rest'][3] == hd and abs(f['rest'][2] - y) < 0.3:
            gaps.append((abs(f['rest'][1] - x) - wt / 2 - min(SA.size(f['pkg'])[:2]) / 2000, n))
if gaps:
    g, gn = min(gaps); okg = g >= SC.TASK_GAP - 0.006           # fillers turn up to 5 deg
    print(f'task products: nearest filler {g*1000:.0f} mm from the side of {gn} (want >= {SC.TASK_GAP*1000:.0f})  {"OK" if okg else "FAIL"}')
    ok1.append(okg)
print(f'scene: {sum(ok1)}/{len(ok1)} OK')

if args.shot:
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    vp = get_active_viewport()
    for view, eye, tgt in (('overview', (-4.6, -0.4, 2.3), (0.0, 0.0, 0.8)), ('racks', (-1.2, -0.6, 1.3), (-1.2, 1.4, 0.9)),
                           ('shelves', (-1.2, 0.7, 1.3), (-1.2, -1.5, 0.9)), ('table', (-0.9, 0.0, 1.5), (0.8, 0.0, 0.85))):
        set_camera_view(eye=np.array(eye), target=np.array(tgt))
        for _ in range(60):
            app.update()
        capture_viewport_to_file(vp, f'{args.shot}_{view}.png')
        for _ in range(20):
            app.update()
    print('pictures saved:', args.shot + '_*.png', flush=True)
    app.close(); sys.exit(0)


# ---------------------------------------------------------------- part 2: grasps
def in_tcp(n):
    tp, tq = task.tcp['left'].get_world_pose(); p, _ = bodies[n][0].get_world_pose()
    return np.array((mat(p, (1, 0, 0, 0)) * mat(tp, tq).GetInverse()).ExtractTranslation())


if False:   # Part 2: pick_to_tote_dual.py --all --place (both arms, the base drives)
    print('\n=== Part 2: grasp + lift 100 mm + hold 2 s + put back (mono arm, fixed base) ===')
    names = [n for n in products if not args.only or any(k in n for k in args.only.split(','))]
    plans = {n: G.table_plan(items[n]['pkg'], SC.TABLE_TOP, SC.TABLE_EDGE) for n in names}
    order = sorted(names, key=lambda n: (plans[n] is None or plans[n]['top'], n))
    task.lift_to(0.55)                    # straight up from the stow fold (clear at any lift), then over the table
    res = []
    for n in order:
        P = plans[n]
        if P is None:
            print(f'{n:28s} no reachable grasp found  FAIL'); res.append(False); continue
        pkg = items[n]['pkg']; yawp = SC.product_yaw(pkg, math.degrees(P['yaw']))
        jopen = min(0.045, P['narrow'] / 2 + 0.015)
        try:
            task.set_jaw('left', jopen)
            task.joint_move('left', task.ik('left', P['a'], P['elbow']))
            teleport(n, P['centre'], yawp, SC.TABLE_TOP + 0.002); task.wait(0.6)
            p_pick, _ = bottom_and_tilt(n)
            task.cart_move('left', P['a'], P['b'], P['elbow'])
            task.jaw('left', 0.0, 1.0)
            q = task.robot.get_joint_positions()
            gap = q[task.idx['left_finger_right']] + q[task.idx['left_finger_left']]
            r0 = in_tcp(n); z0 = p_pick[2]
            task.cart_move('left', P['b'], P['c'], P['elbow']); task.wait(2.0)
            rise = bottom_and_tilt(n)[0][2] - z0; slip = np.linalg.norm(in_tcp(n) - r0)
            task.cart_move('left', P['c'], (*P['b'][:2], P['b'][2] + 0.003, *P['b'][3:]), P['elbow'])
            task.jaw('left', jopen, 0.8)
            tilt_rel = bottom_and_tilt(n)[1]
            task.cart_move('left', P['b'], P['a'], P['elbow'])
            p_end, tilt = bottom_and_tilt(n); moved = math.hypot(*(p_end[:2] - p_pick[:2]))
            ok = rise >= 0.085 and slip < 0.005 and moved < 0.015 and tilt < 5
            print(f'{n:28s} {"top " if P["top"] else "side"} {SA.PRODUCTS[pkg]["mass"]*1000:3.0f} g, narrow {P["narrow"]*1000:3.0f} mm: '
                  f'jaw gap {gap*1000:5.1f} mm, rose {rise*1000:5.1f} mm, slip {slip*1000:4.1f} mm, ended {moved*1000:4.1f} mm '
                  f'from the pick spot, tilt {tilt:4.1f} deg (at release {tilt_rel:4.1f})  {"OK" if ok else "FAIL"}', flush=True)
            res.append(ok)
        except RuntimeError as e:
            print(f'{n:28s} {e}  FAIL'); res.append(False)
        z, x, y, hd = items[n]['rest']; teleport(n, (x, y), items[n]['yaw'], z + SC.DROP); task.wait(0.3)
    print(f'grasps: {sum(res)}/{len(res)} OK')
    task.summary()
print('\nPart 2 (grasps, both arms at once): run pick_to_tote_dual.py --headless --all --place')

if not args.headless:
    import time

    def fps(seconds, physics):
        n, t0 = 0, time.time()
        while app.is_running() and time.time() - t0 < seconds:
            task.world.step(render=True) if physics else app.update()
            n += 1
        return n / max(time.time() - t0, 1e-6)
    print(f'\nframe rate with physics running: {fps(3.0, True):.1f} fps', flush=True)
    task.world.pause()          # viewing only: no physics per frame, so the camera moves freely
    print(f'frame rate with physics paused:  {fps(3.0, False):.1f} fps', flush=True)
    print('Physics paused (press Play in the window to resume). Window stays open; close it to exit.', flush=True)
    while app.is_running():
        print(f'viewing: {fps(10.0, False):.1f} fps', flush=True)
app.close()
