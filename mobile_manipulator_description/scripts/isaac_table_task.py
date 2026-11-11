"""Table-to-table task (prototype layout): drive to table A, both arms pick a product each (top-down) and put it in
the tote, drive to table B, both arms take their product out of the tote and place it on table B.
By default both arms work at the same time (one shared lift: both products are at the same height);
--sequential runs one arm after the other (the waiting arm parks straight out to its side).

The robot (free base, mecanum wheels, self-collision on) starts at the origin facing +x.
  table A: front edge at x = 2.0, faces the robot (-x);  table B: front edge at y = -1.5, faces +y (90 deg turn)
  the robot stops with the table edge 0.36 m in front of its centre (0.10 m past the bumper)
  products: 45 x 45 x 70 mm boxes, 150 g, standing upright; one per arm
Steps:
   1. drive to table A
   2. both arms: table A -> tote          (--sequential: left, then right)
   3. stow both arms, drive to table B
   4. both arms: tote -> table B
Moves: lift first, then swing over a table; swing away, then lower; in and out of the stow fold with the elbow
alone. Checked offline with the URDF meshes, the table and the held products, with both arms moving together
(arm against arm too) for 0.60 and 0.75 m tables. Each joint
move is time-synchronised (proto_task.Task.joint_move). Hand-down picks need table <= 0.84 m (lift travel); the
vertical approach shrinks to fit (0.10 m on a 0.75 m table).

Run with Isaac Sim's Python, from the package folder:
    python scripts/isaac_table_task.py               # window
    python scripts/isaac_table_task.py --headless
    python scripts/isaac_table_task.py --headless --table_h 0.60
"""
import argparse, math, os, sys

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--urdf', default=os.path.join(here, '..', 'urdf', 'mobile_manipulator_prototype.urdf'))
ap.add_argument('--headless', action='store_true')
ap.add_argument('--table_h', type=float, default=0.75, help='height of both tables (m), <= 0.84')
ap.add_argument('--no_self_collision', action='store_true')
ap.add_argument('--sequential', action='store_true', help='one arm after the other instead of both at once')
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless})

import numpy as np
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
sys.path.insert(0, here)
from proto_task import Task
import proto_poses as PP

D = math.radians
P = math.pi / 2
ITEM = np.array([0.045, 0.045, 0.070]); ITEM_MASS = 0.15
EDGE, H = PP.TABLE_EDGE, args.table_h
UP_T = min(0.12, 0.88 + 0.4109 - 0.215 - (H + 0.035 + 0.005))    # vertical approach the 0.885 m lift travel allows
UP_TOTE = PP.TOTE_UP
JAW_OPEN, JAW_CLOSED = 0.035, 0.0
SIDES = (('left', 1), ('right', -1))
SPOTS = {s: dict(pick=(PP.TABLE_PICK[0], g * PP.TABLE_PICK[1]), place=(PP.TABLE_PLACE[0], g * PP.TABLE_PLACE[1]),
                 yaw=g * PP.TABLE_YAW, elbow=g * PP.TABLE_ELBOW) for s, g in SIDES}
GOAL_A = np.array([2.0 - EDGE, 0.0, 0.0])          # base pose (x, y, yaw) at table A
GOAL_B = np.array([1.0, -1.5 + EDGE, -P])          # at table B (turned 90 deg)


def world(goal, xy):
    c, s = math.cos(goal[2]), math.sin(goal[2])
    return np.array([goal[0] + c * xy[0] - s * xy[1], goal[1] + s * xy[0] + c * xy[1]])


task = Task(app, args.urdf, args.headless, fix_base=False, self_collision=not args.no_self_collision)
w = task.world
task.profile = 'trap'        # full allowed speed after a 0.3 s ramp (same limits)
task.settle_mode = 'still'   # a move is done once the joints stop (the wrist pitch sags under a product)
WOOD = np.array([0.62, 0.48, 0.33])
w.scene.add(FixedCuboid(prim_path='/World/tableA', name='tableA', position=np.array([2.35, 0.0, H / 2]),
                        scale=np.array([0.70, 1.20, H]), color=WOOD))
w.scene.add(FixedCuboid(prim_path='/World/tableB', name='tableB', position=np.array([1.0, -1.85, H / 2]),
                        scale=np.array([1.20, 0.70, H]), color=WOOD))
items, START = {}, {}
for s, col in (('left', (0.2, 0.6, 0.9)), ('right', (0.9, 0.4, 0.2))):
    START[s] = world(GOAL_A, SPOTS[s]['pick'])
    items[s] = w.scene.add(DynamicCuboid(prim_path=f'/World/product_{s}', name=f'product_{s}',
                                         position=np.array([*START[s], H + ITEM[2] / 2]), scale=ITEM, mass=ITEM_MASS, color=np.array(col)))
TARGET = {s: world(GOAL_B, SPOTS[s]['place']) for s, _ in SIDES}
task.start()
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([-0.6, 1.6, 2.4]), target=np.array([1.4, -0.6, 0.5]))

task.set_pose({**PP.full('left', PP.STOW['left']), **PP.full('right', PP.STOW['right'])})
for s, _ in SIDES:
    items[s].set_world_pose(position=np.array([*START[s], H + ITEM[2] / 2]), orientation=np.array([1.0, 0, 0, 0]))
task.wait(1.0)
zi = H + ITEM[2] / 2                           # TCP at the product centre: fingertips 23 mm above the table
print(f'\nTable-to-table task ({"one arm at a time" if args.sequential else "both arms at once"}): tables {H:.2f} m (vertical approach {UP_T*1000:.0f} mm), product {ITEM[0]*1000:.0f} x '
      f"{ITEM[1]*1000:.0f} x {ITEM[2]*1000:.0f} mm, {ITEM_MASS*1000:.0f} g, self-collision {'off' if args.no_self_collision else 'on'}\n")


def down_up(side, xy, z, yaw, elbow, close, up):
    hi = (xy[0], xy[1], z + up, yaw, P); lo = (xy[0], xy[1], z, yaw, P)
    task.cart_move(side, hi, lo, elbow)
    task.jaw(side, JAW_CLOSED if close else JAW_OPEN, 0.8)
    task.cart_move(side, lo, hi, elbow)


def to_stow(side):
    task.joint_move(side, PP.ELBOW_OPEN[side], lift=False)          # swing to the elbow-open pose at the current height
    task.joint_move(side, {'lift': PP.STOW[side]['lift']}); task.joint_move(side, PP.STOW[side])


def table_to_tote(side):
    other = 'right' if side == 'left' else 'left'
    sp, tote = SPOTS[side], PP.TOTE[side]
    task.route(other, PP.ROUTE[other]['stow_to_park'], PP.PARK[other])
    task.jaw(side, JAW_OPEN, 0.5)
    c0 = task.to_base(task.obj_state(items[side])[0])                # where the product actually is (a camera would see it)
    q_th = task.ik(side, (c0[0], c0[1], zi + UP_T, sp['yaw'], P), sp['elbow'])
    task.joint_move(side, PP.ELBOW_OPEN[side]); task.joint_move(side, {'lift': q_th['lift']}); task.joint_move(side, q_th)
    down_up(side, c0[:2], zi, sp['yaw'], sp['elbow'], close=True, up=UP_T)
    rise = (task.to_base(task.obj_state(items[side])[0])[2] - c0[2]) * 1000
    task.report(f'{side}: pick from table A', f'product lifted {rise:.0f} mm (gripper {UP_T*1000:.0f})', ok=rise > UP_T * 1000 - 25)
    zt = PP.ZT_UPRIGHT
    q_tu = task.ik(side, (tote['x'], tote['y'], zt + UP_TOTE, tote['yaw'], P), tote['elbow'])
    task.joint_move(side, q_tu, lift=False); task.joint_move(side, {'lift': q_tu['lift']})   # swing high, then lower
    down_up(side, (tote['x'], tote['y']), zt, tote['yaw'], tote['elbow'], close=False, up=UP_TOTE)
    c, up = task.obj_state(items[side]); cb = task.to_base(c); x0, x1, y0, y1 = PP.TOTE_INNER
    task.report(f'{side}: place in tote', f'product at ({cb[0]:+.3f}, {cb[1]:+.3f}, {cb[2]:.3f}) m, {"upright" if up[2] > 0.9 else "tipped"}',
                ok=x0 < cb[0] < x1 and y0 < cb[1] < y1 and cb[2] < 0.376)
    task.jaw(side, JAW_CLOSED, 0.4)
    to_stow(side)
    task.route(other, PP.ROUTE[other]['park_to_stow'], PP.STOW[other])


def tote_to_table(side):
    other = 'right' if side == 'left' else 'left'
    sp, tote = SPOTS[side], PP.TOTE[side]
    task.route(other, PP.ROUTE[other]['stow_to_park'], PP.PARK[other])
    task.jaw(side, JAW_OPEN, 0.5)
    c, _ = task.obj_state(items[side]); cb = task.to_base(c)
    q_hi = task.ik(side, (cb[0], cb[1], cb[2] + UP_TOTE, tote['yaw'], P), tote['elbow'])
    task.joint_move(side, PP.ELBOW_OPEN[side]); task.joint_move(side, {'lift': q_hi['lift']}); task.joint_move(side, q_hi)
    down_up(side, cb[:2], cb[2], tote['yaw'], tote['elbow'], close=True, up=UP_TOTE)
    rise = (task.obj_state(items[side])[0][2] - c[2]) * 1000
    task.report(f'{side}: pick from tote', f'product lifted {rise:.0f} mm', ok=rise > UP_TOTE * 1000 - 25)
    tb = task.to_base(np.array([*TARGET[side], zi]))
    q_ph = task.ik(side, (tb[0], tb[1], zi + 0.005 + UP_T, sp['yaw'], P), sp['elbow'])
    task.joint_move(side, {'lift': q_ph['lift']}); task.joint_move(side, q_ph, lift=False)     # raise first, then swing
    down_up(side, tb[:2], zi + 0.005, sp['yaw'], sp['elbow'], close=False, up=UP_T)
    c, up = task.obj_state(items[side])
    err = np.linalg.norm(c[:2] - TARGET[side]) * 1000
    task.report(f'{side}: place on table B', f'product {err:.0f} mm from the target, height {c[2]:.3f} m, '
                f'{"upright" if up[2] > math.cos(D(10)) else "NOT upright"}',
                ok=err < 20 and abs(c[2] - (H + ITEM[2] / 2)) < 0.01 and up[2] > math.cos(D(10)))
    task.jaw(side, JAW_CLOSED, 0.4)
    to_stow(side)
    task.route(other, PP.ROUTE[other]['park_to_stow'], PP.STOW[other])


def both_unfold():
    """Out of the stow fold one arm at a time (unfolding together crosses the forearms)."""
    task.joint_move2(PP.ELBOW_OPEN['left'], PP.STOW['right']); task.joint_move2(PP.ELBOW_OPEN['left'], PP.ELBOW_OPEN['right'])


def both_to_stow():
    task.joint_move2(PP.ELBOW_OPEN['left'], PP.ELBOW_OPEN['right'])         # swing to elbow open at the current height
    task.lift_to(PP.STOW['left']['lift'])
    task.joint_move2(PP.ELBOW_OPEN['left'], PP.STOW['right']); task.joint_move2(PP.STOW['left'], PP.STOW['right'])


def both_table_to_tote():
    c = {s: task.to_base(task.obj_state(items[s])[0]) for s, _ in SIDES}
    th = {s: (c[s][0], c[s][1], zi + UP_T, SPOTS[s]['yaw'], P) for s, _ in SIDES}
    tl = {s: (c[s][0], c[s][1], zi, SPOTS[s]['yaw'], P) for s, _ in SIDES}
    q_th = {s: task.ik(s, th[s], SPOTS[s]['elbow']) for s, _ in SIDES}
    task.jaw2(JAW_OPEN, 0.5)
    both_unfold()
    task.lift_to(q_th['left']['lift'])                                       # lift first ...
    task.joint_move2(q_th['left'], q_th['right'])                            # ... then swing over the table
    task.cart_move2(th['left'], tl['left'], SPOTS['left']['elbow'], th['right'], tl['right'], SPOTS['right']['elbow'])
    task.jaw2(JAW_CLOSED, 0.8)
    task.cart_move2(tl['left'], th['left'], SPOTS['left']['elbow'], tl['right'], th['right'], SPOTS['right']['elbow'])
    for s, _ in SIDES:
        rise = (task.to_base(task.obj_state(items[s])[0])[2] - c[s][2]) * 1000
        task.report(f'{s}: pick from table A', f'product lifted {rise:.0f} mm (gripper {UP_T*1000:.0f})', ok=rise > UP_T * 1000 - 25)
    zt = PP.ZT_UPRIGHT; T = PP.TOTE2
    hi = {s: (T[s]['x'], T[s]['y'], zt + UP_TOTE, T[s]['yaw'], P) for s, _ in SIDES}
    lo = {s: (T[s]['x'], T[s]['y'], zt, T[s]['yaw'], P) for s, _ in SIDES}
    q_tu = {s: task.ik(s, hi[s], T[s]['elbow']) for s, _ in SIDES}
    task.joint_move2(q_tu['left'], q_tu['right'])                            # swing high over the robot ...
    task.lift_to(q_tu['left']['lift'])                                       # ... then lower over the tote
    task.cart_move2(hi['left'], lo['left'], T['left']['elbow'], hi['right'], lo['right'], T['right']['elbow'])
    task.jaw2(JAW_OPEN, 0.6)
    task.cart_move2(lo['left'], hi['left'], T['left']['elbow'], lo['right'], hi['right'], T['right']['elbow'])
    x0, x1, y0, y1 = PP.TOTE_INNER
    for s, _ in SIDES:
        cc, up = task.obj_state(items[s]); cb = task.to_base(cc)
        task.report(f'{s}: place in tote', f'product at ({cb[0]:+.3f}, {cb[1]:+.3f}, {cb[2]:.3f}) m, {"upright" if up[2] > 0.9 else "tipped"}',
                    ok=x0 < cb[0] < x1 and y0 < cb[1] < y1 and cb[2] < 0.376)
    task.jaw2(JAW_CLOSED, 0.4)
    both_to_stow()


def both_tote_to_table():
    T = PP.TOTE2
    c = {s: task.to_base(task.obj_state(items[s])[0]) for s, _ in SIDES}
    z = max(c[s][2] for s, _ in SIDES)                                     # one lift: pick both at the same height
    hi = {s: (c[s][0], c[s][1], z + UP_TOTE, T[s]['yaw'], P) for s, _ in SIDES}
    lo = {s: (c[s][0], c[s][1], z, T[s]['yaw'], P) for s, _ in SIDES}
    q_hi = {s: task.ik(s, hi[s], T[s]['elbow']) for s, _ in SIDES}
    task.jaw2(JAW_OPEN, 0.5)
    both_unfold()
    task.lift_to(q_hi['left']['lift'])
    task.joint_move2(q_hi['left'], q_hi['right'])
    task.cart_move2(hi['left'], lo['left'], T['left']['elbow'], hi['right'], lo['right'], T['right']['elbow'])
    task.jaw2(JAW_CLOSED, 0.8)
    task.cart_move2(lo['left'], hi['left'], T['left']['elbow'], lo['right'], hi['right'], T['right']['elbow'])
    for s, _ in SIDES:
        rise = (task.to_base(task.obj_state(items[s])[0])[2] - c[s][2]) * 1000
        task.report(f'{s}: pick from tote', f'product lifted {rise:.0f} mm', ok=rise > UP_TOTE * 1000 - 25)
    tb = {s: task.to_base(np.array([*TARGET[s], zi])) for s, _ in SIDES}
    ph = {s: (tb[s][0], tb[s][1], zi + 0.005 + UP_T, SPOTS[s]['yaw'], P) for s, _ in SIDES}
    pl = {s: (tb[s][0], tb[s][1], zi + 0.005, SPOTS[s]['yaw'], P) for s, _ in SIDES}
    q_ph = {s: task.ik(s, ph[s], SPOTS[s]['elbow']) for s, _ in SIDES}
    task.lift_to(q_ph['left']['lift'])                                       # raise first ...
    task.joint_move2(q_ph['left'], q_ph['right'])                            # ... then swing over the table
    task.cart_move2(ph['left'], pl['left'], SPOTS['left']['elbow'], ph['right'], pl['right'], SPOTS['right']['elbow'])
    task.jaw2(JAW_OPEN, 0.6)
    task.cart_move2(pl['left'], ph['left'], SPOTS['left']['elbow'], pl['right'], ph['right'], SPOTS['right']['elbow'])
    for s, _ in SIDES:
        cc, up = task.obj_state(items[s])
        err = np.linalg.norm(cc[:2] - TARGET[s]) * 1000
        task.report(f'{s}: place on table B', f'product {err:.0f} mm from the target, height {cc[2]:.3f} m, '
                    f'{"upright" if up[2] > math.cos(D(10)) else "NOT upright"}',
                    ok=err < 20 and abs(cc[2] - (H + ITEM[2] / 2)) < 0.01 and up[2] > math.cos(D(10)))
    task.jaw2(JAW_CLOSED, 0.4)
    both_to_stow()


try:
    task.drive_to(GOAL_A, 'drive to table A')
    if args.sequential: table_to_tote('left'); table_to_tote('right')
    else: both_table_to_tote()
    task.drive_to(GOAL_B, 'drive to table B')
    if args.sequential: tote_to_table('left'); tote_to_table('right')
    else: both_tote_to_table()
except RuntimeError as e:
    task.report('aborted', str(e), ok=False)
task.summary()
if not args.headless:
    print('Window stays open; close it to exit.')
    while app.is_running():
        w.step(render=True)
app.close()
