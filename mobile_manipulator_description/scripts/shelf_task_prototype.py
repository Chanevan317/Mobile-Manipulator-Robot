"""Shelf task for the prototype layout (run it through isaac_shelf_task.py, the default layout there).

Shelf task scene and steps: drive to shelf A, pick the product, put it in the tote, drive to shelf B,
pick it from the tote and place it on shelf B. Differences for the prototype:
  - the working face is the front (+x), where the column is; the tote is behind the column
  - the left arm works; the right arm waits straight out to its side (PARK), both arms travel stowed
  - every joint-space move follows a route planned offline (proto_poses.py) and is time-synchronised
  - the item sits 0.68 m ahead of the base centre, straight ahead of the left arm; 0.08 m straight approach
"""
import math
import numpy as np
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from proto_task import Task
import proto_poses as PP

D = math.radians
P = math.pi / 2
ITEM = np.array([0.045, 0.045, 0.070]); ITEM_MASS = 0.15
SHELF_A = dict(edge_x=2.0, y=1.0, board=0.70)   # faces -x
SHELF_B = dict(edge_y=-1.5, x=0.8, board=0.50)  # faces +y
REACH = np.array(PP.SHELF_REACH)
ITEM_A = np.array([SHELF_A['edge_x'] + 0.04, SHELF_A['y'], SHELF_A['board'] + ITEM[2] / 2])
ITEM_B = np.array([SHELF_B['x'], SHELF_B['edge_y'] - 0.06, SHELF_B['board'] + ITEM[2] / 2])   # 6 cm in: at 4 cm an
# item that sat ~2 cm deep in the gripper landed on the board edge and tipped off as the hand backed out (2026-11-09)
GRASP_DZ = -0.003
APPROACH = PP.SHELF_APPROACH
E = PP.SHELF_ELBOW
JAW_OPEN, JAW_CLOSED = 0.03, 0.0
JAW_RELEASE = 0.04     # release wide (80 mm for the 45 mm item) and wait until open: at 60 mm a finger still
                       # brushed the item on the way out in some runs and dragged it off shelf B (2026-11-09)
TOTE = PP.TOTE_SHELF['left']
WOOD = np.array([0.62, 0.48, 0.33])


def goal_for(item_xy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([item_xy[0] - (c * REACH[0] - s * REACH[1]), item_xy[1] - (s * REACH[0] + c * REACH[1]), yaw])


def run(app, args):
    task = Task(app, args.urdf, args.headless, fix_base=False, self_collision=not args.no_self_collision)
    w = task.world

    def shelf(name, center, size):
        w.scene.add(FixedCuboid(prim_path=f'/World/{name}', name=name, position=np.array(center), scale=np.array(size), color=WOOD))
    for i, zb in enumerate((0.30, SHELF_A['board'], 1.10)):
        shelf(f'shelfA_board{i}', (2.15, SHELF_A['y'], zb - 0.01), (0.30, 0.80, 0.02))
    for i, yy in enumerate((0.59, 1.41)):
        shelf(f'shelfA_side{i}', (2.15, yy, 0.60), (0.30, 0.02, 1.20))
    shelf('shelfA_back', (2.305, SHELF_A['y'], 0.60), (0.01, 0.84, 1.20))
    for i, zb in enumerate((0.10, SHELF_B['board'], 0.90)):
        shelf(f'shelfB_board{i}', (SHELF_B['x'], -1.65, zb - 0.01), (0.80, 0.30, 0.02))
    for i, xx in enumerate((0.39, 1.21)):
        shelf(f'shelfB_side{i}', (xx, -1.65, 0.50), (0.02, 0.30, 1.00))
    shelf('shelfB_back', (SHELF_B['x'], -1.805, 0.50), (0.84, 0.01, 1.00))
    item = w.scene.add(DynamicCuboid(prim_path='/World/product', name='product', position=ITEM_A, scale=ITEM, mass=ITEM_MASS,
                                     color=np.array([0.85, 0.15, 0.15])))
    task.start()
    if not args.headless:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=np.array([-1.2, -2.6, 2.6]), target=np.array([1.0, -0.2, 0.4]))

    def item_state():
        return task.obj_state(item)

    def deploy_right():
        task.route('right', PP.ROUTE['right']['stow_to_park'], PP.PARK['right'])

    def stow_both():
        task.route('left', PP.ROUTE['left']['tote_to_stow'], PP.STOW['left'])       # left first (planned that way)
        task.route('right', PP.ROUTE['right']['park_to_stow'], PP.STOW['right'])

    def pick_from_shelf(label):
        c = task.to_base(item_state()[0])
        g = np.array([c[0], c[1], c[2] + GRASP_DZ, 0.0, 0.0]); pre = g - np.array([APPROACH, 0, 0, 0, 0])
        task.jaw('left', JAW_OPEN, 0.6)
        task.route('left', PP.ROUTE['left']['stow_to_shelf'], task.ik('left', pre, E))
        task.cart_move('left', pre, g, E)
        task.jaw('left', JAW_CLOSED, 0.8)
        up = g + np.array([0, 0, 0.03, 0, 0]); task.cart_move('left', g, up, E)
        out = up - np.array([APPROACH, 0, 0, 0, 0]); task.cart_move('left', up, out, E)
        q = task.robot.get_joint_positions()
        gap = (q[task.idx['left_finger_right']] + q[task.idx['left_finger_left']]) * 1000
        rise = (task.to_base(item_state()[0])[2] - c[2]) * 1000
        task.report(label, f'jaw gap {gap:.1f} mm (item {ITEM[0]*1000:.0f}), item lifted {rise:.0f} mm (gripper 30) and out of the shelf',
                    ok=abs(gap - ITEM[0] * 1000) < 5 and rise > 20)

    def tote_pose(z, yaw=TOTE['yaw']):
        return np.array([TOTE['x'], TOTE['y'], z, yaw, P])

    def place_in_tote():
        above = tote_pose(PP.ZT_FLAT + PP.TOTE_UP)
        task.route('left', PP.ROUTE['left']['shelf_to_tote'], task.ik('left', above, TOTE['elbow']))
        task.cart_move('left', above, tote_pose(PP.ZT_FLAT), TOTE['elbow'])
        task.jaw('left', JAW_OPEN, 0.6)
        task.cart_move('left', tote_pose(PP.ZT_FLAT), above, TOTE['elbow'])
        task.jaw('left', JAW_CLOSED, 0.6)
        c, up = item_state(); cb = task.to_base(c); x0, x1, y0, y1 = PP.TOTE_INNER
        task.report('place in tote', f'item at ({cb[0]:+.3f}, {cb[1]:+.3f}, {cb[2]:.3f}) m in the base frame, lying '
                    f'{"flat" if abs(up[2]) < 0.2 else "NOT flat"}', ok=x0 < cb[0] < x1 and y0 < cb[1] < y1 and cb[2] < 0.376)

    def pick_from_tote():
        c, up = item_state(); cb = task.to_base(c)
        p, byaw = task.base_pose()
        ax = math.atan2(up[1], up[0]) - byaw
        yaw = min((ax, ax + math.pi), key=lambda a: abs(math.atan2(math.sin(a - TOTE['yaw']), math.cos(a - TOTE['yaw']))))
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        g = np.array([cb[0], cb[1], cb[2], yaw, P]); above = g + np.array([0, 0, PP.TOTE_UP, 0, 0])
        task.jaw('left', JAW_OPEN, 0.6)
        deploy_right()
        task.route('left', PP.ROUTE['left']['stow_to_tote'], task.ik('left', above, TOTE['elbow']))
        task.cart_move('left', above, g, TOTE['elbow'])
        task.jaw('left', JAW_CLOSED, 0.8)
        task.cart_move('left', g, above, TOTE['elbow'])
        rise = (item_state()[0][2] - c[2]) * 1000
        task.report('pick from tote', f'item lifted {rise:.0f} mm (gripper {PP.TOTE_UP*1000:.0f} mm)', ok=rise > PP.TOTE_UP * 1000 - 30)
        return rise > PP.TOTE_UP * 1000 - 30

    def place_on_shelf_b():
        t = task.to_base(ITEM_B)
        g = np.array([t[0], t[1], t[2] + GRASP_DZ, 0.0, 0.0]); hi = g + np.array([0, 0, 0.02, 0, 0])
        pre = hi - np.array([APPROACH, 0, 0, 0, 0])
        task.route('left', PP.ROUTE['left']['tote_to_shelf'], task.ik('left', pre, E))
        task.cart_move('left', pre, hi, E)
        task.cart_move('left', hi, g, E)
        task.jaw('left', JAW_RELEASE, 1.0)
        out = g - np.array([APPROACH, 0, 0, 0, 0]); task.cart_move('left', g, out, E)
        task.jaw('left', JAW_CLOSED, 0.6)
        task.route('left', PP.ROUTE['left']['shelf_to_ready'], PP.READY['left'])
        task.route('left', PP.ROUTE['left']['ready_to_stow'], PP.STOW['left'])
        task.route('right', PP.ROUTE['right']['park_to_stow'], PP.STOW['right'])

    # ------------------------------------------------------------ run
    task.set_pose({**PP.full('left', PP.STOW['left']), **PP.full('right', PP.STOW['right'])})
    item.set_world_pose(position=ITEM_A, orientation=np.array([1.0, 0, 0, 0]))
    task.wait(1.0)
    print(f'\nShelf task (prototype layout): product {ITEM[0]*1000:.0f} x {ITEM[1]*1000:.0f} x {ITEM[2]*1000:.0f} mm, '
          f"{ITEM_MASS*1000:.0f} g, self-collision {'off' if args.no_self_collision else 'on'}\n")
    try:
        task.drive_to(goal_for(ITEM_A[:2], 0.0), 'drive to shelf A')
        deploy_right()
        pick_from_shelf('pick shelf A')
        place_in_tote()
        stow_both()
        task.drive_to(goal_for(ITEM_B[:2], -P), 'drive to shelf B')
        if pick_from_tote():
            place_on_shelf_b()
        task.wait(2.0)
    except RuntimeError as e:
        task.report('aborted', str(e), ok=False)
    c, up = item_state()
    err = np.linalg.norm(c[:2] - ITEM_B[:2]) * 1000
    task.report('result', f'item {err:.0f} mm from the target on shelf B, height {c[2]:.3f} m (board centre {ITEM_B[2]:.3f}), '
                f"{'upright' if up[2] > math.cos(D(10)) else 'NOT upright'}",
                ok=err < 30 and abs(c[2] - ITEM_B[2]) < 0.01 and up[2] > math.cos(D(10)))
    task.summary()
    if not args.headless:
        print('Window stays open; close it to exit.')
        while app.is_running():
            w.step(render=True)
    app.close()
