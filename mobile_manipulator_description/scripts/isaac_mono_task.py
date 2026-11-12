"""One-arm pick-and-place task for the mono-arm EXPERIMENT (layout prototype_mono_arm: the shoulder is a ring around the
lift column). The robot stands still (free base, wheels hold it); one arm:
   1. picks a product from the table in front (hand down)
   2. swings round the column (~160 deg) and drops it into the tote behind
   3. takes it out of the tote again
   4. swings back and places it at a second spot on the table
Plan: mono_poses.py (checked offline against the URDF meshes, the table and the carried product: 125 samples clear).
Moves: time-synchronised joint moves (proto_task.Task.joint_move) and straight TCP lines (cart_move), every command
rate-limited as on the real servos.

    python scripts/isaac_mono_task.py               # window
    python scripts/isaac_mono_task.py --headless
    add --no_smooth for step-by-step moves instead of the TOPP-RA blended ones
    MONO_DEBUG=1: per step, joint errors, base tilt and the PhysX contacts (found the scene camera hitting the table)
"""
import argparse, math, os, sys

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--urdf', default=os.path.join(here, '..', 'urdf', 'mobile_manipulator_prototype_mono_arm.urdf'))
ap.add_argument('--headless', action='store_true')
ap.add_argument('--table_h', type=float, default=0.75)
ap.add_argument('--no_self_collision', action='store_true')
ap.add_argument('--slow_carry', action='store_true', help='half-speed swings while holding the product (smooth_path.CARRY); made no difference here')
ap.add_argument('--no_smooth', action='store_true', help='step-by-step moves instead of the TOPP-RA blended ones (smooth_path.py)')
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless})

import numpy as np
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
sys.path.insert(0, here)
from proto_task import Task
import mono_poses as MP

ITEM = np.array([0.045, 0.045, MP.ITEM_H]); ITEM_MASS = 0.15
H = args.table_h
JAW_OPEN, JAW_CLOSED = 0.030, 0.0

task = Task(app, args.urdf, args.headless, fix_base=False, self_collision=not args.no_self_collision, layout='prototype_mono_arm')
w = task.world
task.profile = 'trap'     # trapezoid moves: full allowed speed after a 0.3 s ramp (same limits)
task.settle_mode = 'still'   # a move is done once the joints stop (the wrist pitch's gravity sag never gets within 2 deg)
# table and product start far away: Isaac resets the robot to its zero pose, where the arm points forward low enough
# to be inside the table (the overlap threw the robot ~35 m on the first run); they move in after the arm is up
TABLE_POS = np.array([MP.TABLE_EDGE + 0.35, 0.0, H / 2]); AWAY = np.array([6.0, 0.0, 0.0])
table = w.scene.add(FixedCuboid(prim_path='/World/table', name='table', position=TABLE_POS + AWAY,
                        scale=np.array([0.70, 1.20, H]), color=np.array([0.62, 0.48, 0.33])))
item = w.scene.add(DynamicCuboid(prim_path='/World/product', name='product', position=np.array([MP.PICK[0], MP.PICK[1], H + ITEM[2] / 2]) + AWAY,
                                 scale=ITEM, mass=ITEM_MASS, color=np.array([0.2, 0.6, 0.9])))
CONTACTS = None
if os.environ.get('MONO_DEBUG'):            # PhysX contact report on every robot link + the product (debug only)
    from pxr import PhysxSchema, UsdPhysics
    from omni.physx import get_physx_simulation_interface
    stage = w.stage
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and ('/mobile_manipulator/' in str(prim.GetPath()) or 'product' in str(prim.GetPath())) \
                and 'roller' not in str(prim.GetPath()):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
    from pxr import PhysicsSchemaTools
    CONTACTS = {}

    def _on_contacts(headers, data):
        for h in headers:
            a = str(PhysicsSchemaTools.intToSdfPath(h.actor0)).split('/')[-1]; b = str(PhysicsSchemaTools.intToSdfPath(h.actor1)).split('/')[-1]
            imp = sum(float(np.linalg.norm([d.impulse[0], d.impulse[1], d.impulse[2]])) for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data])
            k = tuple(sorted((a, b))); CONTACTS[k] = CONTACTS.get(k, 0.0) + imp
    _sub = get_physx_simulation_interface().subscribe_contact_report_events(_on_contacts)
task.start()
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[1.4, 1.2, 1.5], target=[0.1, 0.0, 0.6])

steps, info = MP.plan(H)
q0 = steps[0][2]
# start: arm over the product (teleported, then settled), base held at the origin
task.set_pose({'lift': q0['lift'], **{f'left_{k}': v for k, v in q0.items() if k != 'lift'}})
task.hold = np.array([0.0, 0.0, 0.0]); task.wait(1.0)
table.set_world_pose(position=TABLE_POS)
item.set_world_pose(position=np.array([MP.PICK[0], MP.PICK[1], H + ITEM[2] / 2])); item.set_linear_velocity(np.zeros(3))
task.wait(1.0)
bp, yaw = task.base_pose(); task.report('start', f'base at ({bp[0]*1000:+.1f}, {bp[1]*1000:+.1f}) mm, {math.degrees(yaw):+.2f} deg; arm over the product')
t0 = task.t
tote_ok = None
held_off = None            # product offset from the TCP right after lifting it (mm, in the base frame)


def offset():
    p, _ = Task.obj_state(item); tcp = task.to_base(task.tcp['left'].get_world_pose()[0])
    return (task.to_base(p) - tcp) * 1000
run_steps = steps[1:]
if not args.no_smooth:                       # TOPP-RA: blended runs between the jaw actions (smooth_path.py)
    import smooth_path as SP
    run_steps, _ = SP.expand(run_steps, task.arm_now('left'), SP.vmax_of(task), carry=SP.CARRY if args.slow_carry else None)
for kind, label, tgt in run_steps:
    if kind == 'jaw':
        task.jaw('left', JAW_OPEN if tgt == 1 else JAW_CLOSED, 0.8)
        p, up = Task.obj_state(item); pb = task.to_base(p)
        if label == 'release in the tote':
            inside = (-0.238 < pb[0] < 0.098) and abs(pb[1]) < 0.233 and pb[2] < 0.263 + 0.07
            task.wait(0.5); p, up = Task.obj_state(item); pb = task.to_base(p)
            tote_ok = inside and up[2] > 0.9
            task.report('in the tote', f'product at ({pb[0]:+.3f}, {pb[1]:+.3f}, {pb[2]:.3f}), upright {up[2]:.2f}', ok=tote_ok)
        elif label.startswith('grasp'):
            task.report(label, f'product at ({pb[0]:+.3f}, {pb[1]:+.3f}, {pb[2]:.3f})')
        continue
    if CONTACTS is not None: CONTACTS.clear()
    if kind == 'traj':
        SP.play(task, tgt)
    elif kind == 'joint':
        task.joint_move('left', tgt)
    else:
        task.cart_move('left', np.asarray(MP.K.fk('left', task.arm_now('left'))[:3] + (tgt[3], tgt[4])), tgt[:5], tgt[5])
    p, up = Task.obj_state(item); pb = task.to_base(p)
    if label in ('lift it', 'lift it out'):
        tcp = task.to_base(task.tcp['left'].get_world_pose()[0])
        slip = np.linalg.norm(pb - tcp) * 1000
        task.report(label, f'product {slip:.1f} mm from the TCP (held)', ok=slip < 15 and pb[2] > info['zb'] + 0.05)
        held_off = offset()
    elif label.startswith('swing') and held_off is not None:       # how far it slid in the jaws during the swing
        d = np.linalg.norm(offset() - held_off)
        task.report(label, f'product slid {d:.1f} mm in the jaws during the swing', ok=d < 5); held_off = None
    else:
        o = offset()
        if os.environ.get('MONO_DEBUG'):
            qa = task.robot.get_joint_positions()
            err = {k: math.degrees(task.q_goal[task.idx[k]] - qa[task.idx[k]]) for k in
                   ('left_shoulder_pan', 'left_elbow_pan', 'left_wrist_pan', 'left_wrist_pitch')}
            print('      [debug] goal - actual (deg): ' + ', '.join(f'{k[5:]} {v:+.1f}' for k, v in err.items())
                  + f", lift {(task.q_goal[task.idx['lift']] - qa[task.idx['lift']]) * 1000:+.1f} mm", flush=True)
            _, qu = task.robot.get_world_pose(); w_, x_, y_, z_ = [float(v) for v in qu]
            roll = math.degrees(math.atan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w_ * y_ - z_ * x_)))))
            bp_, _ = task.base_pose()
            top = sorted(((v, k) for k, v in (CONTACTS or {}).items() if 'wheel' not in k[0] + k[1] or v > 0), reverse=True)[:8]
            print('      [debug] contacts (N s): ' + ', '.join(f'{a}-{b} {v:.2f}' for v, (a, b) in top), flush=True)
            print(f'      [debug] base tilt roll {roll:+.2f} pitch {pitch:+.2f} deg, base at ({bp_[0]*1000:+.1f}, {bp_[1]*1000:+.1f}, {bp_[2]*1000:+.1f}) mm', flush=True)
        task.report(label, f'done, product at ({pb[0]:+.3f}, {pb[1]:+.3f}, {pb[2]:.3f}), {o[0]:+.0f} {o[1]:+.0f} {o[2]:+.0f} mm from the TCP')
task.wait(1.0)
p, up = Task.obj_state(item); pb = task.to_base(p)
err = math.hypot(pb[0] - MP.PLACE[0], pb[1] - MP.PLACE[1]) * 1000
task.report('placed on the table', f'{err:.1f} mm from the target, height {pb[2]:.3f} (table {H}), upright {up[2]:.2f}',
            ok=err < 20 and abs(pb[2] - (H + ITEM[2] / 2)) < 0.01 and up[2] > 0.95)
bp, yaw = task.base_pose()
task.report('base held still', f'moved {np.linalg.norm(bp[:2]) * 1000:.1f} mm, {math.degrees(yaw):+.2f} deg', ok=np.linalg.norm(bp[:2]) < 0.015)
print(f'task time {task.t - t0:.1f} s (one arm, table -> tote -> table)')
task.summary()
app.close()
