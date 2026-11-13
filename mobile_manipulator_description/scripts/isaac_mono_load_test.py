"""Load test for the mono ring (layout B, mono_ring.py): the forces and torques the ring's printed parts must carry.

Isaac Sim's bodies are rigid, so this does NOT show a printed part breaking. It records the wrench (force + torque) that
goes through each joint while the arm works with a payload in the gripper; mono_ring_strength.py turns those wrenches
into stresses in the printed parts (shelf + ribs, base inserts, gear teeth, hub, bearing seats).

Moves (base fixed, payload = extra mass at the TCP):
  static   arm stretched, hand level / hand down, 2 s each
  ring     arm stretched, the ring swings -150 -> +150 deg at its full speed, then a HARD STOP (target = where it is)
  shoulder arm stretched, the base's shoulder swings -110 -> +110 deg at full speed, hard stop
  lift     arm stretched, lift 0.05 -> 0.75 -> 0.05 m at full speed

    python scripts/isaac_mono_load_test.py --headless --payload 0.5      # -> mono_load_test_<payload>.json
"""
import argparse, json, math, os, sys

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--urdf', default=os.path.join(here, '..', 'urdf', 'mobile_manipulator_prototype_mono_arm.urdf'))
ap.add_argument('--headless', action='store_true')
ap.add_argument('--payload', type=float, default=0.5, help='kg held at the TCP')
ap.add_argument('--out', default=None)
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless})

import numpy as np
import omni.usd
from pxr import UsdPhysics, Gf
sys.path.insert(0, here)
from proto_task import Task, DT
from isaacsim.core.utils.types import ArticulationAction

task = Task(app, args.urdf, args.headless, fix_base=True, self_collision=False, layout='prototype_mono_arm')
stage = omni.usd.get_context().get_stage()
hand = stage.GetPrimAtPath('/mobile_manipulator/left_hand')
mapi = UsdPhysics.MassAPI(hand)
m0 = mapi.GetMassAttr().Get(); c0 = np.array(mapi.GetCenterOfMassAttr().Get())
if args.payload > 0:
    tcp = np.array([0.130, 0.0, 0.0])                         # TCP in the hand frame (urdf_build: 142 - 12 mm)
    m1 = m0 + args.payload; c1 = (m0 * c0 + args.payload * tcp) / m1
    mapi.GetMassAttr().Set(m1); mapi.GetCenterOfMassAttr().Set(Gf.Vec3f(*[float(v) for v in c1]))
print(f'hand mass {m0:.3f} kg (+ payload {args.payload} kg at the TCP)')
task.start()

view = task.robot._articulation_view
jidx = view._metadata.joint_indices                            # name -> index (incl. fixed joints); row = index + 1
WATCH = ['left_shoulder_pan', 'left_arm_base_fixed', 'left_elbow_pan', 'lift']
rows = {nm: jidx[nm] + 1 for nm in WATCH if nm in jidx}
print('watching', rows)
ring_i = task.idx['left_shoulder_pan']; sh_i = task.idx['left_elbow_pan']
log = {}


def sample(phase):
    W = np.asarray(task.robot.get_measured_joint_forces())
    eff = np.asarray(task.robot.get_measured_joint_efforts())
    rec = log.setdefault(phase, {nm: dict(F=0.0, Fz=0.0, Fxy=0.0, Mxy=0.0, Mz=0.0, W=None) for nm in rows})
    for nm, r in rows.items():
        f, m = W[r, :3], W[r, 3:]
        Mxy = float(np.hypot(m[0], m[1]))
        if Mxy >= rec[nm]['Mxy']: rec[nm]['W'] = [float(v) for v in W[r]]          # the wrench at the worst tipping moment
        for k, v in (('F', np.linalg.norm(f)), ('Fz', abs(f[2])), ('Fxy', np.hypot(f[0], f[1])), ('Mxy', Mxy), ('Mz', abs(m[2]))):
            rec[nm][k] = max(rec[nm][k], float(v))
    log.setdefault(phase + '_effort', dict(ring=0.0, shoulder=0.0, lift=0.0))
    e = log[phase + '_effort']
    e['ring'] = max(e['ring'], abs(float(eff[ring_i]))); e['shoulder'] = max(e['shoulder'], abs(float(eff[sh_i])))
    e['lift'] = max(e['lift'], abs(float(eff[task.idx['lift']])))


def run(phase, seconds):
    for _ in range(int(seconds / DT)):
        task.step_once(); sample(phase)


def go(q, phase, wait=0.3):
    task.set_arm('left', q)
    a = {k: float(task.q_cmd[task.idx['lift' if k == 'lift' else 'left_' + k]]) for k in q}
    T = max(abs(q[k] - a[k]) / (0.9 * task.vmax[task.idx['lift' if k == 'lift' else 'left_' + k]]) for k in q) + wait
    run(phase, T)


def hard_stop(phase, joint, seconds=1.0):
    """Freeze the command where the joint is NOW (its rate-limited command is still moving): the drive brakes at its cap."""
    i = task.idx[joint]
    q = float(task.robot.get_joint_positions()[i])
    task.q_goal[i] = q; task.q_cmd[i] = q
    run(phase, seconds)


STRETCH = dict(shoulder_pan=0.0, elbow_pan=0.0, wrist_pan=0.0, wrist_pitch=0.0, wrist_roll=0.0, lift=0.40)
task.set_pose({'lift': 0.40})
go(STRETCH, 'settle', 1.0)
log.clear()
run('static_level', 2.0)
go(dict(STRETCH, wrist_pitch=math.radians(90)), 'move', 0.5); run('static_hand_down', 2.0)
go(STRETCH, 'move', 0.5)
go(dict(STRETCH, shoulder_pan=math.radians(-150)), 'move', 0.5)
task.set_arm('left', dict(shoulder_pan=math.radians(150)))
run('ring_swing', 2.0)                                           # accelerating + full speed (~1.06 rad/s)
hard_stop('ring_stop', 'left_shoulder_pan')
go(dict(STRETCH, shoulder_pan=0.0), 'move', 0.5)
go(dict(STRETCH, elbow_pan=math.radians(-110)), 'move', 0.5)
task.set_arm('left', dict(elbow_pan=math.radians(110)))
run('shoulder_swing', 0.5)
hard_stop('shoulder_stop', 'left_elbow_pan')
go(dict(STRETCH, elbow_pan=0.0), 'move', 0.5)
go(dict(STRETCH, lift=0.05), 'move', 0.5)
go(dict(STRETCH, lift=0.75), 'lift_up', 0.5)
go(dict(STRETCH, lift=0.05), 'lift_down', 0.5)

out = args.out or os.path.join(here, '..', f'mono_load_test_{args.payload:g}kg.json')
log.pop('move', None); log.pop('move_effort', None); log.pop('settle', None); log.pop('settle_effort', None)
json.dump(dict(payload_kg=args.payload, hand_mass_kg=m0, frames='each joint wrench in its child joint frame (N, N m)', log=log),
          open(out, 'w'), indent=1)
print(f'\nLOAD TEST payload {args.payload} kg')
for ph, rec in log.items():
    if ph.endswith('_effort'):
        print(f'  {ph:22s} drive torque / force max: ring {rec["ring"]:.2f} N m, shoulder {rec["shoulder"]:.2f} N m, lift {rec["lift"]:.1f} N')
        continue
    for nm, r in rec.items():
        print(f'  {ph:16s} {nm:20s} |F| {r["F"]:6.1f} N (axial {r["Fz"]:6.1f}, radial {r["Fxy"]:6.1f})  tipping {r["Mxy"]:5.2f} N m  twist {r["Mz"]:5.2f} N m')
print('wrote', out)
app.close()
