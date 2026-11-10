"""Shared helpers for the prototype-layout Isaac Sim tasks (isaac_table_task.py, isaac_shelf_task.py --layout prototype).

Create the SimulationApp in the task script first, then build a Task. Arm motions:
  joint_move(side, q)      time-synchronised joint-space move: every joint (and the lift) starts and ends together,
                           so the arm sweeps exactly the straight joint-space line the offline planner checked
                           (proto_poses.py); speed stays under 90 % of each joint's URDF limit
  route(side, names, q)    joint_move through named waypoints of proto_poses.WAYPOINTS, then to q
  cart_move(side, a, b)    straight TCP line (x, y, z, yaw, pitch in the base frame), IK every tick
Every command also goes through drives.rate_limit(), as on the real servos.
"""
import math, os
import numpy as np
import omni.kit.commands, omni.usd
from pxr import UsdLux, Sdf
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation as Articulation, SingleXFormPrim
from isaacsim.core.utils.types import ArticulationAction
from drives import apply_drives, rate_limit
import scara_ik as K

CTRL_HZ = 60
DT = 1.0 / CTRL_HZ
TCP_SPEED = 0.20          # m/s for straight-line hand moves (time follows the distance)
TCP_TURN = 1.5            # rad/s for heading / pitch changes in those moves
MIN_MOVE = 0.4            # s, shortest straight-line move
ARM_KEYS = ('shoulder_pan', 'elbow_pan', 'wrist_pan', 'wrist_pitch', 'wrist_roll')
RAMP = 0.3                # s, accel / decel ramp of the 'trap' profile


def _trap(t, T, tr):
    """Normalised trapezoid profile s(t) in [0, 1]: linear speed ramps of tr at both ends, constant speed between."""
    vp = 1.0 / (T - tr)
    if t <= tr: return 0.5 * vp * t * t / tr
    if t >= T - tr: return 1.0 - 0.5 * vp * (T - t) ** 2 / tr
    return vp * (t - tr / 2)


def _profile(kind, base):
    """(duration, s(i, steps)) for a move whose slowest joint needs `base` s at its speed limit.
    cosine: peak = pi/2 x mean (the tested default);  trap: a RAMP s ramp, then the full allowed speed (~35 % faster on long moves)."""
    if kind == 'trap':
        tr = min(RAMP, 0.5 * base); T = base + tr          # never slower than cosine (1.5 x base at most); peak = the limit
        return T, (lambda i, n: _trap(T * i / n, T, tr))
    T = base * math.pi / 2
    return T, (lambda i, n: 0.5 - 0.5 * math.cos(math.pi * i / n))


class Task:
    def __init__(self, app, urdf, headless, fix_base, self_collision=True, layout='prototype'):
        K.use(layout); self.layout = layout
        self.app, self.headless = app, headless
        self.world = World(stage_units_in_meters=1.0, physics_dt=1 / 240.0, rendering_dt=DT)
        self.world.scene.add_ground_plane()
        stage = omni.usd.get_context().get_stage()
        UsdLux.DistantLight.Define(stage, Sdf.Path('/DistantLight')).CreateIntensityAttr(3000)
        UsdLux.DomeLight.Define(stage, Sdf.Path('/DomeLight')).CreateIntensityAttr(500)
        _, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
        cfg.merge_fixed_joints = False; cfg.fix_base = fix_base; cfg.make_default_prim = True; cfg.self_collision = False
        cfg.import_inertia_tensor = True; cfg.convex_decomp = False; cfg.distance_scale = 1.0
        cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        _, self.art_path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=os.path.abspath(urdf),
                                                     import_config=cfg, get_articulation_root=True)
        apply_drives(stage, self.art_path, layout)
        if self_collision:
            from self_collision import setup_self_collision
            print('self-collision on,', setup_self_collision(stage, os.path.abspath(urdf)), 'pairs ignored')
        self.results = []
        self.t = 0.0
        self.profile = 'cosine'   # 'trap' (mono tasks): faster, same speed limits

    def start(self):
        """Call after the scene objects are added."""
        self.robot = self.world.scene.add(Articulation(prim_path=self.art_path, name='mobile_manipulator'))
        self.world.reset(); self.robot.initialize()
        self.tcp = {s: SingleXFormPrim(f'/mobile_manipulator/{s}_tcp') for s in K.SHOULDER}   # one arm in the mono layout
        self.names = list(self.robot.dof_names); n = len(self.names)
        self.idx = {nm: i for i, nm in enumerate(self.names)}
        self.vmax = np.array(self.robot.dof_properties['maxVelocity'], dtype=float)
        self.wheel = [self.idx[f'wheel_{w}_joint'] for w in ('fl', 'fr', 'rl', 'rr')]
        self.driven = [i for i, nm in enumerate(self.names) if 'roller' not in nm and not nm.startswith('wheel')]
        self.substeps = max(1, round(DT / self.world.get_physics_dt()))
        self.q_goal = np.zeros(n); self.q_cmd = np.zeros(n); self.wheel_v = np.zeros(4)
        self.hold = None           # parked base pose (x, y, yaw) the wheels hold while the arms work (set by drive_to)

    # ------------------------------------------------------------ low level
    def tick(self):
        if self.headless:
            for _ in range(self.substeps):
                self.world.step(render=False)
        else:
            self.world.step(render=True)
        if not self.app.is_running():
            raise SystemExit

    def hold_base(self):
        """Wheel velocities that pull the parked base back to self.hold. The wheels' velocity drives are soft, and the
        passive rollers let the arms' reaction push the base around (5 cm in the shelf task, 2026-11-09); a real base
        holds position with its wheel motors."""
        R, LX, LY = 0.05, 0.17, 0.222
        p, yaw = self.base_pose(); g = self.hold
        ex, ey = g[0] - p[0], g[1] - p[1]; eyaw = math.atan2(math.sin(g[2] - yaw), math.cos(g[2] - yaw))
        vw = np.array([ex, ey]) * 2.0; sp = np.linalg.norm(vw)
        if sp > 0.05: vw *= 0.05 / sp
        wz = max(-0.2, min(0.2, 2.0 * eyaw))
        c, s = math.cos(yaw), math.sin(yaw); vx, vy = c * vw[0] + s * vw[1], -s * vw[0] + c * vw[1]; kk = LX + LY
        self.wheel_v[:] = [(vx - vy - kk * wz) / R, (vx + vy + kk * wz) / R, (vx + vy - kk * wz) / R, (vx - vy + kk * wz) / R]

    def step_once(self):
        if self.hold is not None:
            self.hold_base()
        d = self.driven
        self.q_cmd[d] = rate_limit(self.q_cmd[d], self.q_goal[d], self.vmax[d], DT)
        self.robot.apply_action(ArticulationAction(joint_positions=self.q_cmd[d], joint_indices=np.array(d)))
        self.robot.apply_action(ArticulationAction(joint_velocities=self.wheel_v, joint_indices=np.array(self.wheel)))
        self.tick(); self.t += DT

    def wait(self, seconds):
        for _ in range(int(seconds * CTRL_HZ)):
            self.step_once()

    def settle(self, tol=0.035, timeout=6.0):
        d = self.driven
        for _ in range(int(timeout * CTRL_HZ)):
            self.step_once()
            q = self.robot.get_joint_positions()
            if np.max(np.abs(self.q_cmd[d] - self.q_goal[d])) < 1e-6 and np.max(np.abs(q[d] - self.q_goal[d])) < tol:
                self.wait(0.1); return True
        return False

    def set_pose(self, q_all):
        """Teleport the joints (start of a task only)."""
        q0 = np.zeros(len(self.names))
        for k, v in q_all.items(): q0[self.idx[k]] = v
        self.robot.set_joint_positions(q0); self.q_goal[:] = q0; self.q_cmd[:] = q0

    def set_arm(self, side, q):
        for k, v in q.items():
            if k == 'lift': self.q_goal[self.idx['lift']] = v
            elif k in ARM_KEYS: self.q_goal[self.idx[f'{side}_{k}']] = v

    def set_jaw(self, side, v):
        self.q_goal[self.idx[f'{side}_finger_right']] = self.q_goal[self.idx[f'{side}_finger_left']] = v

    def jaw(self, side, v, seconds=0.7):
        self.set_jaw(side, v); self.wait(seconds)

    def arm_now(self, side):
        q = {k: float(self.q_goal[self.idx[f'{side}_{k}']]) for k in ARM_KEYS}
        q['lift'] = float(self.q_goal[self.idx['lift']]); return q

    # ------------------------------------------------------------ motions
    def joint_move(self, side, q, lift=True):
        """Time-synchronised straight joint-space move of one arm (and the lift unless lift=False)."""
        a = self.arm_now(side); keys = [k for k in q if k in ARM_KEYS or (k == 'lift' and lift)]
        lim = {k: 0.88 * self.vmax[self.idx['lift' if k == 'lift' else f'{side}_{k}']] for k in keys}
        T, prof = _profile(self.profile, max([abs(q[k] - a[k]) / lim[k] for k in keys] + [0.05]))
        steps = max(2, int(T * CTRL_HZ) + 1)
        for i in range(1, steps + 1):
            s = prof(i, steps)
            self.set_arm(side, {k: a[k] + (q[k] - a[k]) * s for k in keys})
            self.step_once()
        self.settle(timeout=8.0)

    # ------------------------------------------------------------ both arms at once (one shared lift)
    def joint_move2(self, qL, qR, lift=None):
        """Time-synchronised move of both arms (and the lift to `lift`, or None to keep it): one straight joint-space
        line for all of them, as checked offline."""
        a = {s: self.arm_now(s) for s in ('left', 'right')}
        tgt = {'left': {k: qL[k] for k in qL if k in ARM_KEYS}, 'right': {k: qR[k] for k in qR if k in ARM_KEYS}}
        l0 = a['left']['lift']; l1 = l0 if lift is None else lift
        Ts = [0.05, abs(l1 - l0) / (0.88 * self.vmax[self.idx['lift']])]
        for s in tgt:
            Ts += [abs(tgt[s][k] - a[s][k]) / (0.88 * self.vmax[self.idx[f'{s}_{k}']]) for k in tgt[s]]
        steps = max(2, int(max(Ts) * math.pi / 2 * CTRL_HZ) + 1)
        for i in range(1, steps + 1):
            u = 0.5 - 0.5 * math.cos(math.pi * i / steps)
            for s in tgt:
                self.set_arm(s, {k: a[s][k] + (tgt[s][k] - a[s][k]) * u for k in tgt[s]})
            self.q_goal[self.idx['lift']] = l0 + (l1 - l0) * u
            self.step_once()
        self.settle(timeout=8.0)

    def lift_to(self, v):
        self.joint_move2({}, {}, lift=v)

    def cart_move2(self, aL, bL, eL, aR, bR, eR):
        """Straight TCP lines for both hands at once; the lift must agree (same heights), so z moves are shared."""
        aL, bL, aR, bR = (np.asarray(v, float) for v in (aL, bL, aR, bR))
        d = np.maximum(np.abs(bL - aL), np.abs(bR - aR))
        vlift = 0.85 * self.vmax[self.idx['lift']]
        seconds = max(MIN_MOVE, float(np.linalg.norm(d[:3])) / TCP_SPEED * math.pi / 2,
                      max(d[3], d[4]) / TCP_TURN * math.pi / 2, d[2] / vlift * math.pi / 2)
        steps = max(2, int(seconds * CTRL_HZ))
        for k in range(1, steps + 1):
            u = 0.5 - 0.5 * math.cos(math.pi * k / steps)
            qL = K.ik('left', *(aL + (bL - aL) * u), elbow=eL); qR = K.ik('right', *(aR + (bR - aR) * u), elbow=eR)
            if qL is None or qR is None:
                raise RuntimeError(f'no IK on the two-arm straight move at u={u:.2f}')
            if abs(qL['lift'] - qR['lift']) > 0.002:
                raise RuntimeError(f'the two hands need different lift heights ({qL["lift"]:.3f} / {qR["lift"]:.3f})')
            self.set_arm('left', qL); self.set_arm('right', qR); self.step_once()
        self.settle(timeout=3.0)

    def jaw2(self, v, seconds=0.7):
        self.set_jaw('left', v); self.set_jaw('right', v); self.wait(seconds)

    def route(self, side, waypoints, q):
        for w in waypoints:
            self.joint_move(side, w)
        self.joint_move(side, q)

    def cart_move(self, side, a, b, elbow, seconds=None):
        """Straight TCP line; with seconds=None the duration follows the distance (TCP_SPEED, TCP_TURN, MIN_MOVE)."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        if seconds is None:
            d = np.abs(b - a)
            vlift = 0.85 * self.vmax[self.idx['lift']]                     # the lift does all of a vertical move
            base = max(float(np.linalg.norm(d[:3])) / TCP_SPEED, max(d[3], d[4]) / TCP_TURN, d[2] / vlift)
            seconds, prof = _profile(self.profile, base); seconds = max(MIN_MOVE, seconds)
        else:
            _, prof = _profile('cosine', 1.0)
        steps = max(2, int(seconds * CTRL_HZ))
        for k in range(1, steps + 1):
            s = prof(k, steps)
            q = K.ik(side, *(a + (b - a) * s), elbow=elbow)
            if q is None:
                raise RuntimeError(f'{side}: no IK on the straight move at {np.round(a + (b - a) * s, 3)}')
            self.set_arm(side, q); self.step_once()
        self.settle(timeout=3.0)

    def ik(self, side, pose, elbow):
        q = K.ik(side, *pose, elbow=elbow)
        if q is None:
            raise RuntimeError(f'{side}: no IK for {np.round(pose, 3)} elbow {elbow:+d}')
        return q

    # ------------------------------------------------------------ base and objects
    def base_pose(self):
        p, quat = self.robot.get_world_pose()
        w, x, y, z = [float(v) for v in quat]
        return np.array(p, dtype=float), math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def to_base(self, p_world):
        p, yaw = self.base_pose(); d = np.asarray(p_world, float) - p; c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])

    @staticmethod
    def obj_state(obj):
        p, quat = obj.get_world_pose(); w, x, y, z = [float(v) for v in quat]
        up = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        return np.array(p, dtype=float), up

    def drive_to(self, goal, label, timeout=45.0):
        """Closed-loop base move to (x, y, yaw) in the world with mecanum wheel mixing ."""
        R, LX, LY = 0.05, 0.17, 0.222
        self.hold = None
        for _ in range(int(timeout * CTRL_HZ)):
            p, yaw = self.base_pose()
            ex, ey = goal[0] - p[0], goal[1] - p[1]
            eyaw = math.atan2(math.sin(goal[2] - yaw), math.cos(goal[2] - yaw))
            if math.hypot(ex, ey) < 0.008 and abs(eyaw) < math.radians(0.5):
                break
            vw = np.array([ex, ey]) * 1.2; sp = np.linalg.norm(vw)
            if sp > 0.25: vw *= 0.25 / sp
            elif 0 < sp < 0.03: vw *= 0.03 / sp
            wz = max(-0.5, min(0.5, 1.5 * eyaw))
            if 0 < abs(wz) < 0.03: wz = math.copysign(0.03, wz)
            c, s = math.cos(yaw), math.sin(yaw); vx, vy = c * vw[0] + s * vw[1], -s * vw[0] + c * vw[1]; kk = LX + LY
            self.wheel_v[:] = [(vx - vy - kk * wz) / R, (vx + vy + kk * wz) / R, (vx + vy - kk * wz) / R, (vx - vy + kk * wz) / R]
            self.step_once()
        self.wheel_v[:] = 0; self.hold = np.array(goal, dtype=float); self.wait(1.0)
        p, yaw = self.base_pose()
        err = (math.hypot(goal[0] - p[0], goal[1] - p[1]) * 1000, math.degrees(math.atan2(math.sin(goal[2] - yaw), math.cos(goal[2] - yaw))))
        self.report(label, f'stopped {err[0]:.1f} mm, {err[1]:+.2f} deg from the goal', ok=err[0] < 15 and abs(err[1]) < 1.0)

    # ------------------------------------------------------------ reporting
    def report(self, step, text, ok=None):
        tag = '' if ok is None else ('  OK' if ok else '  FAIL')
        print(f'[{self.t:6.1f} s] {step:26s} {text}{tag}', flush=True)
        if ok is not None: self.results.append((step, ok, text))

    def summary(self):
        print(f"\n{sum(ok for _, ok, _ in self.results)}/{len(self.results)} steps OK. Simulated time {self.t:.0f} s.")
