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
WHEELS = {'prototype_mono_arm': (0.145, 0.197)}   # wheel centre offsets (x, y) per layout: the one-arm base is 0.45 since
                                                  # 2026-11-24; the others (0.50 base) use (0.17, 0.222)
ARM_KEYS = ('shoulder_pan', 'elbow_pan', 'wrist_pan', 'wrist_pitch', 'wrist_roll')
RAMP = 0.3                # s, accel / decel ramp of the 'trap' profile
STILL_V = 0.1             # rad/s: 'stopped' for settle_mode 'still' (the wrist pitch holding a product hunts at 0.03-0.06)
STILL_N = 6               # control steps (0.1 s) over which a joint must not move more than STILL_SPREAD
STILL_SPREAD = 0.0026     # rad (0.15 deg) / STILL_SPREAD_LIFT m: 'stopped' for settle_mode 'still'
STILL_SPREAD_LIFT = 0.0003
JAW_STILL_V = 0.004       # m/s: a jaw has stopped (jaw_mode 'stop')
STILL_V_LIFT = 0.005      # m/s for the lift: its top speed is 0.10 m/s, so 0.1 counted it as stopped while it was still
                          # ~19 mm behind going into the tote (the jaws then opened 20 mm above the floor)


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
    def __init__(self, app, urdf, headless, fix_base, self_collision=True, layout='prototype', physics_hz=240, gpu=False, render_every=1):
        """physics_hz: PhysX steps per second (240 tested everywhere; arm training uses scene_robot's default).
        gpu: PhysX GPU dynamics. A URDF from urdf_variants.planar_base (no wheels, base on base_x / base_y / base_yaw
        joints, import with fix_base=True) switches drive_to / base_pose to those joints.
        render_every: in a window, draw only every Nth control frame (the others step physics only): drawing the
        viewport costs far more than the physics (0.22 x real time in a window vs 1.8 x headless, 2026-11-17)."""
        K.use(layout); self.layout = layout
        self.app, self.headless = app, headless
        self.render_every, self._frame = max(1, int(render_every)), 0
        self.world = World(stage_units_in_meters=1.0, physics_dt=1.0 / physics_hz, rendering_dt=DT)
        if gpu:
            self.world.get_physics_context().enable_gpu_dynamics(True)
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
        self.settle_log = []      # (seconds spent settling, None or the joint that never got within tol)
        self.jaw_mode = 'fixed'   # 'stop' (scene_v0): a jaw action ends when the jaws stop, not after a fixed time
        self.settle_cap = None    # seconds: the longest a move may wait to settle (None = each move's own timeout)
        self.jaw_log = []
        self.settle_mode = 'tol'  # 'still' (mono tasks): also done once the arm joints + lift have stopped (< STILL_V for 0.1 s)

    def start(self):
        """Call after the scene objects are added."""
        self.robot = self.world.scene.add(Articulation(prim_path=self.art_path, name='mobile_manipulator'))
        self.world.reset(); self.robot.initialize()
        self.tcp = {s: SingleXFormPrim(f'/mobile_manipulator/{s}_tcp') for s in K.SHOULDER}   # one arm in the mono layout
        import cameras                                         # the robot's scene + wrist cameras (MM_CAMERAS=0: off)
        self.cams = cameras.add(self)
        self.names = list(self.robot.dof_names); n = len(self.names)
        self.idx = {nm: i for i, nm in enumerate(self.names)}
        self.vmax = np.array(self.robot.dof_properties['maxVelocity'], dtype=float)
        self.planar = 'base_x' in self.idx                  # planar_base variant: base moved by joints, no wheels
        self.wheel = [] if self.planar else [self.idx[f'wheel_{w}_joint'] for w in ('fl', 'fr', 'rl', 'rr')]
        self.base_idx = [self.idx[n] for n in ('base_x', 'base_y', 'base_yaw')] if self.planar else []
        self.driven = [i for i, nm in enumerate(self.names) if 'roller' not in nm and not nm.startswith('wheel')]
        self.still_idx = [i for i in self.driven if 'finger' not in self.names[i]]
        self.still_v = np.array([STILL_V_LIFT if self.names[i] == 'lift' else STILL_V for i in self.still_idx])
        self.still_spread = np.array([STILL_SPREAD_LIFT if self.names[i] == 'lift' else STILL_SPREAD for i in self.still_idx])
        self.substeps = max(1, round(DT / self.world.get_physics_dt()))
        self.q_goal = np.zeros(n); self.q_cmd = np.zeros(n); self.wheel_v = np.zeros(len(self.wheel))
        # optional offset added to the command actually sent (train_pipeline: human-like tremor in recorded demos);
        # q_sent = what the drives got last step (= q_cmd without an offset)
        self.cmd_offset = None; self.q_sent = np.zeros(n)
        self.hold = None           # parked base pose (x, y, yaw) the wheels hold while the arms work (set by drive_to)

    # ------------------------------------------------------------ low level
    def tick(self):
        self._frame += 1
        if self.headless or self._frame % self.render_every:
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
        R = 0.05; LX, LY = WHEELS.get(self.layout, (0.17, 0.222))
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
        self.q_sent[:] = self.q_cmd if self.cmd_offset is None else self.q_cmd + self.cmd_offset
        self.robot.apply_action(ArticulationAction(joint_positions=self.q_sent[d], joint_indices=np.array(d)))
        if self.wheel:
            self.robot.apply_action(ArticulationAction(joint_velocities=self.wheel_v, joint_indices=np.array(self.wheel)))
        self.tick(); self.t += DT

    def wait(self, seconds):
        for _ in range(int(seconds * CTRL_HZ)):
            self.step_once()

    def settle(self, tol=0.035, timeout=6.0):
        d = self.driven; t0 = self.t
        if self.settle_cap is not None:              # scene_v0: never sit out a long timeout (the next move starts from
            timeout = min(timeout, self.settle_cap)  # the last target, so ending early can't jump)
        hist = []
        for _ in range(int(timeout * CTRL_HZ)):
            self.step_once()
            q = self.robot.get_joint_positions()
            hist = (hist + [q[self.still_idx].copy()])[-12:]
            reached = np.max(np.abs(self.q_cmd[d] - self.q_goal[d])) < 1e-6
            if reached and np.max(np.abs(q[d] - self.q_goal[d])) < tol:
                if self.settle_cap is None:              # the older tasks keep their extra 0.1 s; scene_v0 (settle_cap)
                    self.wait(0.1)                       # goes on at once: the next move starts from this target anyway
                self.settle_log.append((self.t - t0, False)); return True
            if reached and self.settle_mode == 'still' and len(hist) >= STILL_N:
                # stopped moving: a steady gravity sag (the wrist pitch holding a product, ~2-3 deg) won't shrink by waiting.
                # Judged by how far each joint MOVED over the last STILL_N steps, not by the reported joint speed: holding
                # a product PhysX reports ~0.2 rad/s on a wrist whose angle didn't change by 0.01 deg in 0.2 s, so the
                # speed test never passed and every such move sat out its timeout (2026-11-20).
                # arm joints + lift only: the fingers squeezing a product jitter and never read as stopped
                H = np.array(hist[-STILL_N:])
                if np.all(H.max(0) - H.min(0) < self.still_spread):
                    self.settle_log.append((self.t - t0, False)); return True
        worst = self.names[d[int(np.argmax(np.abs(q[d] - self.q_goal[d])))]]
        if self.settle_mode == 'still':
            v = np.abs(self.robot.get_joint_velocities()[self.still_idx])
            worst += f' (fastest {self.names[self.still_idx[int(np.argmax(v))]]} {np.max(v):.3f})'
            if os.environ.get('SETTLE_DEBUG') and hist:
                H = np.array(hist); spread = H.max(0) - H.min(0)
                print('      [settle] timeout; 0.2 s spread (deg / mm): ' + ', '.join(
                    f'{self.names[i]} {spread[k] * (1000 if self.names[i] == "lift" else 57.3):.2f} v {v[k]:.3f}'
                    for k, i in enumerate(self.still_idx) if spread[k] > 1e-4 or v[k] > 0.02), flush=True)
        self.settle_log.append((self.t - t0, worst)); return False

    def set_pose(self, q_all):
        """Teleport the joints (start of a task only)."""
        q0 = np.zeros(len(self.names))
        for k, v in q_all.items(): q0[self.idx[k]] = v
        self.robot.set_joint_positions(q0); self.q_goal[:] = q0; self.q_cmd[:] = q0; self.q_sent[:] = q0

    def set_arm(self, side, q):
        for k, v in q.items():
            if k == 'lift': self.q_goal[self.idx['lift']] = v
            elif k in ARM_KEYS: self.q_goal[self.idx[f'{side}_{k}']] = v

    def set_jaw(self, side, v):
        self.q_goal[self.idx[f'{side}_finger_right']] = self.q_goal[self.idx[f'{side}_finger_left']] = v

    def jaw(self, side, v, seconds=0.7):
        """Open / close a gripper. jaw_mode 'fixed' (default) waits `seconds`; 'stop' (scene_v0) waits until both jaws
        have stopped (on the product, or at the target) for 4 steps, at least 0.15 s, at most `seconds`."""
        self.set_jaw(side, v)
        if self.jaw_mode != 'stop':
            self.wait(seconds); return
        ii = [self.idx[f'{side}_finger_right'], self.idx[f'{side}_finger_left']]
        n = still = 0
        moved = False                     # a stop counts only after the jaws moved, or once the command got there (a target
        while n < seconds * CTRL_HZ:      # past the joint limit leaves the jaws still while the command winds back)
            self.step_once(); n += 1
            v = np.abs(self.robot.get_joint_velocities()[ii])
            moved |= bool(np.any(v > 3 * JAW_STILL_V))
            arrived = np.max(np.abs(self.q_cmd[ii] - self.q_goal[ii])) < 1e-6
            still = still + 1 if np.all(v < JAW_STILL_V) else 0
            if n >= 0.15 * CTRL_HZ and still >= 4 and (moved or arrived):
                break
        self.jaw_log.append(n / CTRL_HZ)

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
        sides = [s for s in ('left', 'right') if f'{s}_shoulder_pan' in self.idx]   # the mono layout has only 'left'
        a = {s: self.arm_now(s) for s in sides}
        tgt = {s: {k: q[k] for k in q if k in ARM_KEYS} for s, q in (('left', qL), ('right', qR)) if s in sides}
        l0 = a[sides[0]]['lift']; l1 = l0 if lift is None else lift
        Ts = [0.05, abs(l1 - l0) / (0.88 * self.vmax[self.idx['lift']])]
        for s in tgt:
            Ts += [abs(tgt[s][k] - a[s][k]) / (0.88 * self.vmax[self.idx[f'{s}_{k}']]) for k in tgt[s]]
        Tm, prof = _profile(self.profile, max(Ts))
        steps = max(2, int(Tm * CTRL_HZ) + 1)
        for i in range(1, steps + 1):
            u = prof(i, steps)
            for s in tgt:
                self.set_arm(s, {k: a[s][k] + (tgt[s][k] - a[s][k]) * u for k in tgt[s]})
            self.q_goal[self.idx['lift']] = l0 + (l1 - l0) * u
            self.step_once()
        self.settle(timeout=8.0)

    def lift_to(self, v):
        self.joint_move2({}, {}, lift=v)

    # ------------------------------------------------------------ both arms ASYNCHRONOUS (2026-11-25, user: "asynchronous,
    # not waiting for the other arm"): each arm runs its own chain of steps on its own clock, the lift optionally moves at
    # the same time; the call returns when everything has finished (one settle at the end, not one per step).
    #   steps: ('joint', q)  arm joints only, time-synchronised within the arm, at 0.88 x its joint speed limits
    #          ('cart', a, b, elbow)  straight TCP line at the CURRENT lift (a and b at one height: z moves are shared)
    #          ('jaw', v, max_s)  open / close until the jaws stop (as jaw_mode 'stop'), at most max_s
    #          ('wait', s)
    def parallel(self, chains, lift=None, lift_after=0.0):
        """chains: {'left': [steps], 'right': [steps]}; lift: target moved at the same time (after `lift_after` s)."""
        gens = [self._g_chain(s, steps) for s, steps in chains.items() if steps]
        if lift is not None:
            gens.append(self._g_lift(lift, lift_after))
        while gens:
            for g in list(gens):
                try:
                    next(g)
                except StopIteration:
                    gens.remove(g)
            if gens:
                self.step_once()
        self.settle(timeout=3.0)

    def _g_chain(self, side, steps):
        for st in steps:
            kind = st[0]
            if kind == 'joint': yield from self._g_joint(side, st[1])
            elif kind == 'cart': yield from self._g_cart(side, *st[1:])
            elif kind == 'jaw': yield from self._g_jaw(side, *st[1:])
            elif kind == 'wait':
                for _ in range(int(st[1] * CTRL_HZ)): yield
            else: raise ValueError(kind)

    def _g_joint(self, side, q):
        a = self.arm_now(side); keys = [k for k in q if k in ARM_KEYS]
        lim = {k: 0.88 * self.vmax[self.idx[f'{side}_{k}']] for k in keys}
        T, prof = _profile(self.profile, max([abs(q[k] - a[k]) / lim[k] for k in keys] + [0.05]))
        steps = max(2, int(T * CTRL_HZ) + 1)
        for i in range(1, steps + 1):
            s = prof(i, steps)
            self.set_arm(side, {k: a[k] + (q[k] - a[k]) * s for k in keys}); yield

    def _g_cart(self, side, a, b, elbow):
        a, b = np.asarray(a, float), np.asarray(b, float)
        d = np.abs(b - a)
        if d[2] > 0.002:
            raise RuntimeError(f'{side}: an asynchronous straight move must stay at one height (dz {d[2] * 1000:.0f} mm)')
        seconds, prof = _profile(self.profile, max(float(np.linalg.norm(d[:3])) / TCP_SPEED, max(d[3], d[4]) / TCP_TURN))
        steps = max(2, int(max(MIN_MOVE, seconds) * CTRL_HZ))
        lift_now = float(self.q_goal[self.idx['lift']])
        for k in range(1, steps + 1):
            q = K.ik(side, *(a + (b - a) * prof(k, steps)), elbow=elbow)
            if q is None:
                raise RuntimeError(f'{side}: no IK on the straight move at {np.round(a + (b - a) * prof(k, steps), 3)}')
            if abs(q['lift'] - lift_now) > 0.003:
                raise RuntimeError(f'{side}: straight move needs lift {q["lift"]:.3f}, the lift is at {lift_now:.3f}')
            self.set_arm(side, {k_: v for k_, v in q.items() if k_ != 'lift'}); yield

    def _g_jaw(self, side, v, seconds=0.7):
        self.set_jaw(side, v)
        ii = [self.idx[f'{side}_finger_right'], self.idx[f'{side}_finger_left']]
        n = still = 0; moved = False
        while n < seconds * CTRL_HZ:
            yield; n += 1
            vel = np.abs(self.robot.get_joint_velocities()[ii])
            moved |= bool(np.any(vel > 3 * JAW_STILL_V))
            arrived = np.max(np.abs(self.q_cmd[ii] - self.q_goal[ii])) < 1e-6
            still = still + 1 if np.all(vel < JAW_STILL_V) else 0
            if n >= 0.15 * CTRL_HZ and still >= 4 and (moved or arrived):
                break
        self.jaw_log.append(n / CTRL_HZ)

    def _g_lift(self, target, delay=0.0):
        for _ in range(int(delay * CTRL_HZ)): yield
        i = self.idx['lift']; l0 = float(self.q_goal[i])
        T, prof = _profile(self.profile, max(0.05, abs(target - l0) / (0.88 * self.vmax[i])))
        steps = max(2, int(T * CTRL_HZ) + 1)
        for k in range(1, steps + 1):
            self.q_goal[i] = l0 + (target - l0) * prof(k, steps); yield

    def cart_move2(self, aL, bL, eL, aR, bR, eR):
        """Straight TCP lines for both hands at once; the lift must agree (same heights), so z moves are shared."""
        aL, bL, aR, bR = (np.asarray(v, float) for v in (aL, bL, aR, bR))
        d = np.maximum(np.abs(bL - aL), np.abs(bR - aR))
        vlift = 0.85 * self.vmax[self.idx['lift']]
        base = max(float(np.linalg.norm(d[:3])) / TCP_SPEED, max(d[3], d[4]) / TCP_TURN, d[2] / vlift)
        seconds, prof = _profile(self.profile, base); seconds = max(MIN_MOVE, seconds)
        steps = max(2, int(seconds * CTRL_HZ))
        for k in range(1, steps + 1):
            u = prof(k, steps)
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
        if self.planar:                                     # the articulation root is the floor anchor: read the joints
            q = self.robot.get_joint_positions(); x, y, yaw = (float(q[i]) for i in self.base_idx)
            return np.array([x, y, 0.0]), math.atan2(math.sin(yaw), math.cos(yaw))
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
        if self.planar:
            return self._planar_move(goal, label)
        R = 0.05; LX, LY = WHEELS.get(self.layout, (0.17, 0.222))
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

    def _planar_move(self, goal, label, speed=1.0, turn=1.5, accel=1.5, alpha=3.0, settle=0.3):
        """Base move on the planar joints: straight line + turn together on a trapezoid profile (user 2026-11-17: max
        1 m/s): accelerate at `accel` m/s2 (`alpha` rad/s2) to `speed` (`turn`), cruise, brake. The slower of the two
        (line, turn) sets the time; the other follows the same profile. Shortest way round for the heading."""
        ix, iy, iw = self.base_idx
        q0 = self.q_goal[[ix, iy, iw]].copy()
        dyaw = math.atan2(math.sin(goal[2] - q0[2]), math.cos(goal[2] - q0[2]))
        q1 = np.array([goal[0], goal[1], q0[2] + dyaw])

        def trap(d, v, a):                                   # (total time, ramp time) for distance d
            if d < 1e-9: return 0.0, 0.0
            if d >= v * v / a: return d / v + v / a, v / a
            tr = math.sqrt(d / a); return 2 * tr, tr
        T, tr = max(trap(math.hypot(*(q1[:2] - q0[:2])), speed, accel), trap(abs(dyaw), turn, alpha), (0.3, 0.15))
        tr = min(tr, T / 2); vp = 1.0 / (T - tr)            # peak rate of the path parameter s (0 -> 1)
        n = max(1, int(round(T * CTRL_HZ)))
        for i in range(1, n + 1):
            t = T * i / n
            if t < tr: s = 0.5 * vp / tr * t * t
            elif t > T - tr: s = 1.0 - 0.5 * vp / tr * (T - t) ** 2
            else: s = vp * (t - tr / 2)
            self.q_goal[[ix, iy, iw]] = q0 + (q1 - q0) * min(1.0, s)
            self.step_once()
        self.wait(settle)
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
        to = [(t, j) for t, j in self.settle_log if j]
        print(f"settling: {sum(t for t, _ in self.settle_log):.1f} s in {len(self.settle_log)} moves; {len(to)} timed out ({sum(t for t, _ in to):.1f} s), joints: { {j: sum(1 for _, k in to if k == j) for _, j in to} }")
        if self.jaw_log:
            print(f"jaws: {sum(self.jaw_log):.1f} s in {len(self.jaw_log)} actions (ended when the jaws stopped)")
