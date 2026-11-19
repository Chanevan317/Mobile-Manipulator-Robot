"""Offline collision check + transfer planning for the one-arm robot (no simulator; the generator's URDF FK and
manifold3d, so it runs in env_isaaclab or any env with manifold3d + numpy).

    import collide as CO
    chk = CO.Checker('prototype_mono_arm')
    env = CO.boxes_to_base(SC.furniture_boxes(), base_xy, base_yaw)     # world-aligned boxes -> the robot's base frame
    chk.check(q, env, held=CO.held_box(...))      # [(a, b, mm3)] overlaps; q = scara_ik joint dict (+ 'jaw')
    moves = chk.transfer(q0, q1, env, held)       # [q1] if the straight joint move is clear, else [waypoint, q1], else None

Checks the arm's links against the robot's own static links (base, column, tote, carriage, ring, scene camera), the
arm's non-adjacent links against each other, the furniture boxes, and an optional carried product (a box in the TCP
frame) against all of them. It uses the collision meshes of the layout's URDF (wheels and rollers left out).
Same idea as the scratch checkers used for the mono task plans (mono_poses lessons); found 2026-11-18 that a direct
swing from the travel fold to a shelf grasp hits the grey shelf's front post (pick_to_tote.py).
"""
import collections, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import repo_paths as RP
sys.path.append(RP.GENERATOR)          # appended: its module names must not shadow the scripts' ones
from urdf_fk import Robot, org             # noqa: E402  (generator modules)
from lib import union, xf, M               # noqa: E402
import scara_ik as K                        # noqa: E402

SKIP = {('left_upper_arm', 'left_base'), ('left_upper_arm', 'carriage')}     # the ring sits on these
HAND = ('left_hand', 'left_finger_right', 'left_finger_left', 'left_tilt', 'left_wrist')


def cube(c, size):
    return M.cube(tuple(float(v) for v in size)).translate(tuple(float(v) for v in np.asarray(c) - np.asarray(size) / 2))


def boxes_to_base(boxes, base_xy, base_yaw, margin=0.0):
    """World-aligned boxes (name, centre, size) -> the base frame; turned boxes become their axis-aligned bounds
    (exact for yaws that are multiples of 90 deg, as the stations are). margin grows every box on each side (m)."""
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    out = []
    for nm, ctr, sz in boxes:
        dx, dy = ctr[0] - base_xy[0], ctr[1] - base_xy[1]
        bc = (c * dx + s * dy, -s * dx + c * dy, ctr[2])
        bs = (abs(c) * sz[0] + abs(s) * sz[1] + 2 * margin, abs(s) * sz[0] + abs(c) * sz[1] + 2 * margin, sz[2] + 2 * margin)
        out.append((nm, bc, bs))
    return out


def held_box(centre_tcp, size_tcp, shrink=0.002):
    """A carried product as a box in the TCP frame (x along the hand, y across the jaws, z = the hand's up when level)."""
    return (np.asarray(centre_tcp, float), np.asarray(size_tcp, float) - 2 * shrink)


class Checker:
    def __init__(self, layout='prototype_mono_arm'):
        self.layout = layout
        K.use(layout)
        self.R = Robot(RP.PKG, layout)
        self.par = {j.find('child').get('link'): j.find('parent').get('link') for j in self.R.J}
        fixedc = {j.find('child').get('link') for j in self.R.J if j.get('type') == 'fixed'}

        def body(l):
            while l in fixedc:
                l = self.par[l]
            return l
        self.body = body
        kids = collections.defaultdict(list)
        for ch, p in self.par.items():
            kids[p].append(ch)
        arm, st = set(), ['left_base']
        while st:
            l = st.pop()
            for ch in kids[l]:
                arm.add(ch); st.append(ch)
        self.arm = arm
        self.local = {}
        for nm, l in self.R.links.items():
            if 'wheel' in nm or 'roller' in nm:
                continue
            sol = [xf(self.R.geom(e.find('geometry')), org(e)) for e in l.findall('collision')]
            if sol:
                self.local[nm] = union(sol)
        self.bpar = {body(k): body(v) for k, v in self.par.items() if body(k) != body(v)}

    @staticmethod
    def q_urdf(q):
        out = {}
        for k, v in q.items():
            if k == 'lift' or k.startswith('scene'):
                out[k] = v
            elif k == 'jaw':
                out['left_finger_right'] = v; out['left_finger_left'] = v
            else:
                out['left_' + k] = v
        return out

    def solids(self, q):
        W = self.R.fk(self.q_urdf(q))
        B = collections.defaultdict(list)
        for nm, s in self.local.items():
            B[self.body(nm)].append(xf(s, W[nm]))
        return {k: union(v) for k, v in B.items()}, W

    def check(self, q, env=(), held=None, thr=2.0):
        """[(a, b, mm3)] overlapping pairs above thr mm3. held: held_box(...) or None."""
        B, W = self.solids(q)
        bb = {k: np.array(v.bounding_box()) for k, v in B.items()}
        bodies = list(B)
        armb = sorted(b for b in bodies if b in self.arm)
        stat = [b for b in bodies if b not in self.arm]
        pairs = [(a, s) for a in armb for s in stat if (a, s) not in SKIP]
        for i in range(len(armb)):
            for j in range(i + 1, len(armb)):
                a, b = armb[i], armb[j]
                if self.bpar.get(a) == b or self.bpar.get(b) == a or {a, b} <= set(HAND[:3]):
                    continue
                pairs.append((a, b))
        extra = {nm: cube(c, sz) for nm, c, sz in env}
        if held is not None:
            T = W['left_tcp']; ctr, sz = held
            Tc = T.copy(); Tc[:3, 3] = T[:3, :3] @ ctr + T[:3, 3]
            extra['HELD'] = xf(cube((0, 0, 0), sz), Tc)
        for nm, s in extra.items():
            B[nm] = s; bb[nm] = np.array(s.bounding_box())
            for b in bodies:
                if nm == 'HELD' and b in HAND:
                    continue
                pairs.append((nm, b))
        if held is not None:
            pairs += [('HELD', nm) for nm in extra if nm != 'HELD']
        bad = []
        for a, b in pairs:
            A, Bx = bb[a], bb[b]
            if np.all(A[:3] <= Bx[3:] + 1e-4) and np.all(Bx[:3] <= A[3:] + 1e-4):
                v = (B[a] ^ B[b]).volume() * 1e9
                if v > thr:
                    bad.append((a, b, round(v, 1)))
        return bad

    def path_clear(self, q0, q1, env=(), held=None, n=14):
        """Straight joint-space move (as proto_task.Task.joint_move) sampled n times."""
        keys = [k for k in q1 if k in q0]
        for t in np.linspace(0.0, 1.0, n)[1:]:
            q = dict(q0); q.update({k: q0[k] + t * (q1[k] - q0[k]) for k in keys})
            if self.check(q, env, held):
                return False
        return True

    TUCKS = ((-100, -30), (100, 30), (-110, -120), (110, 120), (-110, 60), (110, -60), (-60, -120), (60, 120))   # deg

    def plan_clear(self, q0, moves, env=(), held=None, n=12):
        q = q0
        for m in moves:
            if not self.path_clear(q, m, env, held, n):
                return False
            q = m
        return True

    def _options(self, q0, q1):
        """Candidate move lists from q0 to q1: straight, then 'tuck' (elbow + wrist pan folded in at q0's ring angle, turn
        the ring and the wrist to q1's with the arm folded, unfold)."""
        yield [q1]
        D = math.radians
        for e_, w_ in self.TUCKS:
            t = {'elbow_pan': D(e_), 'wrist_pan': D(w_)}
            yield [{**q0, **t}, {**q1, **t}, q1]

    def transfer(self, q0, q1, env=(), held=None, lifts=None, step=30, n=12, max_tests=150):
        """Joint targets that take the arm from q0 to q1 without contact (a list ending with q1), or None. Tries, in
        order: the straight move; tuck moves; the same at other lift heights from `lifts` (lift there with the arm still,
        move, lift back: a swing past a shelf front fits only between the products and the next board); a grid search
        for one waypoint (max_tests clear candidates)."""
        q1 = {**q0, **q1}
        for opt in self._options(q0, q1):
            if all(not self.check(m, env, held) for m in opt[:-1]) and self.plan_clear(q0, opt, env, held, n):
                return opt
        for L in sorted(lifts if lifts is not None else (), key=lambda L: abs(L - q1['lift'])):
            a0, a1 = {**q0, 'lift': float(L)}, {**q1, 'lift': float(L)}
            if self.check(a0, env, held) or self.check(a1, env, held):
                continue
            if not (self.path_clear(q0, a0, env, held, 8) and self.path_clear(a1, q1, env, held, 8)):
                continue
            for opt in self._options(a0, a1):
                if all(not self.check(m, env, held) for m in opt[:-1]) and self.plan_clear(a0, opt, env, held, n):
                    return [a0] + opt + [q1]
        D = math.radians; keys = ('shoulder_pan', 'elbow_pan', 'wrist_pan')
        lim = {k: K.LIMITS[k] for k in keys}
        cands = []
        for s_ in range(-180, 181, step):
            for e_ in range(-110, 111, step):
                for w_ in range(-120, 121, step):
                    W = {**q1, 'shoulder_pan': D(s_), 'elbow_pan': D(e_), 'wrist_pan': D(w_)}
                    if any(not (lim[k][0] <= W[k] <= lim[k][1]) for k in keys):
                        continue
                    cands.append((sum(abs(W[k] - q0[k]) + abs(q1[k] - W[k]) for k in keys), W))
        cands.sort(key=lambda c: c[0])
        tested = 0
        for _, W in cands:
            if self.check(W, env, held):
                continue
            tested += 1
            if self.path_clear(q0, W, env, held, n) and self.path_clear(W, q1, env, held, n):
                return [W, q1]
            if tested >= max_tests:
                break
        return None


# ---------------------------------------------------------------- parallel search (one lift height per process)
_W = {}


def _noop():
    return os.getpid()


def _worker_init(layout):
    _W['chk'] = Checker(layout)


def _worker_lift(args):
    q0, q1, env, held, L, max_tests = args
    chk = _W['chk']
    a0, a1 = {**q0, 'lift': float(L)}, {**q1, 'lift': float(L)}
    if chk.check(a0, env, held) or chk.check(a1, env, held):
        return None
    if not (chk.path_clear(q0, a0, env, held, 8) and chk.path_clear(a1, q1, env, held, 8)):
        return None
    moves = chk.transfer(a0, a1, env, held, lifts=None, max_tests=max_tests)
    return None if moves is None else [a0] + moves + [q1]


class Planner:
    """Checker + a pool of worker processes for the slow case: each tries the move at one lift height (lift there, move,
    lift back) with the full search. Workers start with the 'spawn' method, so it is safe inside an Isaac script."""
    def __init__(self, layout='prototype_mono_arm', workers=None):
        import concurrent.futures as cf, multiprocessing as mp
        self.chk = Checker(layout)
        n = workers or max(2, min(8, (os.cpu_count() or 4) // 2))
        self.pool = cf.ProcessPoolExecutor(n, mp_context=mp.get_context('spawn'), initializer=_worker_init, initargs=(layout,))
        self.n = n
        # start every worker now, with the main script hidden: 'spawn' children would otherwise re-run the main script
        # (an Isaac script would start a second simulator in each one)
        main = sys.modules['__main__']; saved = {k: getattr(main, k) for k in ('__file__', '__spec__') if hasattr(main, k)}
        try:
            if '__file__' in saved:
                del main.__file__
            main.__spec__ = None
            for f in [self.pool.submit(_noop) for _ in range(n)]:
                f.result()
        finally:
            for k, v in saved.items():
                setattr(main, k, v)
            if '__spec__' not in saved:
                del main.__spec__

    def transfer(self, q0, q1, env=(), held=None, lifts=(), max_tests=80):
        """As Checker.transfer, but the other lift heights are searched in parallel; the plan at the lift nearest to
        q1's wins among those found."""
        q1 = {**q0, **q1}
        moves = self.chk.transfer(q0, q1, env, held, lifts=None, max_tests=40)
        if moves is not None:
            return moves
        env = [(n_, tuple(map(float, c_)), tuple(map(float, z_))) for n_, c_, z_ in env]
        order = sorted((float(L) for L in lifts), key=lambda L: abs(L - q1['lift']))
        futs = [self.pool.submit(_worker_lift, (q0, q1, env, held, L, max_tests)) for L in order]
        for f in futs:                          # in order of preference: take the first lift that has a plan
            r = f.result()
            if r is not None:
                for g in futs:
                    g.cancel()
                return r
        return None

    def close(self):
        self.pool.shutdown(cancel_futures=True)
