"""Leader arm v3: cable and board checks (2026-11-12; the first design had no room for any servo plug).

    python3 leader_cables.py            (~a few minutes)

1. Plugs: every servo's cable plug (housing + the first 5 mm of wire) is clear of every part over each joint's range.
2. Cable spaces: the space each cable needs is clear over the joint it crosses (v4, 2026-11-17: every cable 20 cm):
   no servo cable crosses the shoulder (S1 turns with link 1, the board rides on link 1); elbow / wrist pan: a
   ring-shaped gap between the servo top and the raised idler prong, left at CABLE_EXIT (-60 deg) beside the fork;
   the S3 -> S4 drop down the wrist's -Y side; pitch (inside the moved-out idler prong); behind the pitch fork's back
   plate and over the roll servo; the roll twist (around the roll servo's top, into the gripper frame's notch).
3. Board: the SO-101 Waveshare plate (real STL) sits in link 1's dovetail without touching, is held sideways, slides
   out upward; the plate + board (ASSUMED 16 mm tall) vs every other part in sampled poses (mono and proto).
4. Cable lengths each run needs: paths on the geometry, joint crossings as a wrap around the boss over the joint's
   whole range (worst case), plus 12 mm per plug end; must stay under 200 mm with 15 % to spare.
"""
import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from multiprocessing import Pool
import leader as LD
from lib import box, cyl, union, xf, T, Rx, Ry, Rz, M
from stlio import read_stl

LD.build()
LIM = LD.LIMITS
R = LD.RIM
X0, X1, YY = LD.PLUG['x'][0], LD.PLUG['x'][1], LD.PLUG['y']
HOUSING = box(X0 + 0.4, X1 - 0.4, -YY + 0.4, YY - 0.4, R, R + LD.PLUG_ENV['housing'])   # the slots are 0.4 mm bigger
WIRES = box(X0 + 1.5, X1 - 1.5, -YY + 1.0, YY - 1.0, R + LD.PLUG_ENV['housing'], R + LD.PLUG_ENV['housing'] + LD.PLUG_ENV['wires'])
CW, CT = LD.CABLE['w'], LD.CABLE['t']
TOL = 0.3                                  # mm3


def annulus_z(r0, r1, z0, z1, x=0.0, y=0.0):
    return cyl(r1, z0, z1, x, y) - cyl(r0, z0 - 1, z1 + 1, x, y)


# cable spaces, each: (name, frame it is fixed in, solid in that frame)
zb0, zb1 = R + LD.PLUG_ENV['housing'] + 0.2, LD.PR_T[0] - 0.3                       # gap under the raised idler prong
th = math.radians(LD.CABLE_EXIT)
EX = (15.0 * math.cos(th), 15.0 * math.sin(th))                                    # exit beside the prong (link frame)
F = LD.FLIP                                                                         # S2 / S3 turned over: their gaps are UNDER the joints
zu0, zu1 = -zb1, -zb0                                                               # the gap under S2 / S3 (-25.4 .. -19.6)
SPACES = [
    ('link 1: S1 -> S2 over the top, down the clip, under the clamp', 'link1', union([
        box(18.7, 54.0, -CW / 2, CW / 2, LD.CL['z'] + 0.05, LD.CL['z'] + CT),                   # over the S1 clamp + beam top
        box(LD.LINK1_CLIP['x'] - CW / 2, LD.LINK1_CLIP['x'] + CW / 2, -CW / 2, LD.CL['y'] + CT, LD.CL['z'] + 0.05, LD.CL['z'] + CT),
        box(LD.LINK1_CLIP['x'] - CW / 2, LD.LINK1_CLIP['x'] + CW / 2, LD.CL['y'] + 0.05, LD.CL['y'] + CT, -LD.CL['z'] - CT, LD.CL['z'] + CT),
        box(LD.LINK1_CLIP['x'] - CW / 2, LD.L1 - 18.5, -CW / 2, LD.CL['y'] + CT, -LD.CL['z'] - CT, -LD.CL['z'] - 0.05)])),   # under to S2's plug
    ('elbow loop (under S2)', 'link1', annulus_z(10.5, 25.0, zu0, zu1, LD.L1, 0.0)),
    ('elbow exit beside the fork, onto the clamp (link 2, under)', 'link2', xf(union([
        box(EX[0] - 3.0, 50.0, -LD.W - CT - 0.6, -LD.W - 0.3, zb0, zb1),
        box(46.5, LD.L2 + LD.CL['x'][0] - 0.05, -LD.W - CT - 0.6, CW / 2, LD.BEAM_Z + 0.05, LD.BEAM_Z + CT),   # across the short beam
        box(LD.L2 + LD.CL['x'][0] + 0.05, LD.L2 - 18.5, -CW / 2, CW / 2, LD.CL['z'] + 0.05, LD.CL['z'] + CT)]), F)),
    ('wrist-pan loop (under S3)', 'link2', annulus_z(10.5, 25.0, zu0, zu1, LD.L2, 0.0)),
    ('wrist: out beside the prong, down the -Y side to S4', 'wrist', union([
        box(EX[0] - 3.0, 22.4, -LD.CL['z'] - CT - 0.2, -LD.W - 0.3, zu0, zu1),
        box(17.6, 22.4, -LD.CL['z'] - CT + 0.2, -LD.CL['z'] - 0.1, -LD.ZPD + 16.0, zu1)])),
    ('pitch loop (inside the idler prong)', 'pitch', xf(annulus_z(11.0, 30.0, R + LD.PLUG_ENV['housing'] + 0.1, 24.4), Rx(90))),
    ('behind the back plate + over the roll servo', 'pitch', union([
        box(30.0, LD.XR - R - 4.05, -24.0, YY, 9.0, 19.0),
        box(30.0, LD.XR + LD.OUT_F - 0.4, -CW / 2, CW / 2, 12.4, 12.4 + CT)])),
    ('roll twist (around the roll servo top, +-90 deg)', 'pitch',
        xf(annulus_z(16.5, 21.0, 62.0, LD.XR + LD.OUT_F - 0.3), Ry(90)) - box(50.0, 90.0, -30.0, 30.0, -30.0, -3.0)),
]


def overlap(a, b):
    ab, bb = a.bounding_box(), b.bounding_box()
    if any(ab[i] > bb[i + 3] or bb[i] > ab[i + 3] for i in range(3)): return 0.0
    return (a ^ b).volume()


def check_space(job):
    k, q, trig = job
    name, frame, solid = SPACES[k]
    F = LD.frames(q, trig)
    S = xf(solid, F[frame])
    hits = []
    W = LD.world(q, trig, variant='mono')
    if folded_into_base(W): return name, q, trig, []                  # the arm is already folded into the base there
    for nm, s, c, body in W:
        if body == 'desk': continue
        v = overlap(S, s)
        if v > TOL: hits.append((nm, round(v, 1)))
    return name, q, trig, hits


BASE = ('shoulder_hub', 'pedestal')


def folded_into_base(W):
    """The arm itself already touches a shoulder mount / pedestal (folded poses the follower can't take either)."""
    arm = [(n, s) for n, s, c, b in W if not b.endswith('base') and b != 'desk' and not n.split('_')[-2:-1] == [''] and
           not n.lstrip('LR_').startswith('S')]
    base = [s for n, s, c, b in W if n.endswith('shoulder_hub') or n.endswith('pedestal')]
    return any(overlap(a, b) > 1.0 for _, a in arm for b in base)


def check_plug(job):
    q, trig = job
    F = LD.frames(q, trig); W = LD.world(q, trig, variant='mono'); hits = []
    skip_base = folded_into_base(W)
    for nm, body, Mx in LD.SERVOS:
        Tm = F[body] @ Mx
        for kind, env in (('housing', HOUSING), ('wires', WIRES)):
            e = xf(env, Tm)
            for n2, s2, c2, b2 in W:
                if n2 == nm or (skip_base and n2 in BASE): continue
                v = overlap(e, s2)
                if v > TOL: hits.append((nm, kind, n2, round(v, 1)))
    return q, trig, hits


def sweep_poses():
    qs = [((0, 0, 0, 0, 0), 0.0)]
    for a in np.arange(-173, 174, 15): qs.append(((float(a), 0, 0, 0, 0), 0.0))
    j2 = LIM['J2'][1]; rl = LIM['roll'][1]
    for a in np.arange(-j2, j2 + 1, 12): qs.append(((0, float(a), 0, 0, 0), 0.0))
    for a in np.arange(-127, 128, 12.7): qs.append(((0, 0, float(a), 0, 0), 0.0))
    for a in np.arange(-20, 121, 10): qs.append(((0, 0, 0, float(a), 0), 0.0))
    for a in np.arange(-rl, rl + 1, 10): qs.append(((0, 0, 0, 0, float(a)), 0.0))
    for t in (0.0, 20.0, 40.0): qs.append(((0, 0, 0, 0, 0), t))
    for qp in (-20, 120):
        for qr in (-rl, rl): qs.append(((0, 0, 0, qp, qr), 40.0))
    for q2 in (-j2, j2):
        for q3 in (-127, 127): qs.append(((0, q2, q3, 90, 0), 0.0))
    return qs


def plate_solid():
    V = read_stl(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'CAD', 'SCARA_arm', 'SO100',
                              'WaveShare_Mounting_Plate_SO101.stl'))
    n = len(V); verts = V.reshape(-1, 3).astype(np.float32); tri = np.arange(3 * n, dtype=np.uint32).reshape(-1, 3)
    import manifold3d as m
    mesh = m.Mesh(vert_properties=verts, tri_verts=tri); mesh.merge()
    return M(mesh)


def board_in_link1_frame():
    """(plate, plate + board envelope) at rest in link 1's socket. STL: plate z 0..4.0, peg z 4.0..7.6 (centred); the peg
    side against the socket face (x = face_x, looking -X): STL z -> -X, STL x -> Y, STL y -> Z."""
    B = LD.BOARD; zc = B['zc'] - 0.15; fx = B['face_x']
    Tp = np.array([[0, 0, 1, fx - 4.0], [1, 0, 0, 0], [0, 1, 0, zc], [0, 0, 0, 1]], float)
    plate = xf(plate_solid(), Tp)
    env = box(fx - 4.0 - B['comp'], fx, -25.5, 25.5, zc - 21.0, zc + 21.0)
    return plate, env


def board_job(args):
    q, variant = args
    W = LD.world(q, 0.0, variant=variant); res = []
    if folded_into_base(W): return q, variant, None
    qs_ = [q] if variant == 'mono' else [q, tuple(-v if i in (0, 1, 2, 4) else v for i, v in enumerate(q))]
    envs = [xf(BOARD_ENV, LD.frames(qa, 0.0, root=R0)['link1']) for R0, qa in zip(LD.roots(variant), qs_)]
    for k, e in enumerate(envs):
        bb = e.bounding_box(); own = '' if variant == 'mono' else ('L_', 'R_')[k]
        for nm, s, c, body in W:
            if body in ('desk',) or body == own + 'link1': continue
            sb = s.bounding_box()
            if any(bb[i] > sb[i + 3] or sb[i] > bb[i + 3] for i in range(3)): continue
            v = (e ^ s).volume()
            if v > TOL: res.append((k, nm, round(v, 1)))
    return q, variant, res


PLUG_END = 12.0                       # per plug: housing 3.5 + the wires' first bend


def _wrap_len(ra, rb, phi, r0):
    aa, ab = math.acos(min(1, r0 / ra)), math.acos(min(1, r0 / rb))
    if abs(phi) <= aa + ab:
        return math.sqrt(max(0.0, ra * ra + rb * rb - 2 * ra * rb * math.cos(phi)))
    return math.sqrt(ra * ra - r0 * r0) + math.sqrt(rb * rb - r0 * r0) + r0 * (abs(phi) - aa - ab)


def crossing(ra, rb, beta, rng, r0):
    """Cable around a joint's boss (r0) from the socket (ra, at 180 deg: toward the previous joint) to the exit (rb, at
    beta deg on the turning part): the longest it gets over +-rng, in the better of the two wrap directions."""
    out = None
    for k in (-1, 0, 1):
        phi0 = math.radians(beta - 180.0) + 2 * math.pi * k
        L = max(_wrap_len(ra, rb, phi0 + math.radians(q), r0) for q in np.linspace(-rng, rng, 241))
        out = L if out is None else min(out, L)
    return out


def cable_lengths():
    r0, ra, rb = LD.BOSS_R + 0.8, 14.25, 15.0; th = math.radians(LD.CABLE_EXIT)
    ex = (rb * math.cos(th), rb * math.sin(th))
    side = -LD.W - CT / 2 - 0.5
    B = LD.BOARD
    s1 = (14.25, 0.0)                                                    # S1 socket in link 1 (rear toward +X)
    out = {}
    out['board -> shoulder (on link 1, over S1)'] = (abs(B['face_x'] - 4.0 - B['comp'] / 2 - s1[0]) + (B['zc'] + 21.0 - 19.4)) + 2 * PLUG_END
    out['shoulder -> elbow (along link 1 top, down its side, under S2)'] = (LD.LINK1_CLIP['x'] - s1[0]) + (LD.CL['y'] + 2 * LD.CL['z'] + 4.0) + (LD.L1 - 14.25 - LD.LINK1_CLIP['x']) + 2 * PLUG_END
    post = math.hypot(48.0 - ex[0], side - ex[1]) + math.hypot(LD.L2 - 14.25 - 48.0, side) + 3.0
    out['elbow -> wrist pan (gap under S2, out at -60, along link 2)'] = crossing(ra, rb, LD.CABLE_EXIT, LIM['J2'][1], r0) + post + 2 * PLUG_END
    s4 = (LD.XP, -(R + LD.PLUG_ENV['housing'] + LD.PLUG_ENV['wires']), -LD.ZPD + 14.25)
    post = math.hypot(ex[0] - 20.0, -LD.CL['z'] - CT / 2 - ex[1]) + (-(zb0 + zb1) / 2 - s4[2]) + abs(s4[0] - 20.0) + 3.0
    out['wrist pan -> pitch (gap under S3, out at -60, down the -Y side)'] = crossing(ra, rb, LD.CABLE_EXIT, LIM['J3'][1], r0) + post + 2 * PLUG_END
    out['pitch -> roll (pitch loop, behind the back plate)'] = 20 * math.radians(140 + 10) + 30 + 20 + 2 * PLUG_END
    # roll: S5 socket (behind it, 14 mm under the axis) -> up behind the back plate -> over S5 -> twist +-90 at r ~18.5
    # around S5's top -> the notch -> up to the arch -> along the frame top -> S6's plug
    up = 13.5 + 14.25; over = (LD.XR + LD.OUT_F) - (LD.XR - R - 4.0 - 3.0)
    twist = 18.5 * math.radians(LIM['roll'][1]) - 0.0
    grip = (24.0 - LD.ROLL_NOTCH['z0']) + (LD.PX + 14.25) + (26.0 - 19.4)
    out['roll -> gripper (over S5, twist in the notch, along the top)'] = up + over + twist + grip + 2 * PLUG_END
    return out


PLATE, BOARD_ENV = board_in_link1_frame()


if __name__ == '__main__':
    poses = sweep_poses()
    with Pool(8) as P:
        # 1. plugs
        res = P.map(check_plug, poses)
        agg = {}
        for q, t, hits in res:
            for h in hits: agg.setdefault(h[:3], []).append((q, t, h[3]))
        print(f'1. plugs ({len(poses)} poses):', 'all clear' if not agg else '')
        for k, v in agg.items(): print('   HIT', k, len(v), 'poses, e.g.', v[0])
        # 2. cable spaces
        jobs = [(k, q, t) for k in range(len(SPACES)) for (q, t) in poses]
        res = P.map(check_space, jobs)
        print('2. cable spaces:')
        for (n, f, s) in SPACES:
            hits = {}
            for name, q, t, h in res:
                if name != n: continue
                for nm, v in h: hits.setdefault(nm, []).append((q, t, v))
            print(f'   {n}:', 'clear' if not hits else '')
            for nm, v in hits.items(): print('      HIT', nm, len(v), 'poses, e.g.', v[0])
    # 3. board
    ped = LD.PARTS['link1']['solid']
    print('3. board plate in link 1\'s socket: overlap at rest %.2f mm3 (must be ~0)' % (PLATE ^ ped).volume())
    held = {d: round((PLATE.translate(v) ^ ped).volume(), 1) for d, v in
            (('out -X 0.5', (-0.5, 0, 0)), ('sideways +Y 0.5', (0, 0.5, 0)), ('sideways -Y 0.5', (0, -0.5, 0)),
             ('down 0.5', (0, 0, -0.5)))}
    print('   held (overlap mm3 if pushed, must be > 0):', held)
    up = [dz for dz in np.arange(0.0, 45.1, 3.0) if (PLATE.translate((0, 0, dz)) ^ ped).volume() > 0.05]
    print('   slides out upward:', 'yes' if not up else f'blocked at {up}')
    rng = np.random.default_rng(1)
    qs = []
    for _ in range(1500):
        qs.append((tuple(float(x) for x in (rng.uniform(-173, 173), rng.uniform(*LIM['J2']), rng.uniform(-127, 127),
                                             rng.choice([0, 45, 90, 120]), 0.0)), 'mono'))
    for _ in range(600):
        qs.append((tuple(float(x) for x in (rng.uniform(-120, 120), rng.uniform(*LIM['J2']), rng.uniform(-127, 127),
                                             rng.choice([0, 45, 90, 120]), 0.0)), 'proto'))
    with Pool(8) as P: res = P.map(board_job, qs)
    for variant in ('mono', 'proto'):
        rr = [(q, r) for q, v, r in res if v == variant and r is not None]
        bad = [(q, r) for q, r in rr if r]
        print(f'   board vs the arm ({variant}, {len(rr)} random poses not folded into the base): {len(bad)} touch it '
              f'({100 * len(bad) / max(1, len(rr)):.1f} %), e.g.', [q for q, r in bad[:3]])
    # 4. cable lengths: paths on the geometry; a joint crossing = a wrap around the boss over the whole range (worst case)
    print('4. cable lengths needed (mm; + 12 per plug end; must be <= 200 / 1.15 = 174 for a 20 cm cable with 15 % spare):')
    for name, L in cable_lengths().items():
        print(f'   {name}: {L:5.0f}   (+15 % {L * 1.15:4.0f})  {"20 cm OK" if L * 1.15 <= 200 else "TOO LONG for 20 cm"}')
