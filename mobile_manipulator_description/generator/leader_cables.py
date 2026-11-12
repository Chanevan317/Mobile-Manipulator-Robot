"""Leader arm v3: cable and board checks (2026-11-12; the first design had no room for any servo plug).

    python3 leader_cables.py            (~a few minutes)

1. Plugs: every servo's cable plug (housing + the first 5 mm of wire) is clear of every part over each joint's range.
2. Cable spaces: the space each cable needs is clear over the joint it crosses (the loop absorbs the turning):
   shoulder (loop around the support ring), elbow / wrist pan (a ring-shaped gap between the servo top and the raised
   idler prong), pitch (inside the moved-out idler prong), roll (around the gripper's back), plus the runs between
   (down the wrist's front, behind the pitch fork's back plate, over the roll servo, along the gripper top).
3. Board: the SO-101 Waveshare plate (real STL) sits in the pedestal's dovetail without touching, is held sideways,
   slides out upward; the plate + board (ASSUMED 16 mm tall) vs the arm in sampled poses (mono and proto).
4. Cable lengths each run needs (estimate, incl. the loop for the joint's range).
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
SPACES = [
    ('shoulder loop around the ring', 'base', union([annulus_z(LD.SUP['ro'] + 4.5, 62.0, -44.0, -28.0),     # below link 2 / wrist (z >= -26.7)
                                                     box(LD.SUP['ro'] + 0.1, LD.SUP['ro'] + 5.0, -CW / 2, CW / 2, LD.MOUNT_Z, -28.0)])),
    ('link 1 drop to the loop', 'link1', box(LD.CLIP1['x'] - CW / 2, LD.CLIP1['x'] + CW / 2, LD.CL['y'] + 0.05, LD.CL['y'] + CT,
                                            -44.0, -LD.CL['z'])),
    ('elbow loop', 'link1', annulus_z(10.5, 25.0, zb0, zb1, LD.L1, 0.0)),
    ('wrist-pan loop', 'link2', annulus_z(10.5, 25.0, zb0, zb1, LD.L2, 0.0)),
    ('wrist: over the top, down the front, under to S4', 'wrist', union([
        box(20.0, 30.0, -LD.W - CT - 0.2, -LD.W - 0.2, zb0, LD.PR_T[1]),               # up beside the top prong arm
        box(28.0, LD.WEB[1] + CT + 0.1, -CW / 2, CW / 2, LD.PR_T[1] + 0.05, LD.PR_T[1] + CT),   # over the arm + web top
        box(LD.WEB[1] + 0.05, LD.WEB[1] + CT, -CW / 2, CW / 2, -50.5, LD.PR_T[1]),     # down the web's front face
        box(41.0, LD.WEB[1] + CT, -24.0, CW / 2, -50.5, -46.0)])),                   # under the clamp to S4's -Y side
    ('pitch loop (inside the idler prong)', 'pitch', xf(annulus_z(11.0, 30.0, R + LD.PLUG_ENV['housing'] + 0.1, 24.4), Rx(90))),
    ('behind the back plate + over the roll servo + up', 'pitch', union([
        box(30.0, LD.XR - R - 4.05, -24.0, YY, 9.0, 19.0),
        box(30.0, LD.XR + LD.OUT_F - 0.4, -CW / 2, CW / 2, 12.2, 12.2 + CT),
        box(70.0, LD.XR + LD.OUT_F - 0.4, -CW / 2, CW / 2, 12.2, 40.0)])),
    ('roll loop (around the gripper back)', 'pitch', xf(annulus_z(33.0, 40.0, 81.0, 89.5), Ry(90))),       # the moving jaw's blade reaches back to 90.6 (open 40)
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


BASE = ('shoulder_mount', 'pedestal')


def folded_into_base(W):
    """The arm itself already touches a shoulder mount / pedestal (folded poses the follower can't take either)."""
    arm = [(n, s) for n, s, c, b in W if not b.endswith('base') and b != 'desk' and not n.split('_')[-2:-1] == [''] and
           not n.lstrip('LR_').startswith('S')]
    base = [s for n, s, c, b in W if n.endswith('shoulder_mount') or n.endswith('pedestal')]
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
    for a in np.arange(-130, 131, 13): qs.append(((0, float(a), 0, 0, 0), 0.0))
    for a in np.arange(-127, 128, 12.7): qs.append(((0, 0, float(a), 0, 0), 0.0))
    for a in np.arange(-20, 121, 10): qs.append(((0, 0, 0, float(a), 0), 0.0))
    for a in np.arange(-125, 126, 12.5): qs.append(((0, 0, 0, 0, float(a)), 0.0))
    for t in (0.0, 20.0, 40.0): qs.append(((0, 0, 0, 0, 0), t))
    for qp in (-20, 120):
        for qr in (-125, 125): qs.append(((0, 0, 0, qp, qr), 40.0))
    for q2 in (-130, 130):
        for q3 in (-127, 127): qs.append(((0, q2, q3, 90, 0), 0.0))
    return qs


def plate_solid():
    V = read_stl(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'CAD', 'SCARA_arm', 'SO100',
                              'WaveShare_Mounting_Plate_SO101.stl'))
    n = len(V); verts = V.reshape(-1, 3).astype(np.float32); tri = np.arange(3 * n, dtype=np.uint32).reshape(-1, 3)
    import manifold3d as m
    mesh = m.Mesh(vert_properties=verts, tri_verts=tri); mesh.merge()
    return M(mesh)


def board_in_pedestal_frame():
    """(plate, plate + board envelope) at rest in the socket, pedestal frame. STL: plate z 0..4.0, peg z 4.0..7.6
    (centred); peg side against the pad face (x = pad_x[1]), STL x -> Y, STL y -> Z."""
    B = LD.BOARD; zc = -LD.H_ARM + B['zc'] - 0.15; xf_ = B['pad_x'][1]
    Tp = np.array([[0, 0, -1, xf_ + 4.0], [1, 0, 0, 0], [0, 1, 0, zc], [0, 0, 0, 1]], float)
    plate = xf(plate_solid(), Tp)
    env = box(xf_, xf_ + 4.0 + B['comp'], -25.5, 25.5, zc - 21.0, zc + 21.0)
    return plate, env


def board_job(args):
    q, variant = args
    W = LD.world(q, 0.0, variant=variant); res = []
    if folded_into_base(W): return q, variant, None
    roots = LD.roots(variant)
    envs = [xf(BOARD_ENV, LD.ped_frame(R0, variant)) for R0 in roots]
    for k, e in enumerate(envs):
        bb = e.bounding_box()
        for nm, s, c, body in W:
            if body in ('desk',) or nm.endswith('pedestal'): continue
            sb = s.bounding_box()
            if any(bb[i] > sb[i + 3] or sb[i] > bb[i + 3] for i in range(3)): continue
            v = (e ^ s).volume()
            if v > TOL: res.append((k, nm, round(v, 1)))
    return q, variant, res


PLATE, BOARD_ENV = board_in_pedestal_frame()


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
    ped = LD.BASE_PARTS['pedestal']['solid']
    print('3. board plate in the socket: overlap at rest %.2f mm3 (must be ~0)' % (PLATE ^ ped).volume())
    held = {d: round((PLATE.translate(v) ^ ped).volume(), 1) for d, v in
            (('out +X 0.5', (0.5, 0, 0)), ('sideways +Y 0.5', (0, 0.5, 0)), ('sideways -Y 0.5', (0, -0.5, 0)),
             ('down 0.5', (0, 0, -0.5)))}
    print('   held (overlap mm3 if pushed, must be > 0):', held)
    up = [dz for dz in np.arange(0.0, 45.1, 3.0) if (PLATE.translate((0, 0, dz)) ^ ped).volume() > 0.05]
    print('   slides out upward:', 'yes' if not up else f'blocked at {up}')
    rng = np.random.default_rng(1)
    qs = []
    for _ in range(1500):
        qs.append((tuple(float(x) for x in (rng.uniform(-173, 173), rng.uniform(-130, 130), rng.uniform(-127, 127),
                                             rng.choice([0, 45, 90, 120]), 0.0)), 'mono'))
    for _ in range(600):
        qs.append((tuple(float(x) for x in (rng.uniform(-120, 120), rng.uniform(-130, 130), rng.uniform(-127, 127),
                                             rng.choice([0, 45, 90, 120]), 0.0)), 'proto'))
    with Pool(8) as P: res = P.map(board_job, qs)
    for variant in ('mono', 'proto'):
        rr = [(q, r) for q, v, r in res if v == variant and r is not None]
        bad = [(q, r) for q, r in rr if r]
        print(f'   board vs the arm ({variant}, {len(rr)} random poses not folded into the base): {len(bad)} touch it '
              f'({100 * len(bad) / max(1, len(rr)):.1f} %), e.g.', [q for q, r in bad[:3]])
    # 4. cable lengths (estimates: path + the loop for the joint's range at its radius, +15 %)
    est = {
        'board -> shoulder servo (window, up the column, cap hole)': 45 + ((LD.MOUNT_Z - 10.0) - (-LD.H_ARM + 16.0)) + 20,
        'shoulder servo -> elbow servo (channel, ring loop +-174 deg, link 1 clip)': 62 + 57 * math.radians(174) + 26 + 36 + 15,
        'elbow -> wrist-pan servo (elbow loop, along link 2)': 18 * math.radians(285) + 75 + 30,
        'wrist-pan -> pitch servo (loop, over the wrist, down its front, under)': 18 * math.radians(285) + 25 + 20 + 80 + 30,
        'pitch -> roll servo (pitch loop, behind the back plate)': 20 * math.radians(140) + 30 + 20,
        'roll -> gripper servo (over the roll servo, roll loop +-125 deg, gripper top)': 40 + 25 + 36 * math.radians(250) + 30,
    }
    print('4. cable lengths needed (mm, +15 %):')
    for k, v in est.items(): print(f'   {k}: {v * 1.15:.0f}')
