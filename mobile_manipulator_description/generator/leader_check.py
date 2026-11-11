"""Leader arm v3 checks: clashes between bodies (printed parts + servos + base) at the zero pose, over each joint's
range (to 5 deg past the leader limits, which are >= the follower's + 5), in combined poses (folded arm, hand hanging
down), over the trigger range, the desk clearance, both bases (mono: shoulder +-180; proto: two arms at yaw +-135),
and the printed volume."""
import sys, itertools
sys.path.insert(0, '.')
import numpy as np
from multiprocessing import Pool
import leader as LD

TOL = 1.0     # mm3: contact skins between a plate and the servo disc / pocket it sits on are ~0
VARIANT = 'mono'


def clashes(q, trig=0.0, variant=None, q2=None, tol=TOL):
    W = LD.world(q, trig, variant=variant or VARIANT, q2=q2); bad = []
    bbs = [w[1].bounding_box() for w in W]
    for (a, ba), (b, bb) in itertools.combinations(list(zip(W, bbs)), 2):
        (n1, s1, c1, b1), (n2, s2, c2, b2) = a, b
        if b1 == b2: continue
        if {b1.split('_')[-1] if b1 != 'desk' else b1, b2.split('_')[-1] if b2 != 'desk' else b2} == {'base', 'desk'}: continue
        if any(ba[k] > bb[k + 3] or bb[k] > ba[k + 3] for k in range(3)): continue
        v = (s1 ^ s2).volume()
        if v > tol: bad.append((n1, n2, round(v, 1)))
    return bad


def lowest(q, trig=0.0):
    return min(w[1].bounding_box()[2] for w in LD.world(q, trig, bought=True) if not w[3].endswith('base') and w[3] != 'desk') + LD.H_ARM


def _job(args):
    q, trig, variant = args
    return clashes(q, trig, variant)


def run(jobs):
    with Pool(8) as P: return P.map(_job, jobs)


if __name__ == '__main__':
    LD.build()
    tot = sum(p['solid'].volume() for p in LD.PARTS.values()) / 1000
    print(f'arm module printed volume {tot:.0f} cm3 (~{tot * 1.24 * 0.45:.0f} g at 15 % infill, ~45 % effective fill)')
    for nm, p in list(LD.PARTS.items()) + list(LD.BASE_PARTS.items()):
        print(f'  {nm:16s} components {len(p["solid"].decompose())}  genus {p["solid"].genus()}')
    print('zero pose (mono):', clashes((0, 0, 0, 0, 0)) or 'clear')
    idx = {'J1': 0, 'J2': 1, 'J3': 2, 'pitch': 3, 'roll': 4}
    for nm, i in idx.items():
        lo, hi = LD.LIMITS[nm]; angs = [float(a) for a in np.arange(lo - 5, hi + 5.01, 5)]
        if nm == 'J1': angs = [float(a) for a in np.arange(-180, 180.01, 10)]
        jobs = []
        for a in angs:
            q = [0, 0, 0, 0, 0]; q[i] = a; jobs.append((tuple(q), 0.0, 'mono'))
        res = [(j[0][i], c[:2]) for j, c in zip(jobs, run(jobs)) if c]
        print(f'{nm:6s} {LD.LIMITS[nm]} (+-5): {"clear" if not res else res}')
    # first contact of the folding joints
    for nm, i in (('J2', 1), ('J3', 2)):
        for sg in (1, -1):
            for a in range(130, 181, 2):
                q = [0, 0, 0, 0, 0]; q[i] = sg * a
                c = clashes(tuple(q))
                if c: print(f'{nm} first contact at {sg * a:+d} deg: {c[:2]}'); break
    combos = []
    for j3 in (-127, -90, 0, 90, 127):
        for p in (0, 90, 120):
            for r in (-90, 0, 90):
                combos.append((0, 0, j3, p, r))
    for q2 in (-130, -90, 90, 130):
        for p in (0, 90, 120):
            for j3 in (-127, 0, 127):
                combos.append((0, q2, j3, p, 0))
    for q1 in (-180, -90, 90, 180):
        combos += [(q1, 0, 0, 90, 0), (q1, 130, -127, 90, 0), (q1, -130, 127, 90, 0), (q1, 130, 127, 120, 90)]
    res = run([(q, 0.0, 'mono') for q in combos])
    bad = [(q, c[:2]) for q, c in zip(combos, res) if c]
    print(f'combined poses (mono): {len(combos)} checked,', 'all clear' if not bad else f'{len(bad)} with contact:')
    for b in bad[:12]: print('   ', b)
    for t in (-4, 0, 10, 20, 30, 40, 45):
        print(f'jaw {t:3d} deg:', clashes((0, 0, 0, 0, 0), t) or 'clear')
    for pr in ((0, 0, 0, 90, 90), (0, 0, 0, 120, -90), (0, 0, 0, -20, 0)):
        print(f'jaw 40 at pitch {pr[3]} roll {pr[4]}:', clashes(pr, 40) or 'clear')
    low = min(lowest(q) for q in [(0, 0, 0, 90, r) for r in (-90, 0, 90)] + [(0, 0, 0, 120, 0), (0, 0, 90, 90, 90)])
    print(f'lowest point with the hand hanging down: {low:.1f} mm above the desk')
    # proto base: both arms, J1 within the prototype's +-120, a few poses each, plus the two hands reaching inward
    pj = []
    for q1 in (-120, -60, 0, 60, 120):
        for q2_ in (-130, 0, 130):
            pj.append(((q1, q2_, 0, 90, 0), 0.0, 'proto'))
    res = run(pj)
    bad = [(j[0], c[:2]) for j, c in zip(pj, res) if c]
    print(f'proto base (2 arms, mirrored poses): {len(pj)} checked,', 'all clear' if not bad else f'{len(bad)} with contact:')
    for b in bad[:10]: print('   ', b)
