"""Leader arm v2 checks: clashes between bodies (printed parts + servos) at the zero pose, over each joint's range
(to 5 deg past the leader limits), in combined poses (hand hanging down at the pan limits), over the trigger range,
the desk clearance, and the printed volume / mass."""
import sys, itertools
sys.path.insert(0, '.')
import numpy as np
import leader as LD

TOL = 1.0     # mm3: contact skins between a plate and the servo disc / pocket it sits on are ~0


def clashes(q, trig=0.0, tol=TOL):
    W = LD.world(q, trig); bad = []
    bbs = [w[1].bounding_box() for w in W]
    for (a, ba), (b, bb) in itertools.combinations(list(zip(W, bbs)), 2):
        (n1, s1, c1, b1), (n2, s2, c2, b2) = a, b
        if b1 == b2: continue
        if any(ba[k] > bb[k + 3] or bb[k] > ba[k + 3] for k in range(3)): continue
        v = (s1 ^ s2).volume()
        if v > tol: bad.append((n1, n2, round(v, 1)))
    return bad


def lowest(q, trig=0.0):
    return min(w[1].bounding_box()[2] for w in LD.world(q, trig) if w[3] != 'base') + LD.H_B   # moving parts, above the desk


if __name__ == '__main__':
    LD.build()
    tot = sum(p['solid'].volume() for p in LD.PARTS.values()) / 1000
    print(f'printed volume {tot:.0f} cm3 -> ~{tot * 1.24 * 0.45:.0f} g at 3 walls / 20 % (PLA, ~45 % effective fill)')
    for nm, p in LD.PARTS.items():
        print(f'  {nm:11s} components {len(p["solid"].decompose())}  genus {p["solid"].genus()}')
    print('zero pose:', clashes((0, 0, 0, 0, 0)) or 'clear')
    idx = {'J1': 0, 'J2': 1, 'J3': 2, 'pitch': 3, 'roll': 4}
    for nm, i in idx.items():
        lo, hi = LD.LIMITS[nm]; res = []
        for a in list(np.arange(lo - 5, hi + 5.01, 5)):
            q = [0, 0, 0, 0, 0]; q[i] = float(a)
            c = clashes(q)
            if c: res.append((float(a), c[:2]))
        print(f'{nm:6s} {LD.LIMITS[nm]} (+-5): {"clear" if not res else res}')
    combos = []
    for j in (-127, -90, 0, 90, 127):
        for p in (0, 90, 120):
            for r in (-90, 0, 90):
                combos.append((0, 0, j, p, r))
    for q2 in (-130, 130):
        for p in (90, 120):
            combos.append((0, q2, 0, p, 0)); combos.append((0, q2, 127, p, 0)); combos.append((0, q2, -127, p, 0))
    for q1 in (-120, 120):
        combos.append((q1, 0, 0, 90, 0)); combos.append((q1, 130, -127, 90, 0)); combos.append((q1, -130, 127, 90, 0))
    bad = [(q, clashes(q)[:2]) for q in combos]
    bad = [b for b in bad if b[1]]
    print(f'combined poses: {len(combos)} checked,', 'all clear' if not bad else f'{len(bad)} with contact:')
    for b in bad[:12]: print('   ', b)
    for t in (0, 10, 20, 25, 30):
        print(f'trigger {t:2d}:', clashes((0, 0, 0, 0, 0), t) or 'clear')
    low = min(lowest(q) for q in [(0, 0, 0, 90, r) for r in (-90, 0, 90)] + [(0, 0, 0, 120, 0)])
    print(f'lowest point with the hand hanging down: {low:.1f} mm above the desk')
