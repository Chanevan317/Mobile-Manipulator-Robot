"""Leader arm checks: clashes between bodies at zero pose and over each joint's range, mass estimate."""
import sys, itertools
sys.path.insert(0, '.')
import leader as LD

def clashes(q, trig=0.0, tol=0.5):
    W = LD.world(q, trig); bad = []
    for (n1, s1, c1, b1), (n2, s2, c2, b2) in itertools.combinations(W, 2):
        if b1 == b2 or 'spacer' in n1 + n2: continue
        v = (s1 ^ s2).volume()
        if v > tol: bad.append((n1, n2, round(v, 1)))
    return bad

if __name__ == '__main__':
    LD.build()
    tot = sum(p['solid'].volume() for p in LD.PARTS.values()) / 1000
    print(f'printed volume {tot:.0f} cm3 -> ~{tot * 1.24 * 0.45:.0f} g at 2 walls / 15 % (PLA, ~45 % effective fill)')
    z = clashes((0, 0, 0, 0, 0)); print('zero pose:', z or 'clear')
    sweeps = {'J1': 0, 'J2': 1, 'J3': 2, 'pitch': 3, 'roll': 4}
    for nm, i in sweeps.items():
        lo, hi = LD.LIMITS[nm]; first = {}
        for sgn, end in ((1, hi), (-1, lo)):
            a = 0; hit = None
            while abs(a) <= abs(end) + 10:
                q = [0, 0, 0, 0, 0]; q[i] = a
                c = clashes(q)
                if c: hit = (a, c[:2]); break
                a += sgn * 5
            first[sgn] = hit
        print(f'{nm:6s} limits {LD.LIMITS[nm]}: first contact +side {first[1]}, -side {first[-1]}')
    for t in (0, 15, 30):
        print('trigger', t, clashes((0, 0, 0, 0, 0), t)[:2] or 'clear')
