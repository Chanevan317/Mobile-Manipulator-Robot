"""Store locations + product database of scene_v0 (no simulator needed).

Every board of the racks / shelves is cut into fixed-width slots, each with a location ID (scene.locations()):
    <unit>-B<board>-<slot>      unit R1 / R2 = blue racks, S1 / S2 = grey shelves; board 1 = the lowest;
                                slot 01 = the leftmost as seen from the aisle      e.g. R1-B1-03
    T1-01 .. T1-04              the pick spots on the table
The product database scenes/scene_v0/products.json says which locations each product covers (one or more, on one or
more boards). It is the source of truth for the store layout: scene.py builds the product runs from it (fillers stock
them, the shelf task products sit in them), and stations.py ties every robot station to the locations it serves.

    python3 scripts/scenes/scene_v0/store_db.py              check products.json, write locations.json, print the catalogue
    ... --product P03            where product P03 is (also a package name or part of a name: --product cereal)
    ... --at R1-B1-03            what is at a location

After editing products.json: run this script (checks it), then stations.py (new stations if runs moved), then
check_scene.py / save_scene.py in Isaac.
"""
import argparse, json, os, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
import scene as SC
import assets as SA

LOC_FILE = os.path.join(RP.scene_dir('scene_v0'), 'locations.json')


def short_ranges(ids):
    """['R1-B1-01', 'R1-B1-02', 'R1-B2-05'] -> 'R1-B1-01..02, R1-B2-05'"""
    out, run = [], []
    for l in sorted(ids):
        head, slot = l.rsplit('-', 1)
        if run and run[-1][0] == head and int(slot) == run[-1][2] + 1:
            run[-1] = (head, run[-1][1], int(slot))
        else:
            run.append((head, int(slot), int(slot)))
    for head, a, b in run:
        out.append(f'{head}-{a:02d}' + (f'..{b:02d}' if b != a else ''))
    return ', '.join(out)


def stations_by_location():
    try:                                                 # the file, not stations.py (that needs numpy)
        st = json.load(open(os.path.join(RP.scene_dir('scene_v0'), 'stations.json')))['stations']
    except FileNotFoundError:
        return {}
    out = {}
    for n, s in st.items():
        for l in s.get('locations', []):
            out.setdefault(l, []).append(n)
    return out


def find_product(key):
    P = SC.PRODUCTS
    if key in P:
        return [key]
    k = key.lower()
    return [pid for pid, p in P.items() if k in p['package'].lower() or k in p['name'].lower()]


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--product', help='product id, package or part of a name')
    ap.add_argument('--at', help='location ID')
    args = ap.parse_args()
    L, P = SC.LOCATIONS, SC.PRODUCTS
    owner = {l: pid for pid, p in P.items() for l in p['locations']}
    st = stations_by_location()
    if args.at:
        l = L.get(args.at)
        if l is None:
            sys.exit(f'unknown location {args.at}')
        pid = owner.get(args.at)
        print(f"{args.at}: {UN if (UN := SC.UNIT_NAMES[l['unit']]) else ''}, {l['section']}, board {l['level']} (z {l['z']} m), "
              f"slot {l['slot']}, {'reachable' if l['reachable'] else 'out of the arm reach (decor)'}")
        print(f"   product: {pid + ' ' + P[pid]['name'] if pid else 'empty'};  station: {', '.join(st.get(args.at, [])) or '-'}")
        sys.exit(0)
    if args.product:
        hits = find_product(args.product)
        if not hits:
            sys.exit(f'no product matches {args.product}')
        for pid in hits:
            p = P[pid]
            print(f"{pid} {p['name']} ({p['package']}): {p['mass_kg']} kg, {p['size_mm']} mm")
            for l in sorted(p['locations']):
                o = L[l]
                print(f"   {l}  {SC.UNIT_NAMES[o['unit']]}, board {o['level']} (z {o['z']} m), "
                      f"{'reachable' if o['reachable'] else 'decor'}, station {', '.join(st.get(l, [])) or '-'}")
        sys.exit(0)
    bad = SC.check_products()
    # mass / size must match the asset manifest (assets.py stays the source of the physics values)
    for pid, p in P.items():
        a = SA.PRODUCTS.get(p['package'])
        if a is None:
            bad.append(f"{pid}: package {p['package']} not in assets.py"); continue
        if abs(a['mass'] - p['mass_kg']) > 1e-6:
            bad.append(f"{pid}: mass {p['mass_kg']} kg, assets.py says {a['mass']}")
    with open(LOC_FILE, 'w') as f:
        json.dump(dict(note='scene_v0 location IDs (store_db.py, from scene.locations()); world frame = robot start, m; '
                            'u = along the board, centre = (x, y) at the board front; facing = approach heading (deg)',
                       locations={l: dict(o, product=owner.get(l), stations=st.get(l, [])) for l, o in L.items()}),
                  f, indent=1)
    used = len(owner); free = [l for l in L if l not in owner]
    print(f'{len(L)} locations ({used} stocked, {len(free)} empty), {len(P)} products -> {os.path.relpath(LOC_FILE)}')
    for pid, p in P.items():
        reach = sum(L[l]['reachable'] for l in p['locations'])
        print(f"{pid}  {p['name']:28s} {len(p['locations']):2d} locations ({reach} reachable): {short_ranges(p['locations'])}")
    if free:
        print('empty:', short_ranges(free))
    print('database:', 'OK' if not bad else 'PROBLEMS')
    for b in bad:
        print('   ', b)
    sys.exit(1 if bad else 0)
