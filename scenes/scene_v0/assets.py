"""The NVIDIA SimReady assets used by scene_v0 (manifest). fetch_assets.py downloads exactly these.

Source: Hugging Face dataset nvidia/simready-assets (CC-BY-4.0, every package checked 2026-11-16). They are NOT in git:
fetch_assets.py puts them in scenes/assets/simready/ (git-ignored). Attribution: CREDITS.md.

Sizes were measured from each package's geometry (metres, Z up, as our model). Chosen for the arm (user, 2026-11-16):
- shelves: board heights inside the arm's reach for level shelf boards (~0.38..1.27 m);
- table: under the 0.90 m limit for hand-down picks;
- products: narrowest horizontal side 15..80 mm (the jaws open to 90 mm) and a realistic weight inside the arm's
  ~0.25-0.3 kg payload (the wrist pitch servo). Full drink cans / 500 ml bottles (350-520 g) were left out.

The packages' own masses are NOT used for products: they are inconsistent (drink cans 28-89 g = empty, a 64 x 132 x 8
mm chocolate bar 186 g, a full peach can 870 g). `mass` below is our ASSUMED weight for each product (kg); the scene
sets it on the product body. Furniture is fixed in place (its package masses, e.g. a shelf at 3 t, don't matter).
"""

REPO = 'nvidia/simready-assets'

# role, size (x, y, z mm, as measured), notes
FURNITURE = {
    'rack_f04':                  dict(role='shelf', size=(2114, 459, 1950), note='blue rack; boards ~0.76 / 1.21 m (also 1.66, out of reach)'),
    'storage_shelf_basic_b01':   dict(role='shelf', size=(512, 1512, 2030), note='grey 6-board shelf; board heights to be read in Isaac'),
    'table_metal_stainless_a03': dict(role='table', size=(1800, 699, 850), note='0.85 m top, lower shelf'),
    'storage_basket_plastic_a01': dict(role='basket', size=(254, 331, 120), note='crate on the table (customer basket / order bin)'),
}

# product: assumed mass (kg), measured size (x, y, z mm), shape
PRODUCTS = {
    'food_processed_boxedFood_b01':   dict(mass=0.15, size=(37, 129, 130), shape='box'),
    'food_processed_boxedFood_c01':   dict(mass=0.20, size=(35, 135, 190), shape='box'),
    'food_processed_boxedFood_d01':   dict(mass=0.20, size=(48, 104, 210), shape='box'),
    'food_box_cracker_a01':           dict(mass=0.25, size=(164, 72, 213), shape='box'),
    'food_processed_boxedCereal_l01': dict(mass=0.25, size=(60, 140, 200), shape='box'),
    'food_processed_yogurt_a01':      dict(mass=0.13, size=(63, 63, 51), shape='round'),
    'food_processed_yogurt_c01':      dict(mass=0.15, size=(70, 70, 110), shape='round'),
    'food_processed_cannedFood_h01':  dict(mass=0.16, size=(54, 94, 51), shape='round'),     # flat (tuna-size) can
    'food_beverage_coffeeCup_b01':    dict(mass=0.03, size=(67, 67, 102), shape='round'),    # empty cup (stock)
    'food_processed_milk_b01':        dict(mass=0.25, size=(68, 66, 152), shape='box'),
    'food_processed_milk_c01':        dict(mass=0.25, size=(60, 60, 123), shape='round'),
    'food_processed_ketchup_c01':     dict(mass=0.25, size=(55, 55, 191), shape='bottle'),
    'food_processed_ketchup_i01':     dict(mass=0.22, size=(55, 55, 175), shape='bottle'),
    'food_condiment_ketchup_a02':     dict(mass=0.20, size=(45, 74, 145), shape='bottle'),
    'food_processed_chocolate_e01':   dict(mass=0.10, size=(15, 60, 165), shape='flat', lay=True),   # placed lying flat
}
# lay=True: the scene turns the product 90 deg about its y axis (the package stands the bar on its 15 mm edge, where it
# falls over: Isaac 2026-11-16). size() gives the size as placed.


def size(pkg):
    """Product size (x, y, z mm) as the scene places it."""
    sx, sy, sz = PRODUCTS[pkg]['size']
    return (sz, sy, sx) if PRODUCTS[pkg].get('lay') else (sx, sy, sz)

ALL = list(FURNITURE) + list(PRODUCTS)


def usd_path(root, pkg):
    """The package's top-level USD (the one to reference into a scene): from its .metadata/...root_usds.json,
    else the sm_*.usd / sm_*.usda next to the payloads (older packages ship .usd, newer ones .usda)."""
    import glob, json, os
    j = os.path.join(root, pkg, '.metadata', 'com.nvidia.simready.root_usds.json')
    if os.path.exists(j):
        for e in json.load(open(j)).get('entries', []):
            f = os.path.join(root, pkg, e)
            if os.path.exists(f):
                return f
    hits = sorted(glob.glob(os.path.join(root, pkg, 'simready_usd', 'sm_*.usd')) +
                  glob.glob(os.path.join(root, pkg, 'simready_usd', 'sm_*.usda')))
    return hits[0] if hits else None
