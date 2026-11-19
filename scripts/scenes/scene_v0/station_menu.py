"""Drive the mono robot around scene_v0 from a small menu inside the Isaac Sim window (no terminal needed).

    OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/scenes/scene_v0/station_menu.py
    ... --physics_hz 120          # about 2x faster, fine for driving (240 = grasp fidelity, the default)

The "Robot stations" panel:
    Product   the product database (products.json), grouped by store section, or "any product"; under it: size,
              weight, friction (from the asset's physics material) and "rigid" (no softness in the sim)
    Location  "nearest" (the product's nearest reachable spot) or unit / board / slot (e.g. S1 grey shelf 1 / B2 / 05
              = S1-B2-05); each list offers only what exists, and only the chosen product's locations. Out-of-reach
              decor slots are marked and have no station.
    Go to     the robot station that serves the choice (locations.json -> stations.json) and the locations it serves
    Go        drive there on the fixed routes (stations.py: back out to the turning point, across, straight in)
    Home      drive back to the start
    Stop      stop at the next waypoint (a Go from there goes straight on, or via the hub if the line isn't clear)
    Random    drive to a random other station
    Up / Down arrow keys   move the lift while held, at its real speed (0.10 m/s), within 0 .. 0.81 m; also while
              driving. Click into the viewport first if a dropdown still has the keyboard.
    Left / Right arrow keys  tilt the scene camera up / down while held (its pitch servo, -10 .. 80 deg).
    View      Free / Scene cam / Wrist cam: what the viewport looks through (the robot's cameras at their real specs,
              scripts/common/cameras.py; the wrist lens is ASSUMED until calibrated). The same cameras are in the
              viewport's own camera menu (top left).
The arm stays folded; at a station the base is held in place. Close the window to exit.
"""
import argparse, glob, json, math, os, random, re, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap = argparse.ArgumentParser()
ap.add_argument('--physics_hz', type=int, default=240, help='240 = grasp fidelity (default); 120 = about 2x faster, fine for driving')
ap.add_argument('--render_every', type=int, default=1, help='draw every Nth frame only (3 = 20 fps on screen, faster sim)')
ap.add_argument('--wheels', action='store_true', help='full mecanum robot (roller simulation) instead of the planar arm-training base')
ap.add_argument('--fill', type=float, default=1.0)
ap.add_argument('--seed', type=int, default=0, help="the scene's filler layout (as check_scene.py --seed)")
args = ap.parse_args()

import stations as ST
import scene as SC
import assets as SA
from store_db import short_ranges
st = ST.load()
LOC = json.load(open(os.path.join(RP.scene_dir('scene_v0'), 'locations.json')))['locations']
PROD = SC.PRODUCTS                                   # {P01: {name, package, locations, ...}}
ANY = 'any product'
NEAREST = 'nearest reachable'
SECTION_LABEL = {'breakfast and dry goods': 'Dry goods', 'snacks and canned': 'Snacks & canned', 'dairy': 'Dairy', 'sauces': 'Sauces'}


def section_of(pid):
    secs = [LOC[l]['section'] for l in PROD[pid]['locations'] if LOC[l]['section'] in SECTION_LABEL]
    return max(set(secs), key=secs.count) if secs else ''


def friction_of(pkg):
    """static friction range authored in the asset's physics materials (text search of its .usda files)"""
    vals = []
    for f in glob.glob(os.path.join(RP.SIMREADY, pkg, '**', '*.usda'), recursive=True):
        try:
            vals += [float(v) for v in re.findall(r'staticFriction = ([0-9.]+)', open(f, errors='ignore').read())]
        except OSError:
            pass
    if not vals:
        return 'friction default 0.5'
    a, b = round(min(vals), 2), round(max(vals), 2)
    return f'friction {a:g}' if a == b else f'friction {a:g}-{b:g}'


def info_of(pid):
    p = PROD[pid]
    return (f"{' x '.join(str(int(v)) for v in p['size_mm'])} mm · {p['mass_kg']:g} kg · {friction_of(p['package'])} · rigid")


def station_of(loc_id):
    s = LOC.get(loc_id, {}).get('stations') or []
    return s[0] if s else None


def choice_station(pid, loc):
    """(station or None, text for the panel) for the chosen product id (or None) and location (or NEAREST)."""
    if loc != NEAREST:
        s = station_of(loc)
        o = LOC[loc]
        what = f"{loc}: {PROD[o['product']]['name'] if o.get('product') else 'empty'}"
        return s, (f'{s}   ({what})' if s else f'{what}: out of reach (decor), no station')
    if pid is None:
        return None, 'pick a product or a location'
    cands = [l for l in PROD[pid]['locations'] if station_of(l)]
    if not cands:
        return None, f"{PROD[pid]['name']}: no reachable location"
    task_st = [l for l in cands if st[station_of(l)].get('task')]
    l = sorted(task_st or cands)[0]
    return station_of(l), f"{station_of(l)}   ({PROD[pid]['name']} at {l})"


import scene_robot as R
app = R.start_app(False)
import numpy as np
import omni.ui as ui
task, items, watch = R.load(app, False, fill=args.fill, seed=args.seed, wheels=args.wheels,
                            physics_hz=args.physics_hz, render_every=args.render_every)
import cameras as CAM
from omni.kit.viewport.utility import get_active_viewport
cams = task.cams                                     # added by proto_task.Task.start()
_vp = get_active_viewport()
VIEWS = {'Free': _vp.camera_path.pathString if hasattr(_vp.camera_path, 'pathString') else str(_vp.camera_path),
         **{('Scene cam' if n == 'scene' else 'Wrist cam'): c.prim_path for n, c in cams.items()}}


def set_view(name):
    _vp.camera_path = VIEWS[name]


# ---------------------------------------------------------------- lift on the Up / Down keys
import carb.input, omni.appwindow
import scara_ik as K
import proto_task as PT
LIFT_RANGE = (0.0, K.LIFT_MAX['prototype_mono_arm'])
LIFT_V = 0.10 * 0.9                                    # m/s: the URDF lift speed with drives.rate_limit's margin
keys = {'up': False, 'down': False, 'left': False, 'right': False}
TILT_V = math.radians(30)                              # rad/s, scene camera pitch while Left / Right is held
_pi = task.idx['scene_camera_pitch']
TILT_RANGE = (math.radians(-10), math.radians(80))
_KI = carb.input.KeyboardInput
_held = (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT)


def on_key(e, *_):
    k = {_KI.UP: 'up', _KI.DOWN: 'down', _KI.LEFT: 'left', _KI.RIGHT: 'right'}.get(e.input)
    if k:
        keys[k] = e.type in _held
    return True


_input = carb.input.acquire_input_interface()
_key_sub = _input.subscribe_to_keyboard_events(omni.appwindow.get_default_app_window().get_keyboard(), on_key)
_step = task.step_once
_li = task.idx['lift']


def step_with_lift():
    t = keys['right'] - keys['left']                  # right = tilt down (+ pitch)
    if t:
        task.q_goal[_pi] = min(TILT_RANGE[1], max(TILT_RANGE[0], task.q_goal[_pi] + t * TILT_V * PT.DT))
    d = keys['up'] - keys['down']
    if d:
        task.q_goal[_li] = min(LIFT_RANGE[1], max(LIFT_RANGE[0], task.q_goal[_li] + d * LIFT_V * PT.DT))
    _step()
task.step_once = step_with_lift

# ---------------------------------------------------------------- the panel
state = dict(req=None, stop=False, pid=None, loc=NEAREST, station=None, busy=False)
_order = list(SECTION_LABEL)
pids = sorted(PROD, key=lambda p: (_order.index(section_of(p)) if section_of(p) in _order else 99, p))
prod_items = [ANY] + [f"{SECTION_LABEL.get(section_of(p), '?')} · {PROD[p]['name']} ({p})" for p in pids]
INFO = {p: info_of(p) for p in pids}
sel = dict(unit=None, level=None)                    # the location picked so far (unit, board); slot -> state['loc']

win = ui.Window('Robot stations', width=600, height=345, position_x=40, position_y=90)
with win.frame:
    with ui.VStack(spacing=6, height=0):
        with ui.HStack(height=24):
            ui.Label('Product', width=70)
            prod_combo = ui.ComboBox(0, *prod_items)
        with ui.HStack(height=18):
            ui.Spacer(width=70)
            info_label = ui.Label('', style={'color': 0xFF9A9A9A})
        with ui.HStack(height=24, spacing=4):
            ui.Label('Location', width=70)
            unit_frame = ui.Frame(width=170)
            ui.Label('Board', width=40)
            board_frame = ui.Frame(width=100)
            ui.Label('Slot', width=30)
            slot_frame = ui.Frame()
        with ui.HStack(height=36):
            ui.Label('Go to', width=70)
            st_label = ui.Label('pick a product or a location', word_wrap=True)
        with ui.HStack(height=30, spacing=6):
            go_btn = ui.Button('Go')
            ui.Button('Home', clicked_fn=lambda: state.update(req='home', stop=False))
            ui.Button('Stop', clicked_fn=lambda: state.update(stop=True))
            ui.Button('Random', clicked_fn=lambda: state.update(req='__random__', stop=False))
        with ui.HStack(height=22):
            ui.Label('Lift', width=70)
            lift_label = ui.Label('')
        with ui.HStack(height=26, spacing=6):
            ui.Label('View', width=70)
            for _v in VIEWS:
                ui.Button(_v, clicked_fn=lambda v=_v: set_view(v))
        status = ui.Label('at home', word_wrap=True)


def refresh_station():
    s, text = choice_station(state['pid'], state['loc'])
    state['station'] = s
    if s and st[s].get('locations'):
        text += f"\nserves {short_ranges(st[s]['locations'])}"
    st_label.text = text
    go_btn.enabled = s is not None


def cands():
    """location IDs on offer: the chosen product's, or all"""
    return sorted(PROD[state['pid']]['locations']) if state['pid'] else sorted(LOC)


def combo(items, on_pick):
    """a ComboBox for items [(label, value)]; picks the first value at once"""
    c = ui.ComboBox(0, *[t for t, _ in items])
    c.model.add_item_changed_fn(lambda m, _i: on_pick(items[m.get_item_value_model().as_int][1]))
    on_pick(items[0][1])


def build_unit():
    units = sorted({LOC[l]['unit'] for l in cands()})
    combo(([('nearest', None)] if state['pid'] else [('-', None)]) + [(f"{u}  {SC.UNIT_NAMES[u]}", u) for u in units], pick_unit)


def pick_unit(u):
    sel['unit'] = u
    board_frame.rebuild()


def build_board():
    u = sel['unit']
    if u is None:
        combo([('-', None)], pick_board); return
    lv = sorted({LOC[l]['level'] for l in cands() if LOC[l]['unit'] == u})
    z = {LOC[l]['level']: LOC[l]['z'] for l in LOC if LOC[l]['unit'] == u}
    combo([(f"B{v}  {z[v]:.2f} m", v) for v in lv], pick_board)


def pick_board(v):
    sel['level'] = v
    slot_frame.rebuild()


def build_slot():
    u, v = sel['unit'], sel['level']
    if u is None:
        combo([('-', NEAREST)], pick_slot); return
    ids = [l for l in cands() if LOC[l]['unit'] == u and LOC[l]['level'] == v]

    def lab(l):
        o = LOC[l]; name = PROD[o['product']]['name'] if o.get('product') else 'empty'
        return f"{o['slot']:02d}  {'' if state['pid'] else name}{'' if station_of(l) else '  (decor)'}".rstrip()
    combo([(lab(l), l) for l in ids], pick_slot)


def pick_slot(l):
    state['loc'] = l
    refresh_station()


def on_prod(model, _item):
    k = model.get_item_value_model().as_int
    state['pid'] = None if k == 0 else pids[k - 1]
    info_label.text = INFO[state['pid']] if state['pid'] else ''
    unit_frame.rebuild()


unit_frame.set_build_fn(build_unit)
board_frame.set_build_fn(build_board)
slot_frame.set_build_fn(build_slot)
prod_combo.model.add_item_changed_fn(on_prod)
go_btn.set_clicked_fn(lambda: state['station'] and state.update(req=state['station'], stop=False))
refresh_station()

# ---------------------------------------------------------------- driving
here_at = 'home'


def drive(target):
    """Drive from here_at to target waypoint by waypoint; Stop ends it at the next waypoint."""
    global here_at
    src = here_at
    sts = st
    if src not in st:                                 # stopped between stations: start from where the base is
        p, yaw = task.base_pose()
        sts = dict(st, _here={'approach': [float(p[0]), float(p[1])], 'yaw': math.degrees(yaw)})
        src = '_here'
    pts = ST.route(sts, src, target)
    if src == '_here':
        pts = pts[1:]                                 # it is already at its own "turning point"
    t0 = task.t
    print(f'\n[{task.t:7.1f} s] {here_at} -> {target}', flush=True)
    for k, (x, y, yaw) in enumerate(pts):
        if state['stop']:
            state['stop'] = False
            here_at = 'between stations'
            status.text = f'stopped on the way to {target}'
            return
        status.text = f'driving to {target}: waypoint {k + 1}/{len(pts)}'
        lbl = target if k == len(pts) - 1 else '  via waypoint'
        task.drive_to(np.array([x, y, math.radians(yaw)]), lbl)
    here_at = target
    s = st[target]
    p, yaw_now = task.base_pose()
    err = math.hypot(s['x'] - p[0], s['y'] - p[1]) * 1000
    status.text = f"at {target} · {task.t - t0:.1f} s · {err:.0f} mm from the mark"


print('station_menu: use the "Robot stations" panel in the Isaac window; close the window to exit.', flush=True)
try:
    while app.is_running():
        req = state['req']
        if req == '__random__':
            req = random.choice([n for n in st if n != here_at and 'x' in st[n]])
        if req and req != here_at:
            state['req'] = None
            drive(req)
        else:
            state['req'] = None
            state['stop'] = False
            task.step_once()                          # parked: physics keeps running, the base is held
        q = task.robot.get_joint_positions()[_li]
        txt = f"{q:.3f} m  (target {task.q_goal[_li]:.3f}, range {LIFT_RANGE[0]:g} .. {LIFT_RANGE[1]:g})   Up / Down keys;   camera tilt {math.degrees(task.q_goal[_pi]):.0f} deg (Left / Right)"
        if lift_label.text != txt:
            lift_label.text = txt
except SystemExit:                                    # window closed (proto_task.tick raises it)
    pass
app.close()
