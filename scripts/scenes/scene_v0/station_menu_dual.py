"""Drive the DUAL-arm robot around scene_v0 from a small menu inside the Isaac Sim window: the dual version of
station_menu_mono.py, with the pair stations of stations_dual.py (two products, one per arm) and both arms' work.

    OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/scenes/scene_v0/station_menu_dual.py
    ... --physics_hz 120          # about 2x faster, fine for driving (240 = grasp fidelity, the default)

The "Dual robot stations" panel:
    Pair      a pair station: the left arm's product and the right arm's product, side by side on one board
    Go        drive there on the fixed routes (back out to the turning point, across, straight in), both arms folded
    Home      drive back to the start
    Stop      stop at the next waypoint
    Random    drive to a random other pair station
    Pick both / Place both back   both arms at once (pick_place_dual.py: into the two tote spots and back)
    Up / Down arrow keys   move the lift while held (0.10 m/s, 0 .. 0.871 m) while the arms are folded
    Views     the scene camera and both wrist cameras in their own viewport windows (real lens specs, cameras.py)
Close the window to exit.
"""
import argparse, math, os, random, sys, time
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap = argparse.ArgumentParser()
ap.add_argument('--physics_hz', type=int, default=240)
ap.add_argument('--render_every', type=int, default=1)
args = ap.parse_args()

import stations_mono as SM
import stations_dual as SD
st_mono, st = SM.load(), SD.load()
pairs = [n for n in st if n != 'home']
import scene_robot as R
R.ST = SD
app = R.start_app(False)
import numpy as np
import carb.input, omni.appwindow, omni.ui as ui
from omni.kit.viewport.utility import create_viewport_window
import cameras as CAM
import scara_ik as K
import proto_task as PT
task, items, watch = R.load(app, False, layout='prototype', physics_hz=args.physics_hz, render_every=args.render_every)
import pick_place_dual as PD
PD.init(task, items, watch, st_mono, st)

# ---------------------------------------------------------------- camera views
x = 20
for title, cam in (('Scene camera', 'scene'), ('Left wrist camera', 'left_wrist'), ('Right wrist camera', 'right_wrist')):
    if cam in task.cams:
        r = CAM.SCENE['res'] if cam == 'scene' else CAM.WRIST['res']
        v = create_viewport_window(title, width=r[0] + 20, height=r[1] + 45, position_x=x, position_y=820,
                                   camera_path=task.cams[cam].prim_path)
        try:
            v.viewport_api.resolution = r
        except Exception:
            pass
        x += r[0] + 30

# ---------------------------------------------------------------- lift on the arrow keys (arms folded only)
LIFT_RANGE = (0.0, K.LIFT_MAX['prototype'])
LIFT_V = 0.10 * 0.9
keys = {'up': False, 'down': False}
_KI = carb.input.KeyboardInput
_held = (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT)


def on_key(e, *_):
    k = {_KI.UP: 'up', _KI.DOWN: 'down'}.get(e.input)
    if k:
        keys[k] = e.type in _held
    return True


_key_sub = carb.input.acquire_input_interface().subscribe_to_keyboard_events(
    omni.appwindow.get_default_app_window().get_keyboard(), on_key)
_li = task.idx['lift']
_step = task.step_once
state = dict(req=None, stop=False, sel=0, busy=False)


def step_with_lift():
    d = keys['up'] - keys['down']
    if d and not state['busy']:
        task.q_goal[_li] = min(LIFT_RANGE[1], max(LIFT_RANGE[0], task.q_goal[_li] + d * LIFT_V * PT.DT))
    _step()


task.step_once = step_with_lift


def label(p):
    s = st[p]
    name = lambda k: PD.PM.NAMES.get(s[k]['product'], s[k]['product'])
    return f"{s['board']}:  L {name('left')}  |  R {name('right')}"


# ---------------------------------------------------------------- the panel
win = ui.Window('Dual robot stations', width=720, height=230, position_x=40, position_y=90)
with win.frame:
    with ui.VStack(spacing=6, height=0):
        with ui.HStack(height=24):
            ui.Label('Pair', width=50)
            combo = ui.ComboBox(0, *[label(p) for p in pairs])
        with ui.HStack(height=30, spacing=6):
            ui.Button('Go', clicked_fn=lambda: state.update(req=pairs[state['sel']], stop=False))
            ui.Button('Home', clicked_fn=lambda: state.update(req='home', stop=False))
            ui.Button('Stop', clicked_fn=lambda: state.update(stop=True))
            ui.Button('Random', clicked_fn=lambda: state.update(req='__random__', stop=False))
        with ui.HStack(height=30, spacing=6):
            ui.Button('Pick both into the tote', clicked_fn=lambda: state.update(req='__pick__'))
            ui.Button('Place both back', clicked_fn=lambda: state.update(req='__place__'))
            ui.Button('Put products back', clicked_fn=lambda: state.update(req='__restore__'))
            ui.Button('Reset robot', clicked_fn=lambda: state.update(req='__reset__'))
        status = ui.Label('choose a pair and press Go (or "Pick both into the tote": it drives there first)', word_wrap=True)
        lift_lbl = ui.Label('')
combo.model.get_item_value_model().add_value_changed_fn(lambda m: state.update(sel=m.as_int))


def drive(target):
    src = PD.ROBOT['at']
    if src == target:
        status.text = f'already at {target}'; return
    pts = SM.route(st, src, target) if src in st else [(st[target]['approach'][0], st[target]['approach'][1], st[target]['yaw']),
                                                        (st[target]['x'], st[target]['y'], st[target]['yaw'])]
    t0 = task.t
    for k, (x_, y_, yaw) in enumerate(pts):
        if state['stop']:
            state['stop'] = False; PD.ROBOT['at'] = '__between__'
            status.text = f'stopped on the way to {target}'; return
        status.text = f'driving to {target} ({k + 1}/{len(pts)})'
        task.drive_to(np.array([x_, y_, math.radians(yaw)]), target if k == len(pts) - 1 else '  via')
    PD.ROBOT['at'] = target
    status.text = f'at {target} ({task.t - t0:.1f} s)'


print('station_menu_dual: use the "Dual robot stations" panel; close the window to exit.', flush=True)
_quit_at = time.time() + float(os.environ['MENU_AUTOQUIT']) if os.environ.get('MENU_AUTOQUIT') else None
if os.environ.get('MENU_CLICK'):
    state['req'] = os.environ['MENU_CLICK'] if os.environ['MENU_CLICK'] in st else '__' + os.environ['MENU_CLICK'] + '__'
try:
    while app.is_running():
        if _quit_at and time.time() > _quit_at:
            import omni.kit.app; omni.kit.app.get_app().post_quit(); _quit_at = None
        req, state['req'] = state['req'], None
        if req:
            state['busy'] = True
            if req == '__random__':
                req = random.choice([p for p in pairs if p != PD.ROBOT['at']])
            if req == '__pick__':
                status.text = 'picking both ...'
                ok, txt = PD.run(pairs[state['sel']]); status.text = ('both in the tote ' if ok else 'FAILED: ') + txt
            elif req == '__place__':
                status.text = 'placing both back ...'
                ok, txt = PD.run_place(); status.text = ('both back on their spots ' if ok else 'FAILED: ') + txt
            elif req == '__restore__':
                status.text = f'{PD.PM.restore()} product(s) put back'
            elif req == '__reset__':
                PD.reset_robot(); status.text = 'robot back home'
            else:
                drive(req)
            print('panel status:', status.text, flush=True)
            state['busy'] = False
        else:
            task.step_once()
        lift_lbl.text = f'lift {task.robot.get_joint_positions()[_li] * 100:.0f} cm'
except SystemExit:
    pass
PD.PM.close()
app.close()
