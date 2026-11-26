"""DUAL robot: both arms pick two products at the same time into the tote (scene_v0). The dual version of
pick_to_tote_mono.py; the pair stations come from stations_dual.py (two task products on one board, one per arm).

    OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/scenes/scene_v0/pick_to_tote_dual.py
    ... --pair pair_rack_b0_box_cracker_a01__boxedFood_b01     which pair (default: the first)
    ... --headless [--all] [--place]                             no window; --all tries every pair, --place puts both
                                                                 products back after each pick (both arms at once)

In the window: a "Dual pick" panel (choose the pair, "Pick both into the tote", "Place both back", "Pick + place back",
"Put products back" (teleport), "Reset robot") and the scene camera + both wrist cameras.
"""
import argparse, os, sys, time
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap = argparse.ArgumentParser()
ap.add_argument('--pair', default=None)
ap.add_argument('--headless', action='store_true')
ap.add_argument('--all', action='store_true')
ap.add_argument('--place', action='store_true', help='headless: after each pair pick, put both products back on the shelf')
args = ap.parse_args()

import stations_mono as SM
import stations_dual as SD
st_mono, st = SM.load(), SD.load()
pairs = [n for n in st if n != 'home']
pid = args.pair or pairs[0]
if pid not in st:
    sys.exit(f'unknown pair {pid}; pairs: {pairs}')
import scene_robot as R
R.ST = SD                                  # legs / home from the dual stations
app = R.start_app(args.headless)
import cameras as CAM
task, items, watch = R.load(app, args.headless, layout='prototype')
import pick_place_dual as PD
PD.init(task, items, watch, st_mono, st)

if args.headless:
    todo = pairs if args.all else [pid]
    res = []
    for p in todo:
        print(f'\n================ {p}', flush=True)
        ok, txt = PD.run(p)
        if ok and args.place:
            ok, t2 = PD.run_place(); txt = f'pick {txt}, place {t2}'
        res.append((p, ok, txt))
    print('\nRESULTS')
    for p, ok, txt in res:
        print(f'  {"PASS" if ok else "FAIL"}  {p:60s} {txt}')
    task.summary()
    PM = PD.PM; PM.close()
    app.close(); sys.exit(0)

import omni.ui as ui
from omni.kit.viewport.utility import create_viewport_window
CAM_WINS = {}
x = 20
for title, cam in (('Scene camera', 'scene'), ('Left wrist camera', 'left_wrist'), ('Right wrist camera', 'right_wrist')):
    if cam in task.cams:
        r = CAM.SCENE['res'] if cam == 'scene' else CAM.WRIST['res']
        CAM_WINS[cam] = create_viewport_window(title, width=r[0] + 20, height=r[1] + 45, position_x=x, position_y=820,
                                               camera_path=task.cams[cam].prim_path)
        x += r[0] + 30
        try:
            CAM_WINS[cam].viewport_api.resolution = r
        except Exception:
            pass
DEPTH_VIEW = CAM.depth_window(task.cams['scene']) if 'scene' in task.cams else None   # scene camera depth (cameras.py)
state = {'req': None, 'sel': pairs.index(pid)}
win = ui.Window('Dual pick', width=640, height=170, position_x=40, position_y=90)
with win.frame:
    with ui.VStack(spacing=6, height=0):
        combo = ui.ComboBox(state['sel'], *pairs)
        with ui.HStack(height=30, spacing=6):
            ui.Button('Pick both into the tote', clicked_fn=lambda: state.update(req='pick'))
            ui.Button('Place both back', clicked_fn=lambda: state.update(req='place'))
            ui.Button('Pick + place back', clicked_fn=lambda: state.update(req='both'))
            ui.Button('Put products back', clicked_fn=lambda: state.update(req='restore'))
            ui.Button('Reset robot', clicked_fn=lambda: state.update(req='reset'))
        status = ui.Label('choose a pair and press "Pick both into the tote"', word_wrap=True)
combo.model.get_item_value_model().add_value_changed_fn(lambda m: state.update(sel=m.as_int))
# the 4 camera views in a row along the bottom, the panel as a tab next to Render Settings (cameras.arrange)
LAYOUT = CAM.arrange([CAM_WINS.get('scene'), DEPTH_VIEW[0] if DEPTH_VIEW else None, CAM_WINS.get('left_wrist'),
                      CAM_WINS.get('right_wrist')], panel=win)
_quit_at = time.time() + float(os.environ['P2T_AUTOQUIT']) if os.environ.get('P2T_AUTOQUIT') else None
if os.environ.get('P2T_CLICK'):
    state['req'] = os.environ['P2T_CLICK']
try:
    while app.is_running():
        if _quit_at and time.time() > _quit_at:
            import omni.kit.app; omni.kit.app.get_app().post_quit(); _quit_at = None
        req, state['req'] = state['req'], None
        if req in ('pick', 'both'):
            status.text = 'picking both ...'
            ok, txt = PD.run(pairs[state['sel']]); status.text = ('both in the tote ' if ok else 'FAILED: ') + txt
            print('panel status:', status.text, flush=True)
            req = 'place' if (ok and req == 'both') else None
        if req == 'place':
            status.text = 'placing both back ...'
            ok, txt = PD.run_place(); status.text = ('both back on their spots ' if ok else 'FAILED: ') + txt
            print('panel status:', status.text, flush=True)
        elif req in ('pick', 'both', 'place'):
            pass
        elif req == 'restore':
            status.text = f'{PD.PM.restore()} product(s) put back'
        elif req == 'reset':
            PD.reset_robot(); status.text = 'robot back home'
        else:
            task.step_once()
except SystemExit:
    pass
PD.PM.close()
app.close()
