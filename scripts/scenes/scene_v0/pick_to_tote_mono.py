"""One pick from a store shelf / table into the robot's tote, with the scene and wrist camera views side by side.

    OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/scenes/scene_v0/pick_to_tote_mono.py
    ... --station shelf_b1_milk_b01   any station of stations.json (default: the milk carton on the grey shelf)
    ... --tilt 40                      scene camera pitch in deg (+ = down; its servo allows -10 .. 80)
    ... --scene_cam gemini2            scene camera model: gemini2 (Orbbec Gemini 2, 86 deg, the robot's), pro_w (OAK-D Pro W, 127 deg) or pro_af (OAK-D Pro AF, 66 deg)
    ... --headless --out DIR           no window: saves the two camera views side by side at each stage instead
    ... --headless --all | --stations a,b   try several stations in one session (products put back between), PASS / FAIL
    ... --all_stations                 window: list every station, not only the 16 that pass
    ... --random 10                    window: 10 random shelf / rack products, each picked into the tote and put back
    ... --random -1                    the same, forever (panel 'Stop random' or close the window to end)

In the window, a "Pick to tote" panel: choose a product, "Pick into the tote" (drives there from wherever the robot is,
picks, places, folds); before each pick every product that moved goes back to its start spot. "Put products back",
"Reset robot" (teleports it home after an aborted pick).

The mono robot starts at home, drives to the station, then: open, straight in (side grasp, hand level) or down (top
grasp, short products), close, lift, out, swing to the tote with the hand down (a side-gripped product ends up lying),
release, back to the travel fold. Plans: grasp_plan_mono.py (the same side / top grasps as the stations and check_scene_mono.py),
the tote spot of mono_poses.py. Reports where the product ended and anything else that moved more than 10 mm.
In the window: two extra viewport windows show the scene camera and the wrist camera (live, through their real lens
specs); the main viewport stays free. Close the window to exit.
"""
import argparse, math, os, sys, time
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap = argparse.ArgumentParser()
ap.add_argument('--station', default='shelf_b1_milk_b01')
ap.add_argument('--tilt', type=float, default=40.0, help='scene camera pitch, deg (+ = down)')
ap.add_argument('--scene_cam', default='gemini2', choices=('gemini2', 'pro_w', 'pro_af'))
ap.add_argument('--headless', action='store_true')
ap.add_argument('--out', default='pick_to_tote')
ap.add_argument('--all', action='store_true', help='headless: try every station in one session, print PASS / FAIL')
ap.add_argument('--stations', default='', help='headless: comma-separated stations to try in one session')
ap.add_argument('--place', action='store_true', help='headless: after each pick, put the product back from the tote')
ap.add_argument('--random', type=int, default=0, help='window: start with N random shelf / rack picks, each put back (-1 = forever; the panel buttons run more)')
ap.add_argument('--all_stations', action='store_true', help='window: offer every station, not only the pickable ones')
args = ap.parse_args()
os.environ['MM_SCENE_CAM'] = args.scene_cam          # read by cameras.py when the robot's cameras are made

import stations_mono as ST
import scene_robot as R
st = ST.load()
if args.station not in st:
    sys.exit(f'unknown station {args.station}; see stations.json')
app = R.start_app(args.headless)
import numpy as np
import cameras as CAM
task, items, watch = R.load(app, args.headless)
cams = task.cams
import pick_place_mono as PP                      # the pick / place itself (lib/pick_place_mono.py, shared with train_pipeline/record)
PP.init(task, items, watch, st, tilt=args.tilt, headless=args.headless, out=args.out if args.headless else None)

# ---------------------------------------------------------------- the two camera views
if not args.headless:
    from omni.kit.viewport.utility import create_viewport_window
    sres, wres = CAM.SCENE['res'], CAM.WRIST['res']
    views = [create_viewport_window('Scene camera', width=sres[0] + 20, height=sres[1] + 45, position_x=20, position_y=820,
                                    camera_path=cams['scene'].prim_path),
             create_viewport_window('Wrist camera', width=wres[0] + 20, height=wres[1] + 45, position_x=sres[0] + 50,
                                    position_y=820, camera_path=cams['left_wrist'].prim_path)]
    for v, r in zip(views, (sres, wres)):
        try:
            v.viewport_api.resolution = r
        except Exception:
            pass

picks = [n for n in st if n != 'home' and 'product' in st[n]]
if args.headless:
    todo = sorted(PP.PICKABLE - PP.PICK_ONLY) if (args.all and args.place) else picks if args.all else (args.stations.split(',') if args.stations else [args.station])
    summary_ = []
    for sid in todo:
        print(f'\n================ {sid}', flush=True)
        ok, txt = PP.run(sid)
        if ok and args.place:
            ok, t2 = PP.run_place(); txt = 'pick OK, place ' + t2
        summary_.append((sid, ok, txt))
        print(f'---------------- {sid}: {txt}', flush=True)
    print('\nRESULTS')
    for sid, ok, txt in summary_:
        print(f'  {"PASS" if ok else "FAIL"}  {sid:28s} {txt}')
    PP.close()
    task.summary()
    for p in PP.shots:
        print('  picture:', p)
    app.close()
    sys.exit(0)

# ---------------------------------------------------------------- the panel in the Isaac window
import omni.ui as ui
choices = picks if (args.all_stations or PP.PICKABLE is None) else [n for n in picks if n in PP.PICKABLE]
choices.sort(key=PP.label)
state = {'req': None, 'busy': False, 'sel': choices.index(args.station) if args.station in choices else 0, 'random': args.random}
import random as _rnd
shelf_choices = [n for n in choices if (n.startswith('shelf') or n.startswith('rack')) and n not in PP.PICK_ONLY]
win = ui.Window('Pick to tote', width=560, height=262, position_x=40, position_y=90)
with win.frame:
    with ui.VStack(spacing=6, height=0):
        with ui.HStack(height=24):
            ui.Label('Product', width=70)
            combo = ui.ComboBox(state['sel'], *[PP.label(n) for n in choices])
        with ui.HStack(height=30, spacing=6):
            ui.Button('Pick into the tote', clicked_fn=lambda: state.update(req='pick') if not state['busy'] else None)
            ui.Button('Place back', clicked_fn=lambda: state.update(req='place') if not state['busy'] else None)
            ui.Button('Pick + place back', clicked_fn=lambda: state.update(req='both') if not state['busy'] else None)
        with ui.HStack(height=26, spacing=6):
            ui.Button('Random shelf picks (10)', clicked_fn=lambda: state.update(random=10))
            ui.Button('Random forever', clicked_fn=lambda: state.update(random=-1))
            ui.Button('Stop random', clicked_fn=lambda: state.update(random=0))
        with ui.HStack(height=26, spacing=6):
            ui.Button('Put products back', clicked_fn=lambda: state.update(req='restore') if not state['busy'] else None)
            ui.Button('Reset robot', clicked_fn=lambda: state.update(req='reset') if not state['busy'] else None)
        status = ui.Label('choose a product and press "Pick into the tote"', word_wrap=True)
combo.model.get_item_value_model().add_value_changed_fn(lambda m: state.update(sel=m.as_int))

print('pick_to_tote: use the "Pick to tote" panel in the Isaac window; close the window to exit.', flush=True)
# tests without a person: P2T_CLICK=pick|place|both presses that button once; P2T_AUTOQUIT=S closes the window (like
# its X) S seconds after start, or after the click's job when P2T_CLICK is set
_quit_at = time.time() + float(os.environ['P2T_AUTOQUIT']) if os.environ.get('P2T_AUTOQUIT') else None
if os.environ.get('P2T_CLICK'):
    state['req'] = os.environ['P2T_CLICK']
try:
    while app.is_running():
        if _quit_at and time.time() > _quit_at:
            import omni.kit.app; omni.kit.app.get_app().post_quit(); _quit_at = None
        req, state['req'] = state['req'], None
        if req is None and state['random'] != 0 and not state['busy'] and shelf_choices:      # < 0: forever
            sid_ = _rnd.choice(shelf_choices); state['sel'] = choices.index(sid_); state['random'] -= state['random'] > 0
            combo.model.get_item_value_model().set_value(state['sel'])
            print(f"random pick: {sid_} ({state['random'] if state['random'] >= 0 else 'forever'} more)", flush=True); req = 'both'
        if req == 'both' and choices[state['sel']] in PP.PICK_ONLY:
            status.text = f"{PP.label(choices[state['sel']])}: can't be placed back yet; use \"Pick into the tote\""
            req = None
        if req in ('pick', 'both'):
            sid = choices[state['sel']]
            state['busy'] = True
            status.text = f'picking {PP.label(sid)} ...'
            t0 = task.t
            ok, txt = PP.run(sid)
            status.text = f"{PP.label(sid)}: {'in the tote' if ok else 'FAILED: ' + txt}  ({task.t - t0:.0f} s)" + \
                          ('' if ok else '. "Reset robot" if the arm is stuck.')
            if ok and req == 'both':
                req = 'place'
            state['busy'] = False
            print('panel status:', status.text, flush=True)
        if req == 'place' and PP.CTX and PP.CTX['sid'] in PP.PICK_ONLY:
            status.text = f"{PP.label(PP.CTX['sid'])}: can't be placed back yet (it tips over in the tote)"
            req = None
        if req == 'place':
            state['busy'] = True
            what = PP.label(PP.CTX['sid']) if PP.CTX else ''
            status.text = f'placing {what} back ...'
            t0 = task.t
            ok, txt = PP.run_place()
            status.text = f"{what}: {'back on its spot' if ok else 'FAILED: ' + txt}  ({task.t - t0:.0f} s)" + \
                          ('' if ok or not what else '. "Put products back" / "Reset robot" to start again.')
            state['busy'] = False
            print('panel status:', status.text, flush=True)
            if os.environ.get('P2T_CLICK') and os.environ.get('P2T_AUTOQUIT'):
                _quit_at = time.time() + 2
        elif req in ('pick', 'both', 'place'):
            pass
        elif req == 'restore':
            status.text = f'{PP.restore()} product(s) put back'
        elif req == 'reset':
            PP.reset_robot(); status.text = 'robot back home (teleported)'
        else:
            task.step_once()
except SystemExit:                                    # window closed (proto_task.tick raises it)
    pass
PP.close()
app.close()
