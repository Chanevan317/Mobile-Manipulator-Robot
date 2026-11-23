"""Record demonstrations by teleoperation: the leader arm drives the mono robot's arm in the Isaac store scene.
Step 1 of the pipeline, by a person (record_sim.py is the planner's version; both write the same episode files).

    conda activate env_isaaclab           # run from the repo root
    OMNI_KIT_ACCEPT_EULA=YES python train_pipeline/record/teleop_sim.py --name teleop_v1            # the leader arm
    ... --port /dev/serial/by-id/...    leader port (default LEADER_PORT, else the one in the calibration file)
    ... --calib leader_mono             calibration (train_pipeline/leader/calibration/<name>.json, calibrate.py)
    ... --leader dummy                  no hardware: a recorded episode plays the leader (raw servo ticks through a
                                        made-up calibration), recorded to data/raw/teleop_dummy/
    ... --dummy_episode PATH            which episode (default: the first of the newest data/raw set)
    ... --auto                          dummy only: go, teleop, record, save by itself (also --headless), then exit

The window: a "Teleop recording" panel + the scene and wrist camera views (the policy sees exactly these).
    1. choose the product and the task (pick into the tote / place back on its shelf), "Go to station": the robot drives
       there with its arm folded (navigation is scripted, not learned; pick: the products are put back first).
       (Teleop also works at home without a station, to try the leader; recording needs a station.)
    2. "Start teleop": the arm first moves SLOWLY to the leader's pose, then follows it 1:1 (put the leader in the travel
       fold first: then nothing moves). Up / Down arrow keys move the lift while held
    3. R = record, do the task, fold the arm again, N = save (checked automatically: in the tote / back on its spot,
       nothing else knocked; a failed one is saved marked FAILED, the converter skips it) or X = discard
    4. "Stop teleop" (the arm holds), next product. "Fold arm" = a straight move to the travel fold (not collision-
       checked: bring the hand clear of the furniture first). Driving needs the arm folded.

One episode = the same span as the planner's: parked at the station with the arm folded, to the arm folded again.
Recorded at 30 fps: measured state, the command the drives got (leader pose rate-limited to the real servo speeds),
both camera images, and the leader's own angles ('leader.deg', not used for training).
"""
import argparse, math, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'leader'))
import common as C                                                       # noqa: E402
import leader_arm as LA                                                  # noqa: E402
sys.path.insert(0, str(C.REPO / 'scripts' / 'common')); import repo_paths as RP   # noqa: E402,F401

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--name', default=None, help='recording set: data/raw/<name>/ (default teleop_v1, dummy: teleop_dummy)')
ap.add_argument('--leader', default='arm', choices=('arm', 'dummy'))
ap.add_argument('--port', default=None)
ap.add_argument('--calib', default=LA.DEFAULT_NAME)
ap.add_argument('--dummy_episode', default=None)
ap.add_argument('--auto', action='store_true')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--tilt', type=float, default=40.0, help='scene camera pitch, deg')
ap.add_argument('--jpeg', type=int, default=95)
ap.add_argument('--depth', action='store_true')
args = ap.parse_args()
DUMMY = args.leader == 'dummy'
if (args.auto or args.headless) and not DUMMY:
    ap.error('--auto / --headless only with --leader dummy (a person needs the window)')
args.name = args.name or ('teleop_dummy' if DUMMY else 'teleop_v1')

# the leader first: no point loading the scene when it isn't there
if DUMMY:
    import h5py
    ep = args.dummy_episode
    if not ep:
        sets = sorted((p for p in C.RAW_DIR.iterdir() if p.is_dir() and p.name != args.name and any(p.glob('episode_*.h5'))),
                      key=lambda p: p.stat().st_mtime)
        if not sets:
            sys.exit('--leader dummy needs a recorded episode (record_sim.py) or --dummy_episode')
        ep = sorted(sets[-1].glob('episode_*.h5'))[0]
    with h5py.File(ep, 'r') as f:
        D = dict(attrs=dict(f.attrs), actions=f['action'][:])
    if list(D['attrs']['joint_names']) != C.JOINT_NAMES:
        sys.exit(f'{ep}: different joint names')
    print(f'dummy leader: replays {ep} ({len(D["actions"]) / C.FPS:.0f} s, "{D["attrs"]["task"]}")', flush=True)
    CAL = LA.dummy_calibration()
    CLOCK = {'t0': None}
else:
    leader = LA.Leader(port=args.port, name=args.calib)
    print(f'leader arm: calibration {args.calib} ({leader.cal["date"]}), reading OK: {leader.read()}', flush=True)

import stations_mono as ST                                                    # noqa: E402
import scene_robot as R                                                  # noqa: E402
st = ST.load()
app = R.start_app(args.headless)
import numpy as np                                                       # noqa: E402
import cameras as CAM                                                    # noqa: E402
import mono_poses as MP                                                  # noqa: E402
import scara_ik as K                                                     # noqa: E402
import proto_task as PT                                                  # noqa: E402
import pick_place_mono as PP                                                  # noqa: E402
import sim_common as SCM                                                 # noqa: E402
import episode_writer as EW                                              # noqa: E402

task, items, watch = R.load(app, args.headless)
if not task.cams:
    sys.exit('no cameras (MM_CAMERAS=0?): the recorder needs them')
PP.init(task, items, watch, st, tilt=args.tilt, headless=args.headless, out=None)
SCM.bind(task, items, PP)
if DUMMY:
    leader = LA.Leader(bus=LA.ReplayBus(D['actions'], C.FPS, CAL,
                                        lambda: 0.0 if CLOCK['t0'] is None else task.t - CLOCK['t0']), calib=CAL)

ARM = [(k, u) for k, (n, u) in enumerate(C.JOINTS) if n != 'lift.pos']   # (index in the 7-vector, URDF joint(s))
ARM_IDX = [task.idx[u] for _, u in ARM if isinstance(u, str)]           # the 5 arm joints (not the fingers)
LI = task.idx['lift']
LIFT_RANGE = (0.0, K.LIFT_MAX['prototype_mono_arm'])
LIFT_V = 0.10 * 0.9                                     # m/s, the URDF lift speed with drives.rate_limit's margin
SYNC_SPEED = 0.3                                        # the slow move to the leader's pose: this x the joint speeds
SYNC_TOL = math.radians(2)
FOLD_TOL = math.radians(8)

S = dict(mode='idle', deg=None, cmd=None, sid=None, kind='pick', name=None, keys=set(), req=None, p0=None,
         vmax=None, last=None, msg='', read_err=0)
READ_LOST = 30                                          # missed leader reads in a row (0.5 s) -> teleop stops, arm holds
writer = EW.EpisodeWriter(task, items, C.RAW_DIR / args.name, 'leader_dummy' if DUMMY else 'leader', args.tilt,
                          jpeg=args.jpeg, depth=args.depth, keep_failed=True,
                          frame_extra=lambda: {'leader.deg': [S['deg'][j] for j, *_ in LA.JOINTS]})


def set_arm_goal(cmd):
    for (k, u), (_, _, name, _) in zip(ARM, LA.JOINTS):
        for j in (u if isinstance(u, tuple) else (u,)):
            task.q_goal[task.idx[j]] = cmd[name] / (2 if isinstance(u, tuple) else 1)


def hold():
    """The arm stays where it is commanded now (teleop off)."""
    task.q_goal[:] = task.q_cmd


# ---------------------------------------------------------------- every control step (60 Hz)
_step = task.step_once


def step_teleop():
    if S['mode'] in ('sync', 'follow'):
        try:
            S['deg'] = leader.degrees(); S['read_err'] = 0
        except IOError:                               # a missed servo packet: keep the last pose for a moment
            S['read_err'] += 1
            if S['read_err'] >= READ_LOST or S['deg'] is None:
                if S['mode'] == 'sync':
                    task.vmax = S['vmax']
                S['mode'] = 'idle'; hold()
                S['msg'] = 'LEADER LOST (no servo reply for 0.5 s): teleop off, the arm holds. Check the USB / power.'
                print('      ' + S['msg'], flush=True)
                _step(); writer.step()
                return
        S['cmd'] = leader.command(S['deg'])
        set_arm_goal(S['cmd'])
        if DUMMY:
            task.q_goal[LI] = leader.bus.lift()
        else:
            d = ('up' in S['keys']) - ('down' in S['keys'])
            if d:
                task.q_goal[LI] = min(LIFT_RANGE[1], max(LIFT_RANGE[0], task.q_goal[LI] + d * LIFT_V * PT.DT))
        if S['mode'] == 'sync' and np.max(np.abs(task.q_cmd[ARM_IDX] - task.q_goal[ARM_IDX])) < SYNC_TOL:
            task.vmax = S['vmax']; S['mode'] = 'follow'
            S['msg'] = 'following the leader'
            print('      teleop: synced, following the leader', flush=True)
    _step()
    writer.step()


task.step_once = step_teleop

# ---------------------------------------------------------------- actions (buttons, keys, --auto)
def folded():
    """The arm is in one of the two travel folds (normal, ring -80 / mirrored, ring +80): remembered for driving."""
    q = task.robot.get_joint_positions()
    for stow in (PP.ROBOT['stow'], MP.STOW, PP.STOW_LEFT):
        if max(abs(q[task.idx[f'left_{k}']] - v) for k, v in stow.items() if k != 'lift') < FOLD_TOL:
            PP.ROBOT['stow'] = stow
            return True
    return False


def go_to(sid, kind):
    if S['mode'] != 'idle':
        return 'stop teleop first'
    if not folded():
        return 'fold the arm first ("Fold arm" or fold it with the leader)'
    name, _ = PP.product_for(sid)
    if kind == 'place':
        inside, _ = SCM.in_tote(name)
        if not inside:
            return f'{PP.label(sid)} is not in the tote: record or do a pick first'
    else:
        n = PP.restore()
        if n:
            print(f'      {n} product(s) put back where they started', flush=True)
    if PP.ROBOT['at'] != sid:
        R.leg(task, st, PP.ROBOT['at'], sid)
        PP.ROBOT['at'] = sid
    if DUMMY:                                         # rebuild the recorded start exactly (as replay_episode.py)
        a = D['attrs']
        task.set_pose({n_: float(v) for n_, v in zip(a['q0_names'], a['q0'])})
        SCM.teleport(name, a['product_pose0'])
        SCM.set_light(float(a.get('light_scale', 1.0)))
    S.update(sid=sid, kind=kind, name=name)
    return f'at {sid}: {PP.label(sid)} ({kind}). "Start teleop" next.'


def start_teleop():
    # works anywhere, also at home (2026-11-23, user: the arm didn't move: Start needed a station first); only
    # RECORDING needs a station (an episode belongs to a station + product)
    if S['mode'] != 'idle':
        return 'already on'
    if DUMMY:
        CLOCK['t0'] = task.t
    S['vmax'] = task.vmax.copy()
    task.vmax = S['vmax'] * SYNC_SPEED
    leader.last = {}                                  # the roll starts counting from this reading
    S['read_err'] = 0
    S['mode'] = 'sync'
    where = '' if S['sid'] else ' (at home: you can move the arm, but recording needs "Go to station")'
    return 'moving slowly to the leader\'s pose ...' + where


def stop_teleop():
    if writer.active:
        return 'save (N) or discard (X) the episode first'
    if S['mode'] == 'sync':
        task.vmax = S['vmax']
    S['mode'] = 'idle'; hold()
    return 'teleop off: the arm holds its pose'


def record():
    if S['sid'] is None:
        return 'go to a station first: an episode belongs to a station and its product'
    if S['mode'] != 'follow':
        return 'start teleop first (and wait until it follows)'
    if writer.active:
        return 'already recording'
    watch_ = {**PP.watch, S['name']: items[S['name']]['bodies'][0]}
    S['p0'] = (watch_, R.positions(watch_))
    writer.start(S['kind'], S['sid'], S['name'])
    print(f'      recording: {S["kind"]} {S["name"]}', flush=True)
    return 'RECORDING: do the task, fold the arm, then N = save / X = discard'


def check():
    """(ok, text) of the task as it stands now."""
    name = S['name']
    if S['kind'] == 'pick':
        ok, pb = SCM.in_tote(name)
        txt = 'in the tote' if ok else f'NOT in the tote (at {pb[0]:+.2f}, {pb[1]:+.2f}, {pb[2]:.2f} m)'
    else:
        pf, Rf = PP.pose_b(PP.MOVABLE[name][0].prim_path)
        p_start = task.to_base(PP.START[name][0][0]); R_start = PP.pose_b(None, PP.START[name][0])[1]
        d = np.linalg.norm(pf[:2] - p_start[:2]) * 1000
        tl = math.degrees(math.acos(max(-1.0, min(1.0, (Rf @ R_start.T)[2, 2]))))
        ok = d < 20 and tl < 5
        txt = f'{d:.0f} mm from its spot, tilted {tl:.1f} deg'
    moved = {n: d_ for n, d_ in R.knocked(*S['p0']).items() if n != name}
    if moved:
        ok = False; txt += '; knocked: ' + ', '.join(f'{n} {d_ * 1000:.0f} mm' for n, d_ in moved.items())
    if not folded():
        txt += ' (arm not folded at the end)'
    return ok, txt


def save():
    if not writer.active:
        return 'not recording'
    ok, txt = check()
    extra = dict(leader_calibration=leader.cal['name'], leader_calibration_date=leader.cal['date'], light_scale=1.0)
    if S['kind'] == 'place':
        extra['goal_pose'] = np.array([np.concatenate([p, q]) for p, q in PP.START[S['name']]])
    if DUMMY:
        extra['dummy_source'] = str(args.dummy_episode or ep)
    path = writer.stop(ok, txt, extra)
    return f'saved {path.name}: {"OK" if ok else "FAILED"}, {txt}'


def discard():
    if not writer.active:
        return 'not recording'
    writer.stop(False, 'discarded', keep=False)
    return 'discarded'


def fold():
    if S['mode'] != 'idle':
        return 'stop teleop first'
    task.joint_move('left', dict(PP.ROBOT['stow']))
    return 'folded'


ACTIONS = dict(go=None, start=start_teleop, stop=stop_teleop, record=record, save=save, discard=discard, fold=fold,
               restore=lambda: f'{PP.restore()} product(s) put back',
               reset=lambda: (stop_teleop() if not writer.active else None, PP.reset_robot(), 'robot back home')[-1])

choices = sorted(PP.PICKABLE, key=PP.label)
sel = {'i': choices.index(D['attrs']['station']) if DUMMY and D['attrs']['station'] in choices else 0, 'kind': 0}
if DUMMY and str(D['attrs']['kind']) == 'place':
    sel['kind'] = 1


def do(req):
    if req == 'go':
        r = go_to(choices[sel['i']], ('pick', 'place')[sel['kind']])
    else:
        r = ACTIONS[req]()
    S['msg'] = r
    print(f'  [{req}] {r}', flush=True)
    return r


# ---------------------------------------------------------------- the window: panel, camera views, keys
status = None
if not args.headless:
    import carb.input, omni.appwindow, omni.ui as ui
    from omni.kit.viewport.utility import create_viewport_window
    sres, wres = CAM.SCENE['res'], CAM.WRIST['res']
    for title, cam, r, x in (('Scene camera', 'scene', sres, 20), ('Wrist camera', 'left_wrist', wres, sres[0] + 50)):
        v = create_viewport_window(title, width=r[0] + 20, height=r[1] + 45, position_x=x, position_y=820,
                                   camera_path=task.cams[cam].prim_path)
        try:
            v.viewport_api.resolution = r
        except Exception:
            pass
    win = ui.Window('Teleop recording', width=640, height=330, position_x=40, position_y=90)
    btn = lambda label, req: ui.Button(label, clicked_fn=lambda: S.update(req=req))
    with win.frame:
        with ui.VStack(spacing=6, height=0):
            with ui.HStack(height=24):
                ui.Label('Product', width=70)
                combo = ui.ComboBox(sel['i'], *[PP.label(n) for n in choices])
            with ui.HStack(height=24):
                ui.Label('Task', width=70)
                kcombo = ui.ComboBox(sel['kind'], 'Pick into the tote', 'Place back on its shelf (from the tote)')
            with ui.HStack(height=28, spacing=6):
                btn('Go to station', 'go'); btn('Fold arm', 'fold'); btn('Put products back', 'restore'); btn('Reset robot', 'reset')
            with ui.HStack(height=28, spacing=6):
                btn('Start teleop', 'start'); btn('Stop teleop', 'stop')
            with ui.HStack(height=28, spacing=6):
                btn('Record (R)', 'record'); btn('Save (N)', 'save'); btn('Discard (X)', 'discard')
            status = ui.Label('', word_wrap=True, height=80)
            ui.Label('Up / Down arrows: lift (while teleop is on). Click the viewport if a dropdown keeps the keys.',
                     style={'color': 0xFF9A9A9A}, word_wrap=True)
    combo.model.get_item_value_model().add_value_changed_fn(lambda m: sel.update(i=m.as_int))
    kcombo.model.get_item_value_model().add_value_changed_fn(lambda m: sel.update(kind=m.as_int))

    _KI = carb.input.KeyboardInput
    _press = carb.input.KeyboardEventType.KEY_PRESS
    _held = (_press, carb.input.KeyboardEventType.KEY_REPEAT)

    def on_key(e, *_):
        k = {_KI.UP: 'up', _KI.DOWN: 'down'}.get(e.input)
        if k:
            (S['keys'].add if e.type in _held else S['keys'].discard)(k)
        elif e.type == _press:
            r = {_KI.R: 'record', _KI.N: 'save', _KI.X: 'discard'}.get(e.input)
            if r:
                S['req'] = r
        return True
    _key_sub = carb.input.acquire_input_interface().subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(), on_key)


def show():
    if status is None:
        return
    mode = {'idle': 'teleop OFF', 'sync': 'moving to the leader ...', 'follow': 'FOLLOWING the leader'}[S['mode']]
    rec = f'   RECORDING {writer.frames() / C.FPS:5.1f} s' if writer.active else ''
    live = ''
    if S['name'] and S['p0'] is not None and writer.active:
        live = '   product in the tote: ' + ('yes' if SCM.in_tote(S['name'])[0] else 'no') if S['kind'] == 'pick' else ''
    lead = ''
    if S['mode'] != 'idle' and S['deg'] is not None:   # the leader's angles (* = past the follower limit, clamped)
        cells = []
        for j, _, name, lim in LA.JOINTS:
            if j == 'gripper':
                cells.append(f"grip {S['cmd'][name] * 1000:.0f}mm")
            else:
                cells.append(f"{j} {S['deg'][j]:+.0f}" + ('*' if not lim[0] <= S['deg'][j] <= lim[1] else ''))
        lead = '\nleader: ' + '  '.join(cells)
    status.text = f"{mode}{rec}   lift {task.robot.get_joint_positions()[LI] * 100:.0f} cm{live}{lead}\n{S['msg']}"


# ---------------------------------------------------------------- --auto (dummy): the whole round by itself
def auto():
    """go -> teleop -> record -> (the replay ends) -> save; returns the save text."""
    for req in ('go', 'start'):
        r = do(req)
        if req == 'go' and not r.startswith('at '):
            return r
    while S['mode'] != 'follow':
        task.step_once()
    do('record')
    end = CLOCK['t0'] + leader.bus.duration() + 1.0
    while task.t < end:
        task.step_once(); show()
    r = do('save')
    do('stop')
    return r


print('teleop: use the "Teleop recording" panel' + (' (dummy leader)' if DUMMY else ''), flush=True)
result = None
try:
    if args.auto:
        result = auto()
    if not (args.auto and args.headless):
        if args.auto:
            import omni.kit.app
            _quit = time.time() + 3
        while app.is_running():
            req, S['req'] = S['req'], None
            if req:
                do(req)
            else:
                task.step_once()
            if args.auto and time.time() > _quit:
                omni.kit.app.get_app().post_quit(); args.auto = False
            if task.t - (S['last'] or 0) > 0.25:
                S['last'] = task.t; show()
except SystemExit:                                    # window closed (proto_task.tick raises it)
    pass
if writer.active:
    writer.stop(False, 'window closed while recording', keep=False)
if result:
    print('RESULT:', result, flush=True)
PP.close()
app.close()
