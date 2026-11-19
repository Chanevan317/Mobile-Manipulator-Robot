"""Pick a store product into the robot's tote and put it back: the planner-driven pick / place of pick_to_tote.py,
importable (2026-11-19) so the station panel and the LeRobot recorder (train_pipeline/record) share one copy.

    import pick_place as PP
    PP.init(task, items, watch, st, tilt=40, headless=True, out=None)   # after scene_robot.load(); builds the planner
    ok, why = PP.run(sid)            # put moved products back, drive to station sid, pick into the tote, fold
    ok, why = PP.run_place()         # the last picked product from the tote back to its spot
    PP.restore(); PP.reset_robot(); PP.close()

Moves are checked offline (collide.py: furniture, products, the robot, the carried product); plans are cached in
~/.cache/mobile_manipulator/pick_to_tote_plans.json. See pick_to_tote.py and CLAUDE.md item 29 for the lessons.
"""
import json, math, os, time
import numpy as np
import cameras as CAM
import grasp_plan as G
import mono_poses as MP
import scene_robot as R
import scara_ik as K
import collide as CO
import scene as SC
import assets as SA

P = math.pi / 2
READY_GAP = 0.12        # m: the hand this far in front of the furniture while the arm unfolds / folds
STOW_LEFT = {**MP.STOW, 'shoulder_pan': -MP.STOW['shoulder_pan'], 'elbow_pan': -MP.STOW['elbow_pan'],
             'wrist_pan': -MP.STOW['wrist_pan']}   # the travel fold mirrored: clear at lift 0 .. 0.81 (offline check)
PLAN_SPEED = 0.4        # planned (waypoint) moves run at this fraction of the joint speeds
CACHE_FILE = os.path.join(os.path.expanduser('~'), '.cache', 'mobile_manipulator', 'pick_to_tote_plans.json')

# set by init()
task = items = watch = st = cams = planner = chk = None
CACHE = {}
OPT = dict(tilt=40.0, headless=False, out=None, scene_cam=CAM.SCENE_MODEL)
shots = []
MOVABLE, START = {}, {}
ROBOT = {'at': 'home', 'stow': MP.STOW}       # where the robot is parked and which travel fold it has
CTX = {}                                      # the last pick, while its product is in the tote: {'place': fn, 'sid', 'name'}
# recording hooks (train_pipeline/record): HOOKS[event](sid=..., name=...) for 'pick_chosen' (product chosen, before
# driving), 'pick_start' (parked at the station), 'pick_end' (folded, reports done), 'place_start', 'place_end'
HOOKS = {}


def _hook(event, **kw):
    fn = HOOKS.get(event)
    if fn:
        fn(**kw)


def init(task_, items_, watch_, st_, tilt=40.0, headless=False, out=None, workers=None):
    """Bind to a loaded scene (scene_robot.load) and start the planner. out: folder for headless camera shots (None = off)."""
    global task, items, watch, st, cams, planner, chk, CACHE, MOVABLE, START
    task, items, watch, st = task_, items_, dict(watch_), st_
    cams = task.cams
    OPT.update(tilt=tilt, headless=headless, out=out, scene_cam=CAM.SCENE_MODEL)
    if out:
        os.makedirs(out, exist_ok=True)
    planner = CO.Planner('prototype_mono_arm', workers) if workers else CO.Planner('prototype_mono_arm')
    chk = planner.chk
    try:
        CACHE = json.load(open(CACHE_FILE))
    except Exception:
        CACHE = {}
    task.q_goal[task.idx['scene_camera_pitch']] = math.radians(tilt)
    task.profile = 'trap'        # as the mono shelf task: trapezoid moves (full allowed speed after a 0.3 s ramp)
    task.settle_mode = 'still'   # a move is done once the joints stop (holding a product the wrist pitch sags)
    if os.environ.get('P2T_DEBUG'):  # after every move: commanded vs measured joints, TCP, jaw gap              # after every move: commanded vs measured joints, TCP, jaw gap
        def _wrap(fn, name_):
            def w(*a_, **k_):
                r = fn(*a_, **k_)
                q = task.robot.get_joint_positions(); g = task.q_goal
                err = {k: round(math.degrees(q[task.idx[f'left_{k}']] - g[task.idx[f'left_{k}']]), 1) for k in ('shoulder_pan', 'elbow_pan', 'wrist_pan', 'wrist_pitch')}
                err['lift_mm'] = round((q[task.idx['lift']] - g[task.idx['lift']]) * 1000, 1)
                t = task.to_base(task.tcp['left'].get_world_pose()[0])
                gap = (q[task.idx['left_finger_right']] + q[task.idx['left_finger_left']]) * 1000
                print(f'      [debug] {name_:10s} TCP ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:.3f}) jaw gap {gap:5.1f}  errors deg {err}', flush=True)
                return r
            return w
        task.joint_move = _wrap(task.joint_move, 'joint_move'); task.cart_move = _wrap(task.cart_move, 'cart_move')
        task.jaw = _wrap(task.jaw, 'jaw')
    from isaacsim.core.prims import SingleXFormPrim
    MOVABLE = {n: [SingleXFormPrim(b_) for b_ in it['bodies']] for n, it in items.items()
               if it['role'] in ('product', 'filler') and it.get('bodies') and (it['role'] == 'product' or it.get('dynamic'))}
    START = {n: [tuple(np.array(v, float) for v in b_.get_world_pose()) for b_ in bs] for n, bs in MOVABLE.items()}


def close():
    if planner is not None:
        planner.close()


def shot(label):
    """Headless: the two views side by side -> <out>/<station>_<k>_<label>.png."""
    if not OPT['headless'] or not OPT['out'] or not cams:
        return
    from PIL import Image, ImageDraw
    fr = CAM.capture(task, cams)
    a, b = fr['scene_rgb'], fr['left_wrist_rgb']
    im = Image.new('RGB', (a.shape[1] + b.shape[1] + 10, max(a.shape[0], b.shape[0]) + 30), (255, 255, 255))
    im.paste(Image.fromarray(a), (0, 30)); im.paste(Image.fromarray(b), (a.shape[1] + 10, 30))
    d = ImageDraw.Draw(im)
    d.text((8, 8), f'scene camera ({OPT["scene_cam"]}, tilt {OPT["tilt"]:.0f} deg): {label}', fill=(0, 0, 0))
    d.text((a.shape[1] + 18, 8), 'wrist camera', fill=(0, 0, 0))
    p = os.path.join(OPT['out'], f'{ROBOT["at"]}_{len(shots):02d}_{label.replace(" ", "_")}.png'); im.save(p); shots.append(p)


# ---------------------------------------------------------------- one pick
def pick(sid):
    """Drive to station sid, pick its product, put it in the tote, fold. Raises RuntimeError when it can't."""
    S = st[sid]
    global watch
    pkg = S['product']
    # the nearest movable product of that kind to the station's target: a task product, or a filler with physics (the scene
    # has one task product per package; e.g. the yogurt task product is on the table, the dairy board's yogurts are fillers)
    cand = {n: it['bodies'][0] for n, it in items.items() if it.get('pkg') == pkg and it['role'] in ('product', 'filler')
            and it.get('bodies') and (it['role'] == 'product' or it.get('dynamic'))}
    if not cand:
        raise RuntimeError(f'no movable product {pkg} in the scene')
    pos0 = R.positions(cand)
    tgt = np.array(S['target'][:2])
    name = min(cand, key=lambda n: (items[n]['role'] != 'product' or np.linalg.norm(pos0[n][:2] - tgt) > 0.3,
                                     np.linalg.norm(pos0[n][:2] - tgt)))      # the task product if it is there, else the nearest
    if items[name]['role'] != 'product':
        print(f'NOTE: {name} is a filler in a packed run (facings ~6 mm apart); the open jaws need ~15 mm beside the product, '
              f'so the grasp may fail. The scene leaves 50 mm gaps only around the task products (scene.TASK_GAP).', flush=True)
    watch = {**watch, name: cand[name]}
    _hook('pick_chosen', sid=sid, name=name)
    narrow, deep, h = G.dims(pkg)
    print(f"\nPick to tote: {name} ({pkg}), {narrow*1000:.0f} x {deep*1000:.0f} x {h*1000:.0f} mm, {S['grasp']} grasp, "
          f"scene camera {OPT['scene_cam']} at {OPT['tilt']:.0f} deg\n", flush=True)

    if True:
        task.wait(0.5)
        # which side the arm will reach round: the grasp's ring angle, predicted from the product's resting spot and the
        # station's base pose. The ring can't wrap past +-180 deg, so going from the right-hand travel fold (ring -80) to a
        # grasp with the ring on the left turns it through the front, past the furniture (it hit the grey shelf's front post).
        # Such grasps use the MIRRORED fold (ring +80), taken at home in open space before driving.
        zb_, rx, ry, _ = items[name]['rest']; sy_ = math.radians(S['yaw'])
        dx, dy = rx - S['x'], ry - S['y']
        c_pred = (math.cos(sy_) * dx + math.sin(sy_) * dy, -math.sin(sy_) * dx + math.cos(sy_) * dy)
        gz_p, fz_p = G.grip_heights(S['surface'], h)
        a_p = G.side_poses(c_pred, 0.0, deep, gz_p, 0.03)[0] if S['grasp'] == 'side' else G.top_poses(c_pred, 0.0, fz_p, 0.10)[0]
        q_p = next((K.ik('left', *a_p, elbow=el) for el in (1, -1) if K.ik('left', *a_p, elbow=el)), None)
        stow = STOW_LEFT if (q_p and q_p['shoulder_pan'] > 0) else MP.STOW
        if stow is not ROBOT['stow']:                 # the fold changes only at home (open space)
            if ROBOT['at'] != 'home':
                R.leg(task, st, ROBOT['at'], 'home'); ROBOT['at'] = 'home'
            task.report('travel fold', 'the grasp reaches round the ' + ('left: mirrored fold (ring +80 deg)' if stow is STOW_LEFT
                        else 'right: normal fold (ring -80 deg)') + ', switched at home')
            task.joint_move('left', dict(stow)); ROBOT['stow'] = stow
        if ROBOT['at'] != sid:
            R.leg(task, st, ROBOT['at'], sid)
        ROBOT['at'] = sid
        _hook('pick_start', sid=sid, name=name)
        p0 = R.positions(watch)
        c = task.to_base(p0[name])[:2]
        if HOOKS.get('grasp_offset'):                 # train_pipeline noise: aim a little off the product, like a person
            c = c + np.asarray(HOOKS['grasp_offset'](sid=sid, name=name), float)
        bp, byaw = task.base_pose()
        furn = CO.boxes_to_base(SC.furniture_boxes(), bp[:2], byaw, margin=0.005)  # furniture, 5 mm margin

        def product_boxes(skip=()):
            """Products / fillers within 1.2 m as boxes where they rest (the scene's placement; the task product measured)."""
            out = []
            for n, it in items.items():
                if it['role'] not in ('product', 'filler') or n in skip or 'rest' not in it:
                    continue
                zb, x, y, _ = it['rest']
                if n == name:
                    x, y = p0[name][:2]
                if math.hypot(x - bp[0], y - bp[1]) > 1.2:
                    continue
                sx, sy, sz = (v / 1000 for v in SA.size(it['pkg'])); yw = math.radians(it.get('yaw', 0.0))
                ax, ay = abs(math.cos(yw)) * sx + abs(math.sin(yw)) * sy, abs(math.sin(yw)) * sx + abs(math.cos(yw)) * sy
                out.append((n, (x, y, zb + sz / 2), (ax, ay, sz)))
            return CO.boxes_to_base(out, bp[:2], byaw)
        env = furn + product_boxes()                     # the swing to the product: furniture + every product near
        env_held = furn + product_boxes(skip=(name,))    # after the pick: the carried product is not on the shelf any more
        if os.environ.get('P2T_DEBUG'):
            for nm, ctr, sz in sorted(product_boxes(), key=lambda b_: math.hypot(*b_[1][:2]))[:8]:
                print(f'      [debug] {nm:40s} base ({ctr[0]:+.3f}, {ctr[1]:+.3f}, {ctr[2]:.3f}) size {np.round(sz, 3)}', flush=True)
        def now(jaw):
            return {**task.arm_now('left'), 'jaw': jaw}

        gz, fz = G.grip_heights(S['surface'], h)
        if S['grasp'] == 'side':
            a, b, up = G.side_poses(c, 0.0, deep, gz, 0.03)
            out = (a[0], a[1], up[2], 0.0, 0.0)
            e = G._ok((a, b, up, out))
            slow = 2.5                                    # straight in / out slowly (mono_poses SLOW_IN: servo lag)
            near = min(deep / 2, 0.03)                    # TCP this far past the product's near face
            held = CO.held_box((deep / 2 - near, 0.0, h / 2 - (gz - S['surface'])), (deep, narrow, h))
            below = deep - near                           # lying in the tote: product reaches this far below the TCP
        else:
            # hand down from above; on a shelf the board above leaves little room (the forearm rides ~0.3 m over the TCP):
            # the highest approach / lift height (80 .. 25 mm over the grasp) whose pose is clear
            e = None
            # hand heading that puts the product's narrow side across the jaws (scene.product_yaw, inverted), base frame
            sx_, sy_, _ = SA.size(pkg)
            hb = K.wrap(math.radians(items[name].get('yaw', 0.0) - (90.0 if sx_ < sy_ else 0.0)) - byaw)
            for dz in np.arange(0.08, 0.0249, -0.005):
                for yaw in (hb, K.wrap(hb + math.pi)):
                    a, b, up = G.top_poses(c, yaw, fz, dz)
                    a = up = (a[0], a[1], fz + dz, a[3], a[4])
                    e = G._ok((a, b, up))
                    if e and not chk.check({**now(0.0), **K.ik('left', *a, elbow=e)}, env):
                        break
                    e = None
                if e: break
            if not e:
                raise RuntimeError('top grasp: no approach from above fits (hand down, the forearm rides ~0.36 m over the '
                                   'fingertips and would hit the board above): this product cannot be picked here')
            out = up
            slow = None
            held = CO.held_box((fz - S['surface'] - h / 2, 0.0, 0.0), (h, narrow, deep))   # hanging under the TCP (tcp x = down)
            below = fz - S['surface']                     # upright: its bottom this far under the TCP
        if not e:
            raise RuntimeError(f'no IK for the grasp at ({c[0]:.3f}, {c[1]:.3f}) in the base frame')
        jopen = min(0.045, narrow / 2 + 0.015)
        task.report('product seen at', f'({c[0]:+.3f}, {c[1]:+.3f}) in the base frame, grip at z {gz if S["grasp"] == "side" else fz:.3f}')

        def slow_move(q):
            """A planned joint move at PLAN_SPEED of the usual speed: the geared ring lags the other joints at full speed, so
            the arm leaves the checked straight joint path (on the way back it swung the hand into the shelf's front post)."""
            v = task.vmax.copy(); task.vmax = v * PLAN_SPEED
            try:
                task.joint_move('left', {k: val for k, val in q.items() if k != 'jaw'})
            finally:
                task.vmax = v

        def go(q, label, held_=None, jaw=0.0, env_=None):
            """Joint move to q without contact (collide.py offline check): straight, or through a waypoint, or swinging at
            another lift height (lift there with the arm still, swing, lift back with the arm still): a ring swing past the
            front of a shelf only fits in the gap above the products and under the next board."""
            env_ = env if env_ is None else env_
            q0 = now(jaw); q1 = {**q0, **q}
            if os.environ.get('P2T_DUMP'):                # the planning problem as JSON, to work on the planner offline
                json.dump(dict(label=label, q0=q0, q1=q1, env=[(n_, list(map(float, c_)), list(map(float, z_))) for n_, c_, z_ in env_],
                               held=None if held_ is None else [list(map(float, held_[0])), list(map(float, held_[1]))]),
                          open(os.path.join(os.environ['P2T_DUMP'], label.replace(' ', '_') + '.json'), 'w'))
            t0 = time.time()
            key = f'{sid} | {label}'
            plan = None
            if key in CACHE:                              # a plan found on an earlier run: re-check it against the scene now
                cand = [{**q0, **m} for m in CACHE[key][:-1]] + [q1]
                if chk.plan_clear(q0, cand, env_, held_):
                    plan = cand
            if plan is None:
                print(f'      planning "{label}" (offline collision search; the first time at a station can take a few minutes)', flush=True)
                plan = planner.transfer(q0, q1, env_, held_, lifts=np.arange(0.0, K.LIFT_MAX['prototype_mono_arm'] + 1e-6, 0.05))
                if plan is not None:
                    CACHE[key] = [{k: float(v) for k, v in m.items()} for m in plan]
                    os.makedirs(os.path.dirname(CACHE_FILE), exist_ok=True)
                    json.dump(CACHE, open(CACHE_FILE, 'w'), indent=0)
            if plan is None:
                raise RuntimeError(f'{label}: no collision-free move found (offline check; start touches '
                                   f'{chk.check(q0, env_, held_)[:3]}, goal touches {chk.check(q1, env_, held_)[:3]})')
            how = 'direct' if len(plan) == 1 else (f'{len(plan) - 1} waypoint(s)' + (f', moving at lift {plan[0]["lift"]:.2f}' if abs(plan[0]['lift'] - q0['lift']) > 1e-6 else ''))
            print(f'      plan "{label}": {how} ({time.time() - t0:.1f} s offline)', flush=True)
            for m in plan:
                (task.joint_move('left', {k: v for k, v in m.items() if k != 'jaw'}) if len(plan) == 1 else slow_move(m))
            return plan

        # unfold to a READY pose in open space (hand level, pointing at the product, READY_GAP in front of the furniture),
        # then straight in to the approach start: a swing that ends right at the shelf has to thread between the products
        ready = a
        front = c[0] - (deep if S['grasp'] == 'side' else max(narrow, deep)) / 2 - 0.03
        if True:
            for x in np.arange(min(a[0], front - READY_GAP), 0.12, -0.01):
                qr = K.ik('left', x, a[1], a[2], a[3], a[4], elbow=e)
                if qr and not chk.check({**now(0.0), **qr}, env):
                    ready = (float(x), a[1], a[2], a[3], a[4]); break
        qa = task.ik('left', ready, e)
        task.lift_to(qa['lift'])                          # the travel fold is clear at any lift
        unfold = go(qa, 'unfold to the ready pose', jaw=0.0)       # jaws closed while swinging: less to clear
        if ready is not a:
            task.cart_move('left', ready, a, e)
        task.jaw('left', jopen, 0.6)
        shot('in front of the product')
        task.cart_move('left', a, b, e, slow)
        task.jaw('left', 0.0, 1.0)
        q = task.robot.get_joint_positions()
        gap = (q[task.idx['left_finger_right']] + q[task.idx['left_finger_left']]) * 1000
        shot('grasped')
        z_grasp = R.positions(watch)[name][2]
        task.cart_move('left', b, up, e)
        if S['grasp'] == 'side':
            task.cart_move('left', up, out, e, slow)
        if ready is not a:                                # back out level to the ready spot, then swing
            task.cart_move('left', out, (ready[0], ready[1], out[2], out[3], out[4]), e)
        rise = (R.positions(watch)[name][2] - z_grasp) * 1000
        task.report('pick', f'jaw gap {gap:.1f} mm (product {narrow*1000:.0f}), lifted {rise:.0f} mm', ok=narrow * 500 < gap < narrow * 1000 + 8 and rise > 15)   # tapered cups: gripped below the rim, narrower than their box

        # over the tote, hand down: the heading / elbow whose release pose (and the pose above it) are clear with the product
        zr = MP.TOTE_FLOOR + below + float(os.environ.get('P2T_DROP', 0.02))   # release with the product's lowest point 20 mm over the tote floor
        hi = zr + MP.TOTE_UP
        tx, ty = MP.TOTE_FLAT
        jaw_held = narrow / 2
        cur = now(jaw_held); keys = ('shoulder_pan', 'elbow_pan', 'wrist_pan')
        opts = []
        for yw in MP.YAWS:
            for el in (1, -1):
                qh, qr = K.ik('left', tx, ty, hi, yw, P, elbow=el), K.ik('left', tx, ty, zr, yw, P, elbow=el)
                if qh and qr:
                    opts.append((sum(abs(qh[k] - cur[k]) for k in keys), yw, el, qh))
        tote = None
        for _, yw, el, qh in sorted(opts, key=lambda o: o[0]):
            if not chk.check({**cur, **qh, 'jaw': jaw_held}, env_held, held) and \
               not chk.check({**cur, **K.ik('left', tx, ty, zr, yw, P, elbow=el), 'jaw': jaw_held}, env_held, held):
                tote = (yw, el, qh); break
        if tote is None:
            raise RuntimeError('no clear hand-down pose over the tote for this product')
        yt, et, qt = tote
        go(qt, 'swing to the tote', held, jaw_held, env_held)
        shot('over the tote')
        task.cart_move('left', (tx, ty, hi, yt, P), (tx, ty, zr, yt, P), et)
        rel_b = pose_b(cand[name])                    # the product in the hand just before release (for the re-grasp)
        def _tilt(tag):
            if os.environ.get('P2T_DEBUG'):
                pn_, Rn_ = pose_b(cand[name]); Rq = Rn_ @ rel_b[1].T
                print(f'      [debug] {tag}: tilt {math.degrees(math.acos(max(-1, min(1, Rq[2, 2])))):.0f} deg, moved '
                      f'{np.round((pn_ - rel_b[0]) * 1000):} mm', flush=True)
        task.jaw('left', jopen + 0.01, 0.8)
        _tilt('jaws open')
        task.wait(0.6)
        _tilt('0.6 s later')
        shot('released in the tote')
        task.cart_move('left', (tx, ty, zr, yt, P), (tx, ty, hi, yt, P), et)
        _tilt('hand out')
        task.jaw('left', 0.0, 0.5)
        go({**stow, 'lift': task.arm_now('left')['lift']}, 'fold for travel', env_=env_held)
        _tilt('folded')
        task.joint_move('left', {'lift': stow['lift']})
        task.wait(1.0)
        _tilt('lift at the fold height')
        pb = task.to_base(R.positions(watch)[name])
        x0, x1, y0, y1 = MP.TOTE_BOX
        inside = x0 < pb[0] < x1 and y0 < pb[1] < y1 and pb[2] < 0.376
        task.report('in the tote', f'product at ({pb[0]:+.3f}, {pb[1]:+.3f}, {pb[2]:.3f}) in the base frame', ok=inside)
        moved = {n: d for n, d in R.knocked(watch, p0).items() if n != name}
        task.report('nothing else moved', ', '.join(f'{n} {d*1000:.0f} mm' for n, d in moved.items()) or 'none over 10 mm', ok=not moved)
        shot('done')
        _hook('pick_end', sid=sid, name=name)

        def place_back():
            """Pick the product out of the tote and put it back where it stood. The re-grasp keeps the hand-product
            relation of the shelf grasp: the TCP of the release, moved with the product (it drops 20 mm and may slide /
            turn), so the shelf moves of the pick, run backwards, set it down on its spot."""
            if ROBOT['at'] != sid:
                if ROBOT['stow'] is not stow:
                    if ROBOT['at'] != 'home':
                        R.leg(task, st, ROBOT['at'], 'home'); ROBOT['at'] = 'home'
                    task.joint_move('left', dict(stow)); ROBOT['stow'] = stow
                R.leg(task, st, ROBOT['at'], sid); ROBOT['at'] = sid
            print(f"\nPlace back: {name} from the tote to its spot at {sid}\n", flush=True)
            _hook('place_start', sid=sid, name=name)
            (pr, Rr), (pn, Rn) = rel_b, pose_b(cand[name])
            Rrel = Rn @ Rr.T
            if os.environ.get('P2T_DEBUG'):
                print(f'      [debug] product at release {np.round(pr, 3)} axes (cols x y z)\n{np.round(Rr, 2)}\n'
                      f'      [debug] product now {np.round(pn, 3)}\n{np.round(Rn, 2)}', flush=True)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, Rrel[2, 2]))))
            if tilt > 15:
                raise RuntimeError(f'the product tipped {tilt:.0f} deg in the tote: no re-grasp from above')
            dyaw = math.atan2(Rrel[1, 0], Rrel[0, 0])
            cz, sz_ = math.cos(dyaw), math.sin(dyaw)
            d = np.array([tx, ty, zr]) - pr
            g = pn + np.array([cz * d[0] - sz_ * d[1], sz_ * d[0] + cz * d[1], d[2]])
            hig = g[2] + MP.TOTE_UP
            task.report('in the tote', f'slid {np.linalg.norm((pn - pr)[:2])*1000:.0f} mm, turned {math.degrees(dyaw):+.0f} deg, '
                        f'dropped {(pr - pn)[2]*1000:.0f} mm since release')
            yaws = [K.wrap(yt + dyaw)] + ([K.wrap(yt + dyaw + math.pi)] if S['grasp'] != 'side' else [])   # lying: never flip
            grab = None
            for yw in yaws:
                for el in (et, -et):
                    qh, qg = K.ik('left', g[0], g[1], hig, yw, P, elbow=el), K.ik('left', g[0], g[1], g[2], yw, P, elbow=el)
                    if qh and qg and not chk.check({**now(jopen), **qh}, env_held) and not chk.check({**now(jopen), **qg}, env_held):
                        grab = (yw, el, qh); break
                if grab: break
            if grab is None:
                raise RuntimeError('no clear hand-down re-grasp pose over the product in the tote')
            yg, eg, qh = grab
            go(qh, 'over the tote to re-grasp', jaw=0.0, env_=env_held)
            task.jaw('left', jopen + 0.01, 0.6)
            task.cart_move('left', (g[0], g[1], hig, yg, P), (g[0], g[1], g[2], yg, P), eg)
            task.jaw('left', 0.0, 1.0)
            q = task.robot.get_joint_positions()
            gap2 = (q[task.idx['left_finger_right']] + q[task.idx['left_finger_left']]) * 1000
            z0 = R.positions(watch)[name][2]
            task.cart_move('left', (g[0], g[1], g[2], yg, P), (g[0], g[1], hig, yg, P), eg)
            rise2 = (R.positions(watch)[name][2] - z0) * 1000
            task.report('picked from the tote', f'jaw gap {gap2:.1f} mm, lifted {rise2:.0f} mm',
                        ok=narrow * 500 < gap2 < narrow * 1000 + 8 and rise2 > 15)
            shot('re-grasped from the tote')
            # the pick's shelf moves backwards; set down 5 mm over the grasp height (the product drops onto the board)
            b5 = (b[0], b[1], b[2] + 0.005, b[3], b[4])
            if S['grasp'] == 'side':
                start = (ready[0], ready[1], out[2], out[3], out[4]) if ready is not a else out
                go(task.ik('left', start, e), 'swing back to the shelf', held, jaw_held, env_held)
                if ready is not a:
                    task.cart_move('left', start, out, e)
                task.cart_move('left', out, up, e, slow)
                task.cart_move('left', up, b5, e, slow)
            else:
                go(task.ik('left', up, e), 'swing back to the shelf', held, jaw_held, env_held)
                task.cart_move('left', up, b5, e, 1.5)
            task.jaw('left', jopen, 0.8)
            task.wait(0.4)
            shot('set down')
            if S['grasp'] == 'side':
                task.cart_move('left', b5, a, e, slow)
                if ready is not a:
                    task.cart_move('left', a, ready, e)
            else:
                task.cart_move('left', b5, up, e)
            task.jaw('left', 0.0, 0.5)
            go({**stow, 'lift': task.arm_now('left')['lift']}, 'fold after placing', env_=env)   # product back on its shelf
            task.joint_move('left', {'lift': stow['lift']})
            task.wait(1.0)
            pf, Rf = pose_b(cand[name])
            p_start = task.to_base(START[name][0][0])
            R_start = pose_b(None, START[name][0])[1]
            dxy = np.linalg.norm(pf[:2] - p_start[:2]) * 1000
            tl = math.degrees(math.acos(max(-1.0, min(1.0, (Rf @ R_start.T)[2, 2]))))
            dyw = math.degrees(math.atan2((Rf @ R_start.T)[1, 0], (Rf @ R_start.T)[0, 0]))
            task.report('back on its spot', f'{dxy:.0f} mm from where it stood, turned {dyw:+.0f} deg, tilted {tl:.1f} deg',
                        ok=dxy < 20 and tl < 5)
            moved = {n: d_ for n, d_ in R.knocked(watch, p0).items() if n != name}
            task.report('nothing else moved', ', '.join(f'{n} {d_*1000:.0f} mm' for n, d_ in moved.items()) or 'none over 10 mm', ok=not moved)
            shot('placed back')
            _hook('place_end', sid=sid, name=name)

        CTX.clear(); CTX.update(place=place_back, sid=sid, name=name)


def _qmat(q):
    w, x, y, z = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pose_b(path, pose=None):
    """(position, rotation matrix) of a body in the robot's base frame, now (or of a given world pose)."""
    from isaacsim.core.prims import SingleXFormPrim
    p, q = pose if pose is not None else SingleXFormPrim(path).get_world_pose()
    _, byaw_ = task.base_pose()
    c_, s_ = math.cos(-byaw_), math.sin(-byaw_)
    Rz_ = np.array([[c_, -s_, 0], [s_, c_, 0], [0, 0, 1]])
    return task.to_base(np.array(p, float)), Rz_ @ _qmat(q)



# ---------------------------------------------------------------- products back on their shelves
from pxr import PhysicsSchemaTools          # noqa: E402  (Isaac is running when this module is imported)
from omni.physx import get_physx_simulation_interface
import omni.usd


def restore():
    """Every product / filler that moved (picked into the tote, knocked) back to where it stood at the start: a USD
    pose write = a PhysX teleport; sleep + wake zeroes its velocities. Returns how many went back."""
    sim = get_physx_simulation_interface()
    n_back = 0
    for n, bs in MOVABLE.items():
        p_now, q_now = bs[0].get_world_pose()
        p0, q0 = START[n][0]
        if np.linalg.norm(np.array(p_now) - p0) < 0.003 and abs(abs(float(np.dot(q_now, q0))) - 1) < 1e-4:
            continue
        for b_, (p, q) in zip(bs, START[n]):
            b_.set_world_pose(position=p, orientation=q)
            bid = PhysicsSchemaTools.sdfPathToInt(b_.prim_path)
            sid_ = omni.usd.get_context().get_stage_id()
            sim.put_to_sleep(sid_, bid); sim.wake_up(sid_, bid)
        n_back += 1
    if n_back:
        CTX.clear()                                   # the product in the tote went back by teleport
        task.wait(0.3)
    return n_back


def reset_robot():
    """Teleport the robot home with the normal travel fold (after an aborted pick left the arm somewhere)."""
    task.set_pose({'lift': MP.STOW['lift'], **{f'left_{k}': v for k, v in MP.STOW.items() if k != 'lift'},
                   **R._home_joints(task), 'scene_camera_pitch': math.radians(OPT['tilt'])})
    R._place_home(task)
    ROBOT.update(at='home', stow=MP.STOW)
    task.wait(0.5)


def run(sid):
    """restore the shelves, then one pick; returns (ok, text)"""
    n0 = len(task.results)
    back = restore()
    if back:
        print(f'      {back} product(s) put back where they started', flush=True)
    try:
        pick(sid)
    except RuntimeError as err:
        task.report('aborted', str(err), ok=False)
    res = [r_ for r_ in task.results[n0:] if not r_[0].startswith('  via')]   # a midway drive stop doesn't decide the pick
    ok = bool(res) and all(o for _, o, _ in res)
    bad = [f'{st_}: {t}' for st_, o, t in res if not o]
    return ok, ('OK' if ok else bad[0] if bad else 'no result')


def run_place():
    """put the last picked product back from the tote; returns (ok, text)"""
    if not CTX:
        return False, 'nothing to place back (pick a product first)'
    n0 = len(task.results)
    fn = CTX['place']; CTX.clear()
    try:
        fn()
    except RuntimeError as err:
        task.report('aborted', str(err), ok=False)
    res = [r_ for r_ in task.results[n0:] if not r_[0].startswith('  via')]
    ok = bool(res) and all(o for _, o, _ in res)
    bad = [f'{st_}: {t}' for st_, o, t in res if not o]
    return ok, ('OK' if ok else bad[0] if bad else 'no result')


# ---------------------------------------------------------------- what can be picked
# Stations where the whole pick + place passes (headless, all 19 in one session with put-back between, 2026-11-19); --all_stations
# offers every station. Impossible here: see CLAUDE.md item 29.
PICKABLE = {'rack_b0_boxedCereal_l01', 'rack_b0_box_cracker_a01', 'rack_b0_boxedFood_b01', 'rack_b0_boxedFood_c01',
            'rack_b0_boxedFood_d01', 'shelf_b1_milk_b01', 'shelf_b2_milk_c01', 'rack2_b0_chocolate_e01',
            'shelf2_b1_ketchup_c01', 'shelf2_b1_ketchup_i01', 'shelf2_b2_ketchup_a02', 'shelf2_b2_ketchup_c01',
            'table_cannedFood_h01', 'table_coffeeCup_b01', 'table_yogurt_a01', 'table_yogurt_c01'}
# picked fine, but the place back fails: the round bottle pivots about the jaw axis as the jaws open in the tote and
# ends leaning 51 deg (P2T_DEBUG=1 traces it), so it can't be re-grasped from above
PICK_ONLY = {'shelf_b2_milk_c01'}
# not pickable: shelf_b1_yogurt_a01 (forearm vs the board above), shelf_b2_yogurt_c01 (no IK for the grasp from that
# station), rack2_b0_cannedFood_h01 (packed run, no room for the open jaws)
NAMES = {p['package']: p['name'] for p in SC.PRODUCTS.values()}


def label(sid):
    s = st[sid]
    where = 'table' if sid.startswith('table') else sid.split('_')[0] + ' ' + sid.split('_')[1].replace('b', 'board ')
    return f"{NAMES.get(s['product'], s['product'])}  ({where})" + ('  - pick only' if sid in PICK_ONLY else '')


