"""Run a trained policy in the Isaac store scene and score it (step 5 of the pipeline).

    conda activate env_isaaclab           # run from the repo root
    OMNI_KIT_ACCEPT_EULA=YES python train_pipeline/eval/eval_sim.py \\
        --checkpoint train_pipeline/outputs/<job>/checkpoints/last/pretrained_model --station shelf_b1_milk_b01 --episodes 10
    ... --n_action_steps 30 | --ensemble 0.01    how the ACT chunk is executed (default: as trained)
    ... --window                                 watch it
    ... --seed 100                               the randomisation (use another seed than the recordings)

Each test starts like a recorded episode: products put back, the same kind of randomisation as the recorder (robot stop,
product, light), the planner drives to the station; the moment the robot is parked (where recordings start) the
policy takes over. Every 1/30 s: measured state + both camera images -> policy server (eval/policy_server.py in
lerobot_env, started here) -> the 7-value command. Stops when the product has been in the tote for 3 s, or after
--max_s. Scores: grasped (product lifted > 15 mm), in the tote, nothing else knocked over. Contact sheets of every test
go to --out.
"""
import argparse, math, os, pickle, random, socket, struct, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import common as C                                                       # noqa: E402
sys.path.insert(0, str(C.REPO / 'scripts' / 'common')); import repo_paths as RP   # noqa: E402,F401

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--checkpoint', required=True)
ap.add_argument('--station', default='shelf_b1_milk_b01')
ap.add_argument('--episodes', type=int, default=10)
ap.add_argument('--seed', type=int, default=100)
ap.add_argument('--no_jitter', action='store_true')
ap.add_argument('--base_xy', type=float, default=0.02)
ap.add_argument('--base_yaw', type=float, default=3.0)
ap.add_argument('--product_xy', type=float, default=0.01)
ap.add_argument('--product_yaw', type=float, default=8.0)
ap.add_argument('--light', type=float, default=0.25)
ap.add_argument('--tilt', type=float, default=40.0)
ap.add_argument('--max_s', type=float, default=90.0)
ap.add_argument('--n_action_steps', type=int, default=None)
ap.add_argument('--ensemble', type=float, default=None)
ap.add_argument('--port', type=int, default=9988)
ap.add_argument('--out', default=None, help='folder for contact sheets (default: next to the checkpoint run)')
ap.add_argument('--window', action='store_true')
args = ap.parse_args()
ck = os.path.abspath(args.checkpoint)
if not os.path.isfile(os.path.join(ck, 'config.json')):
    sys.exit(f'{ck}: not a pretrained_model folder (no config.json)')

# ---------------------------------------------------------------- policy server (lerobot_env)
cmd = ['conda', 'run', '--no-capture-output', '-n', 'lerobot_env', 'python', os.path.join(HERE, 'policy_server.py'),
       '--checkpoint', ck, '--port', str(args.port)]
if args.n_action_steps:
    cmd += ['--n_action_steps', str(args.n_action_steps)]
if args.ensemble is not None:
    cmd += ['--ensemble', str(args.ensemble)]
server = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
for line in server.stdout:
    print('  [server]', line.rstrip(), flush=True)
    if line.startswith('READY'):
        break
else:
    sys.exit('the policy server did not start')
import threading                                                         # noqa: E402
threading.Thread(target=lambda: [print('  [server]', l.rstrip(), flush=True) for l in server.stdout], daemon=True).start()

import stations_mono as ST                                                    # noqa: E402
import scene_robot as R                                                  # noqa: E402
st = ST.load()
if args.station not in st:
    sys.exit(f'unknown station {args.station}')
app = R.start_app(not args.window)
import numpy as np                                                       # noqa: E402
from PIL import Image, ImageDraw                                          # noqa: E402
import cameras as CAM                                                    # noqa: E402
import pick_place_mono as PP                                                  # noqa: E402
import sim_common as SCM                                                 # noqa: E402

task, items, watch = R.load(app, not args.window)
PP.init(task, items, watch, st, tilt=args.tilt, headless=not args.window, out=None)
SCM.bind(task, items, PP)
EVERY = round(60 / C.FPS)
out = args.out or os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(ck))), f'eval_{time.strftime("%m%d_%H%M")}')
os.makedirs(out, exist_ok=True)
rng = random.Random(args.seed)

sock = socket.create_connection(('127.0.0.1', args.port))


def ask(obs):
    data = pickle.dumps(obs, protocol=4)
    sock.sendall(struct.pack('>I', len(data)) + data)
    head = b''
    while len(head) < 4:
        head += sock.recv(4 - len(head))
    n = struct.unpack('>I', head)[0]; buf = b''
    while len(buf) < n:
        buf += sock.recv(n - len(buf))
    return np.asarray(pickle.loads(buf)['action'], np.float32)


class Handover(Exception):
    pass


def handover(sid, name):
    raise Handover(name)


EP = {}
PP.HOOKS.update(pick_chosen=lambda sid, name: SCM.jitter_product(name, *EP['product']) if EP.get('product') else None,
                pick_start=handover)

results = []
for e in range(args.episodes):
    sid = args.station; jit = not args.no_jitter
    light = 1 + rng.uniform(-args.light, args.light) if jit else 1.0
    bx, by = (rng.uniform(-args.base_xy, args.base_xy) for _ in range(2)) if jit else (0.0, 0.0)
    byaw = rng.uniform(-args.base_yaw, args.base_yaw) if jit else 0.0
    EP['product'] = ((rng.uniform(-args.product_xy, args.product_xy), rng.uniform(-args.product_xy, args.product_xy),
                      math.radians(rng.uniform(-args.product_yaw, args.product_yaw))) if jit else None)
    SCM.set_light(light)
    PP.restore()
    S0 = st[sid]; st[sid] = {**S0, 'x': S0['x'] + bx, 'y': S0['y'] + by, 'yaw': S0['yaw'] + byaw}
    name = None
    try:
        PP.pick(sid)                                      # drives there; the 'pick_start' hook hands over when parked
    except Handover as h:
        name = h.args[0]
    finally:
        st[sid] = S0
    if name is None:
        sys.exit('the planner finished without handing over (pick_start hook not reached)')
    prod = PP.NAMES.get(items[name]['pkg'], name)
    text = C.task_text('pick', prod, 'table' if sid.startswith('table') else 'shelf')
    wt = dict(PP.watch); p0 = R.positions(wt); z0 = p0[name][2]
    print(f'\n=== test {e + 1}/{args.episodes}: {sid}, "{text}" (stop {bx*100:+.1f}/{by*100:+.1f} cm {byaw:+.1f} deg, '
          f'light x{light:.2f}); policy in control', flush=True)
    t0 = task.t; rise = 0.0; tote_since = None; sheet = []; k = 0
    while task.t - t0 < args.max_s:
        fr = CAM.capture(task, task.cams)
        imgs = {key: (*fr[f'{cam}_rgb'].shape[:2], fr[f'{cam}_rgb'].tobytes()) for key, cam in C.CAMERAS.items()}
        a = ask({'state': [float(x) for x in SCM.state()], 'images': imgs, 'task': text, 'reset': k == 0})   # plain types
        SCM.command(a)
        for _ in range(EVERY):
            task.step_once()
        if k % (3 * C.FPS) == 0:
            sheet.append((task.t - t0, fr['scene_rgb'], fr['left_wrist_rgb'], float(a[-1])))
        k += 1
        rise = max(rise, R.positions({name: wt[name]})[name][2] - z0)
        inside, _ = SCM.in_tote(name)
        tote_since = (tote_since if tote_since is not None else task.t) if inside else None
        if tote_since is not None and task.t - tote_since > 3.0:
            break
    task.wait(1.0)
    inside, pb = SCM.in_tote(name)
    moved = {n: d for n, d in R.knocked(wt, p0).items() if n != name}
    res = dict(grasped=rise > 0.015, in_tote=inside, clean=not moved, time=task.t - t0)
    res['success'] = res['in_tote'] and res['clean']
    results.append(res)
    print(f'    {"SUCCESS" if res["success"] else "fail"}: lifted {rise*1000:.0f} mm, product at ({pb[0]:+.3f}, {pb[1]:+.3f}, '
          f'{pb[2]:.3f}) in the base frame ({"in the tote" if inside else "NOT in the tote"}), '
          f'{"nothing knocked" if not moved else "knocked: " + ", ".join(moved)}, {res["time"]:.0f} s', flush=True)
    # contact sheet: scene over wrist, one column every 3 s
    W = 256; cols = sheet[:12]
    im = Image.new('RGB', (W * len(cols), 30 + 160 + 144 + 16), (255, 255, 255)); d = ImageDraw.Draw(im)
    d.text((6, 8), f'test {e}: {"SUCCESS" if res["success"] else "fail"}  lifted {rise*1000:.0f} mm  {text}', fill=(0, 0, 0))
    for i, (t, sc, wr, g) in enumerate(cols):
        im.paste(Image.fromarray(sc).resize((W, 160)), (W * i, 30)); im.paste(Image.fromarray(wr).resize((W, 144)), (W * i, 190))
        d.text((W * i + 4, 336), f't {t:.0f} s  jaw cmd {g * 1000:.0f} mm', fill=(0, 0, 0))
    im.save(os.path.join(out, f'test_{e:02d}.png'))
    PP.reset_robot()                                      # teleport home with the normal fold for the next test

n = len(results)
print(f'\n{args.station}: success {sum(r["success"] for r in results)}/{n}, grasped {sum(r["grasped"] for r in results)}/{n}, '
      f'in the tote {sum(r["in_tote"] for r in results)}/{n}, nothing knocked {sum(r["clean"] for r in results)}/{n}; '
      f'contact sheets in {out}', flush=True)
sock.close()
server.wait(timeout=30)
PP.close()
app.close()
