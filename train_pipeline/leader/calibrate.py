"""Calibrate the leader arm: its zero POSE, which way each joint turns, its ranges and the gripper's opening.

    python train_pipeline/leader/calibrate.py --port /dev/serial/by-id/usb-1a86_..._-if00     # or LEADER_PORT=...
    ... --name leader_mono        calibration file name (default; prototype later: leader_left / leader_right)
    ... --dummy                   no hardware: a simulated leader moves by itself (to try the screens; never saves)

Every step shows a live table of all 6 joints (like lerobot-calibrate's MIN | POS | MAX table), redrawn in place
until you press Enter. Writes train_pipeline/leader/calibration/<name>.json (commit it: a calibration belongs to the
arm, not to the PC). Runs in env_isaaclab or lerobot_env (needs only feetech-servo-sdk). Every servo's torque is off:
move the arm by hand.

Why not lerobot-calibrate: it puts each joint's zero in the middle of the range you happen to sweep, and gives the
wrist roll a full turn (the Store-Picker-Bot needed calibrate_wrist_roll.py for exactly that). Here the zero is the
follower's own zero pose, so the angles copy 1:1 and the encoder seam ends up half a turn from it: behind J1's end
stop (+-174), out of reach of the other joints, and the roll (no stop) is unwrapped in software (leader_arm.py).
Do NOT run lerobot-calibrate on this leader afterwards: it rewrites the servos' homing offsets (leader_arm.py notices
and asks for a new calibration).
"""
import argparse, collections, json, math, os, select, sys, time
from datetime import date
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import leader_arm as LA                                                   # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--port', default=os.environ.get('LEADER_PORT'))
ap.add_argument('--name', default=LA.DEFAULT_NAME)
ap.add_argument('--dummy', action='store_true', help='simulated leader, no hardware (nothing is saved)')
args = ap.parse_args()
if not args.port and not args.dummy:
    ap.error('give --port (ls -l /dev/serial/by-id/) or set LEADER_PORT (or --dummy to try it without the arm)')

ZERO_POSE = """ZERO POSE (the follower's zero: everything straight, as in the leader preview's first panel)
  - link 1 straight ahead, pointing AWAY from the stop rib (the rib is behind, at the dead end)
  - link 2 and the wrist in one straight line with link 1
  - gripper level (pointing straight out, not tipped up or down)
  - roll 0: the two jaw blades side by side, left and right (not one above the other)
  - finger loops closed (jaws touching)"""

# joint -> what to do for its POSITIVE direction (the follower URDF's: pans counter-clockwise from above,
# pitch = tip down, roll = +X right-hand rule, gripper = open)
POSITIVE = {
    'J1': 'turn the WHOLE ARM to the LEFT (counter-clockwise seen from above), about 45 deg',
    'J2': 'bend link 2 to the LEFT at the elbow (counter-clockwise seen from above), about 45 deg',
    'J3': 'turn the wrist to the LEFT (counter-clockwise seen from above), about 45 deg',
    'wrist_pitch': 'tip the gripper DOWN, about 45 deg',
    'wrist_roll': 'turn the gripper CLOCKWISE as you see it from BEHIND (looking along the arm), about 45 deg',
    'gripper': 'open the finger loops FULLY and hold them open',
}
MIN_MOVE = 15.0        # deg a joint must move in step 2
OTHER_MOVE = 10.0      # deg: another joint moving this much in step 2 gets a warning
STILL_TICKS = 8        # step 1: a joint is 'still' if its last 0.5 s of readings span <= this (8 ticks = 0.7 deg)
HZ = 20                # table refresh


# ------------------------------------------------------------------------------------------ live table
def live_table(render):
    """Redraw render() -> [lines] in place until Enter (ANSI cursor-up, like lerobot's record_ranges_of_motion)."""
    shown = 0
    while True:
        lines = render()
        if shown:
            sys.stdout.write(f'\033[{shown}F')                      # back to the first line of the last frame
        sys.stdout.write(''.join('\033[2K' + ln + '\n' for ln in lines))
        sys.stdout.flush()
        shown = len(lines)
        if select.select([sys.stdin], [], [], 1.0 / HZ)[0]:
            if sys.stdin.readline() == '':                         # stdin closed (Ctrl+D / a pipe ran out)
                raise SystemExit('\ninput closed: stopped, nothing saved')
            return


def table(cols, rows, footer=()):
    """cols: [(title, width, align)]; rows: [[cell, ...]] (str). Returns the lines."""
    fmt = lambda cells: ' | '.join(f'{c:{a}{w}}' for c, (_, w, a) in zip(cells, cols))
    head = fmt([t for t, _, _ in cols])
    return [head, '-' * len(head)] + [fmt(r) for r in rows] + [''] + list(footer)


def lim_txt(j, lim):
    return '0 .. 90 mm' if lim is None else f'{lim[0]:+.0f} .. {lim[1]:+.0f}'


# ------------------------------------------------------------------------------------------- dummy bus
class DummyBus:
    """A leader that moves by itself: every joint swings slowly around a made-up zero (for --dummy only)."""
    def __init__(self):
        self.cal = LA.dummy_calibration(); self.ids = [j[1] for j in LA.JOINTS]; self.t0 = time.time()

    def homing_offsets(self):
        return {i: 0 for i in self.ids}

    def read_raw(self):
        t = time.time() - self.t0
        deg = {}
        for k, (j, _, _, lim) in enumerate(LA.JOINTS):
            amp = 20.0 if lim is None else 0.8 * min(abs(lim[0]), abs(lim[1]))
            deg[j] = (amp * (1 + math.sin(0.5 * t + k))) if lim is None else amp * math.sin((0.3 + 0.07 * k) * t + k)
        return LA.to_raw(self.cal, deg)

    def read_avg(self, n=15):
        return {i: float(v) for i, v in self.read_raw().items()}

    def close(self):
        pass


# ---------------------------------------------------------------------------------------------- steps
def deg_from(raw, zero):
    return LA.wrap180((raw - zero) * 360.0 / LA.TICKS)


def step_zero(bus):
    """Live RAW + stillness while the user holds the zero pose."""
    hist = {sid: collections.deque(maxlen=HZ // 2) for sid in bus.ids}
    cols = [('JOINT', 12, '<'), ('ID', 2, '>'), ('RAW', 5, '>'), ('SPREAD', 6, '>'), ('STATE', 10, '<')]

    def render():
        raw = bus.read_raw()
        rows = []
        for j, sid, _, _ in LA.JOINTS:
            h = hist[sid]; h.append(raw[sid])
            spread = max(deg_from(v, h[0]) for v in h) - min(deg_from(v, h[0]) for v in h)   # seam-safe, in deg
            ticks = spread * LA.TICKS / 360.0
            rows.append([j, str(sid), str(raw[sid]), f'{ticks:.0f}', 'still' if ticks <= STILL_TICKS else 'MOVING'])
        return table(cols, rows, ['  Hold the zero pose still (every joint "still"), then press Enter.'])
    live_table(render)


def step_direction(bus, zero_raw, j, sid):
    """Live angle of every joint from the zero pose; the joint being taught is marked >."""
    cols = [('', 1, '<'), ('JOINT', 12, '<'), ('ID', 2, '>'), ('POS', 7, '>'), ('NOTE', 36, '<')]

    def render():
        raw = bus.read_raw()
        rows = []
        for jj, s, _, _ in LA.JOINTS:
            d = deg_from(raw[s], zero_raw[s])
            if jj == j:
                note = 'ok: press Enter' if abs(d) >= MIN_MOVE else f'move it (at least {MIN_MOVE:.0f} deg)'
            else:
                note = 'keep this one still!' if abs(d) > OTHER_MOVE else ''
            rows.append(['>' if jj == j else ' ', jj, str(s), f'{d:+7.1f}', note])
        return table(cols, rows, [f'  {j} (servo {sid}): {POSITIVE[j]}, then Enter.'])
    live_table(render)


def step_ranges(bus, cal):
    """Live MIN | POS | MAX of every joint in calibrated degrees, with the follower's limits beside them."""
    L = LA.Leader(bus=bus, calib=cal)
    lo, hi = {}, {}
    cols = [('JOINT', 12, '<'), ('ID', 2, '>'), ('MIN', 7, '>'), ('POS', 7, '>'), ('MAX', 7, '>'),
            ('FOLLOWER', 13, '>'), ('NOTE', 30, '<')]

    def notes(j, lim):
        if j == 'J1':
            if max(abs(lo[j]), abs(hi[j])) > 176:
                return '!! past the +-174 stop'
            return 'ok: both stops' if min(abs(lo[j]), abs(hi[j])) >= 165 else 'reach both end stops (~+-174)'
        if j == 'gripper':
            return 'ok' if lo[j] <= 2 and hi[j] >= 0.9 * cal['joints'][j]['open_deg'] else 'close and open fully'
        return 'ok' if lo[j] <= lim[0] and hi[j] >= lim[1] else 'short of the follower (info)'

    def render():
        d = L.degrees()
        rows = []
        for j, sid, _, lim in LA.JOINTS:
            lo[j] = min(lo.get(j, d[j]), d[j]); hi[j] = max(hi.get(j, d[j]), d[j])
            rows.append([j, str(sid), f'{lo[j]:+7.1f}', f'{d[j]:+7.1f}', f'{hi[j]:+7.1f}', lim_txt(j, lim), notes(j, lim)])
        return table(cols, rows, [
            '  Move every joint slowly to both ends of its travel (J1 against both end stops, the roll to +-90 or more,',
            '  the loops fully closed and open). Press Enter when done.',
            '  (a leader range shorter or wider than the follower\'s is fine: commands are clamped to the follower limits)'])
    live_table(render)
    bad = max(abs(lo['J1']), abs(hi['J1'])) > 176
    return lo, hi, bad


def step_check(bus, cal):
    """Live calibrated angle and the follower command it makes ('*' = clamped at the follower's limit)."""
    L = LA.Leader(bus=bus, calib=cal)
    cols = [('JOINT', 12, '<'), ('ID', 2, '>'), ('LEADER', 8, '>'), ('FOLLOWER CMD', 12, '>'), ('LIMIT', 13, '>')]
    n, t0 = [0], time.time()

    def render():
        d = L.degrees(); c = L.command(d)
        rows = []
        for j, sid, name, lim in LA.JOINTS:
            if j == 'gripper':
                rows.append([j, str(sid), f'{d[j]:+7.1f}d', f'{c[name] * 1000:6.0f} mm', lim_txt(j, lim)])
            else:
                star = '*' if not lim[0] <= d[j] <= lim[1] else ' '
                rows.append([j, str(sid), f'{d[j]:+8.1f}', f'{math.degrees(c[name]):+7.1f}{star}', lim_txt(j, lim)])
        n[0] += 1
        return table(cols, rows, [
            '  The zero pose must read ~0 everywhere; every joint must turn the way you moved it in step 2.',
            '  (* = the follower stops at its limit there)' +
            (f'     servo read + redraw {n[0] / (time.time() - t0):3.0f} Hz' if n[0] > 10 else '') + '.  Press Enter.'])
    live_table(render)


# ------------------------------------------------------------------------------------------------ main
bus = DummyBus() if args.dummy else LA.Bus(args.port)
homing = bus.homing_offsets()
print(f'connected: {len(bus.ids)} servos on {"(dummy)" if args.dummy else args.port}, torque OFF (hold the arm). '
      f'Homing offsets: {homing}')
if any(homing.values()):
    print('  (non-zero homing offsets from an earlier lerobot-calibrate: fine, they are stored and checked later)')

try:
    print('\n1/4  ' + ZERO_POSE + '\n')
    step_zero(bus)
    zero_raw = bus.read_avg(30)
    cal = {'name': args.name, 'date': str(date.today()), 'port': args.port, 'homing_offsets': homing, 'joints': {}}
    for j, sid, _, _ in LA.JOINTS:
        cal['joints'][j] = {'id': sid, 'zero': round(zero_raw[sid], 1), 'sign': 1}

    print('\n2/4  Directions. For each joint, start from the zero pose, move ONLY that joint as asked, press Enter.\n')
    for j, sid, _, _ in LA.JOINTS:
        while True:
            step_direction(bus, zero_raw, j, sid)
            d = deg_from(bus.read_avg(10)[sid], zero_raw[sid])
            if abs(d) >= MIN_MOVE:
                break
            print(f'    it moved only {d:+.1f} deg: move it further (at least {MIN_MOVE:.0f} deg)\n')
        cal['joints'][j]['sign'] = 1 if d > 0 else -1
        if j == 'gripper':
            cal['joints'][j]['open_deg'] = round(abs(d), 1)
            print(f'    {j}: sign {cal["joints"][j]["sign"]:+d}, fully open = {abs(d):.1f} deg (design: 40)')
            if not 25 <= abs(d) <= 50:
                print('    !! far from the design 40 deg: were the loops fully open / closed in the zero pose?')
        else:
            print(f'    {j}: sign {cal["joints"][j]["sign"]:+d} ({d:+.1f} deg). Bring it back to zero.\n')

    print('\n3/4  Ranges.\n')
    lo, hi, bad = step_ranges(bus, cal)
    cal['ranges_deg'] = {j: [round(lo[j], 1), round(hi[j], 1)] for j in lo}
    if bad:
        print('  !! J1 went past the +-174 end stop: the zero pose was not straight ahead. Calibrate again.')

    print('\n4/4  Check.\n')
    step_check(bus, cal)
finally:
    bus.close()

if args.dummy:
    sys.exit('dummy run: not saved')
if bad:
    sys.exit('not saved (see the !! above)')
ans = input(f'Save to {LA.calib_path(args.name)}? [y/N] ').strip().lower()
if ans == 'y':
    LA.CALIB_DIR.mkdir(exist_ok=True)
    LA.calib_path(args.name).write_text(json.dumps(cal, indent=1) + '\n')
    print('saved. Check it any time with: python train_pipeline/leader/check_leader.py')
else:
    print('not saved')
