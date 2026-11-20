"""The leader arm: read its 6 STS3215 servos and turn them into follower commands (rad, m).

Plain Python + scservo_sdk (feetech-servo-sdk), so it runs in BOTH envs: env_isaaclab (teleop in the sim) and
lerobot_env. No LeRobot code: the leader's zero is a POSE (calibrate.py), not LeRobot's "middle of the range".

Servo IDs (the SO-100 leader's order, which this arm was assembled in):
    1 J1 shoulder (turns the follower's ring)      4 wrist pitch
    2 J2 elbow    (follower 'shoulder.pos')        5 wrist roll
    3 J3 wrist pan (follower 'elbow.pos')          6 gripper (finger-loop jaw)
The follower's mono joint names are shifted by one (CLAUDE.md item 23): leader J1 -> ring, J2 -> shoulder, J3 -> elbow.

Angles: angle = sign * (raw - zero) in degrees, wrapped to +-180 around the zero pose. So the encoder seam (+180 -> -180) is never
inside a joint's travel: J1 has its end stop at +-174 (README), the other joints can't reach +-180, and the roll,
which has no stop, is UNWRAPPED (each reading taken as the nearest turn to the previous one) so it keeps counting past
+-180 instead of jumping. Commands are clamped to the follower's URDF limits.

    python train_pipeline/leader/leader_arm.py --selftest          # the maths, no hardware
"""
import json, math, os, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CALIB_DIR = HERE / 'calibration'
DEFAULT_NAME = 'leader_mono'
BAUD = 1_000_000
TICKS = 4096

# leader joint, servo id, follower name (train_pipeline/common.py), follower limits (URDF prototype_mono_arm)
JOINTS = [
    ('J1',          1, 'ring.pos',        (-180.0, 180.0)),
    ('J2',          2, 'shoulder.pos',    (-115.0, 115.0)),
    ('J3',          3, 'elbow.pos',       (-122.0, 122.0)),
    ('wrist_pitch', 4, 'wrist_pitch.pos', (-15.0, 115.0)),
    ('wrist_roll',  5, 'wrist_roll.pos',  (-120.0, 120.0)),
    ('gripper',     6, 'gripper.pos',     None),
]
UNWRAP = ('wrist_roll',)      # the only joint with no end stop (README "Limits only software can set"): it may pass +-180
GRIPPER_MAX_M = 0.09          # follower jaw opening (both fingers) at the leader's fully open loops

# STS3215 control table (the same addresses LeRobot uses)
ADDR_HOMING = 31              # Homing_Offset, 2 bytes, sign bit 11 (EEPROM: written by lerobot-calibrate)
ADDR_TORQUE = 40
ADDR_POS = 56                 # Present_Position, 2 bytes, sign bit 15


def _decode(v, bit):
    return -(v & ((1 << bit) - 1)) if v & (1 << bit) else v


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


class Bus:
    """The six servos on one serial port (torque always off: the leader is moved by hand)."""

    def __init__(self, port, ids=tuple(j[1] for j in JOINTS), baud=BAUD):
        import scservo_sdk as scs
        self.scs, self.ids = scs, list(ids)
        self.port = scs.PortHandler(port)
        self.pkt = scs.PacketHandler(0)
        if not self.port.openPort():
            raise SystemExit(f'cannot open {port} (plugged in? permission: sudo chmod 666 {port}, or add yourself to dialout)')
        self.port.setBaudRate(baud)
        missing = [i for i in self.ids if self.pkt.ping(self.port, i)[1] != scs.COMM_SUCCESS]
        if missing:
            raise SystemExit(f'no answer from servo id(s) {missing} on {port} at {baud} baud: check power, cables, '
                             f'and the ids (setup_ids.py)')
        self.reader = scs.GroupSyncRead(self.port, self.pkt, ADDR_POS, 2)
        for i in self.ids:
            self.reader.addParam(i)
        self.torque_off()

    def torque_off(self):
        for i in self.ids:
            self.pkt.write1ByteTxRx(self.port, i, ADDR_TORQUE, 0)

    def homing_offsets(self):
        return {i: _decode(self.pkt.read2ByteTxRx(self.port, i, ADDR_HOMING)[0], 11) for i in self.ids}

    def read_raw(self, tries=3):
        """{id: position 0..4095}"""
        for _ in range(tries):
            if self.reader.txRxPacket() == self.scs.COMM_SUCCESS:
                return {i: _decode(self.reader.getData(i, ADDR_POS, 2), 15) % TICKS for i in self.ids}
        raise IOError('servo read failed 3 times in a row')

    def read_avg(self, n=15):
        """Average of n readings (circular, so a pose right at the seam averages correctly)."""
        acc = {i: [0.0, 0.0] for i in self.ids}
        for _ in range(n):
            for i, r in self.read_raw().items():
                a = r * 2 * math.pi / TICKS
                acc[i][0] += math.cos(a); acc[i][1] += math.sin(a)
            time.sleep(0.01)
        return {i: (math.atan2(s, c) * TICKS / (2 * math.pi)) % TICKS for i, (c, s) in acc.items()}

    def close(self):
        self.port.closePort()


def calib_path(name=DEFAULT_NAME):
    return CALIB_DIR / f'{name}.json'


def load_calibration(name=DEFAULT_NAME):
    p = calib_path(name)
    if not p.is_file():
        raise SystemExit(f'{p}: no calibration yet: run train_pipeline/leader/calibrate.py first')
    return json.loads(p.read_text())


class Leader:
    """read() -> {follower name: command} in SI units (rad, gripper m), clamped to the follower's limits."""

    def __init__(self, port=None, name=DEFAULT_NAME, bus=None, calib=None):
        self.cal = calib or load_calibration(name)
        port = port or os.environ.get('LEADER_PORT') or self.cal.get('port')
        self.bus = bus or Bus(port)
        if bus is None:
            now = self.bus.homing_offsets()
            was = {int(k): v for k, v in self.cal['homing_offsets'].items()}
            changed = [i for i in now if now[i] != was.get(i)]
            if changed:
                raise SystemExit(f'servo homing offsets changed since calibration (ids {changed}): was lerobot-calibrate '
                                 f'run on this leader? Calibrate again with calibrate.py.')
        self.last = {}

    def degrees(self, raw=None):
        """Leader joint angles in degrees (gripper: degrees open), unwrapped across calls."""
        raw = raw if raw is not None else self.bus.read_raw()
        out = {}
        for j, sid, _, _ in JOINTS:
            c = self.cal['joints'][j]
            d = c['sign'] * wrap180((raw[sid] - c['zero']) * 360.0 / TICKS)
            if j in UNWRAP and j in self.last:
                d = self.last[j] + wrap180(d - self.last[j])
            self.last[j] = out[j] = d
        return out

    def command(self, deg):
        """Leader degrees -> follower command {name: rad | m}, clamped."""
        cmd = {}
        for j, _, name, lim in JOINTS:
            if j == 'gripper':
                f = deg[j] / self.cal['joints']['gripper']['open_deg']
                cmd[name] = min(max(f, 0.0), 1.0) * GRIPPER_MAX_M
            else:
                cmd[name] = math.radians(min(max(deg[j], lim[0]), lim[1]))
        return cmd

    def read(self):
        return self.command(self.degrees())

    def close(self):
        self.bus.close()


# ------------------------------------------------------------------------------- dummy leader (no hardware)
def dummy_calibration():
    """A calibration that makes the maths work hard: zeros next to the encoder seam, two servos counting backwards."""
    zero = {1: 4050, 2: 30, 3: 2000, 4: 1000, 5: 3500, 6: 2500}
    sign = {1: 1, 2: -1, 3: 1, 4: 1, 5: -1, 6: 1}
    cal = {'name': 'dummy', 'date': 'dummy', 'port': 'dummy', 'homing_offsets': {},
           'joints': {j: {'id': s, 'zero': zero[s], 'sign': sign[s]} for j, s, _, _ in JOINTS}}
    cal['joints']['gripper']['open_deg'] = 40.0
    return cal


def to_raw(cal, deg):
    """Leader degrees -> raw servo ticks (the inverse of Leader.degrees)."""
    out = {}
    for j, sid, _, _ in JOINTS:
        c = cal['joints'][j]
        out[sid] = round(c['zero'] + c['sign'] * deg[j] * TICKS / 360.0) % TICKS
    return out


class ReplayBus:
    """Dummy leader: plays a recorded episode's commands back as raw servo readings, as if a person moved the leader the
    same way. actions: (n, 7) in train_pipeline/common.py order (lift first); clock(): seconds since the replay started.
    lift(): the episode's lift (the real leader has no lift: the keys / the lift controller set it)."""

    def __init__(self, actions, fps, cal, clock, jitter_ticks=1, seed=0):
        import numpy as np
        self.a, self.fps, self.cal, self.clock = np.asarray(actions, float), fps, cal, clock
        self.rng = np.random.default_rng(seed)
        self.jitter = jitter_ticks
        self.ids = [j[1] for j in JOINTS]

    def duration(self):
        return (len(self.a) - 1) / self.fps

    def _row(self):
        x = min(max(self.clock() * self.fps, 0.0), len(self.a) - 1.0)
        i = int(x); f = x - i
        return self.a[i] + (self.a[min(i + 1, len(self.a) - 1)] - self.a[i]) * f

    def lift(self):
        return float(self._row()[0])

    def read_raw(self):
        v = self._row()
        deg = {}
        for k, (j, _, _, _) in enumerate(JOINTS):          # common.JOINTS[1:] in the same order as JOINTS here
            x = v[k + 1]
            deg[j] = x / GRIPPER_MAX_M * self.cal['joints']['gripper']['open_deg'] if j == 'gripper' else math.degrees(x)
        raw = to_raw(self.cal, deg)
        return {i: (r + int(self.rng.integers(-self.jitter, self.jitter + 1))) % TICKS for i, r in raw.items()}

    def close(self):
        pass


# ------------------------------------------------------------------------------------------- self-test
class FakeBus:
    def __init__(self):
        self.raw = {j[1]: 0 for j in JOINTS}

    def read_raw(self):
        return dict(self.raw)

    def close(self):
        pass


def selftest():
    zero = {1: 4000, 2: 100, 3: 2048, 4: 1000, 5: 50, 6: 3000}        # zeros near the seam on purpose
    cal = {'joints': {j: {'zero': zero[s], 'sign': (-1 if s in (2, 5) else 1)} for j, s, _, _ in JOINTS},
           'homing_offsets': {}}
    cal['joints']['gripper']['open_deg'] = 40.0
    fb = FakeBus()
    L = Leader(bus=fb, calib=cal)
    t = lambda d: round(d * TICKS / 360.0)

    def at(**deg):
        for j, s, _, _ in JOINTS:
            c = cal['joints'][j]
            fb.raw[s] = (c['zero'] + c['sign'] * t(deg.get(j, 0.0))) % TICKS
        return L.degrees()

    ok = True
    def check(what, got, want, tol=0.2):
        nonlocal ok
        good = abs(got - want) <= tol
        ok &= good
        print(f'  {"ok " if good else "BAD"} {what}: {got:.2f} (want {want:.2f})')

    d = at(); check('zero pose J1', d['J1'], 0); check('zero pose roll', d['wrist_roll'], 0)
    d = at(J1=170); check('J1 +170 (zero at raw 4000: crosses the raw 0/4095 seam)', d['J1'], 170)
    d = at(J1=-170); check('J1 -170 right after +170 (the stop is between: no unwrapping)', d['J1'], -170)
    d = at(J2=-60); check('J2 -60 with sign -1', d['J2'], -60)
    for r in (60, 120, 170, 200, 250):                                  # roll has no stop: keeps counting past 180
        d = at(wrist_roll=r)
    check('roll turned on to +250 (unwrapped)', d['wrist_roll'], 250)
    c = L.command(d); check('roll command clamped to +120', math.degrees(c['wrist_roll.pos']), 120)
    for r in (150, 60, 0, -100):
        d = at(wrist_roll=r)
    check('roll back to -100', d['wrist_roll'], -100)
    c = L.command(at(gripper=20)); check('gripper half open -> 45 mm', c['gripper.pos'] * 1000, 45, 1.0)
    c = L.command(at(gripper=-3)); check('gripper past closed -> 0', c['gripper.pos'] * 1000, 0, 0.01)
    c = L.command(at(J2=130)); check('J2 +130 clamped to the follower +115', math.degrees(c['shoulder.pos']), 115)
    print('selftest', 'PASSED' if ok else 'FAILED')
    return ok


if __name__ == '__main__':
    import sys
    if '--selftest' in sys.argv:
        sys.exit(0 if selftest() else 1)
    print(__doc__)
