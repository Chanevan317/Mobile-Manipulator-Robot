"""Bench test: can an STS3215 run MULTI-TURN position control? (mono ring: the 4:1 gear needs 4 servo turns for a
+-180 deg shoulder; normal position mode covers only 1 turn = 90 deg of shoulder.)

One servo on the bus, nothing attached to the horn except a marker (tape flag) so you can count turns.
It changes the servo's angle limits in EEPROM (both 0 = multi-turn) and RESTORES them at the end, even on Ctrl+C.

    conda activate lerobot_env
    python servo_multiturn_test.py --port /dev/ttyACM0 --id 1

Registers (LeRobot's STS3215 table): 9/11 Min/Max_Position_Limit, 33 Operating_Mode, 40 Torque_Enable,
41 Acceleration, 42 Goal_Position, 46 Goal_Speed, 55 Lock, 56 Present_Position, 66 Moving.
Positions use sign-magnitude (bit 15 = minus).
"""
import argparse, sys, time
import scservo_sdk as scs

ap = argparse.ArgumentParser()
ap.add_argument('--port', default='/dev/ttyACM0')
ap.add_argument('--id', type=int, default=1)
ap.add_argument('--baud', type=int, default=1_000_000)
args = ap.parse_args()

TURN = 4096
port = scs.PortHandler(args.port)
if not port.openPort() or not port.setBaudRate(args.baud):
    sys.exit(f'cannot open {args.port} at {args.baud} (permissions? try: sudo chmod 666 {args.port})')
pk = scs.PacketHandler(0)                     # STS: little-endian


def ok(res, err, what):
    if res != scs.COMM_SUCCESS:
        raise RuntimeError(f'{what}: {pk.getTxRxResult(res)}')
    if err:
        print(f'   (servo status bits after {what}: {pk.getRxPacketError(err)})')


def r1(a, what):
    v, res, err = pk.read1ByteTxRx(port, args.id, a); ok(res, err, what); return v


def r2(a, what):
    v, res, err = pk.read2ByteTxRx(port, args.id, a); ok(res, err, what); return v


def w1(a, v, what):
    res, err = pk.write1ByteTxRx(port, args.id, a, v); ok(res, err, what)


def w2(a, v, what):
    res, err = pk.write2ByteTxRx(port, args.id, a, v); ok(res, err, what)


def dec(v):                                  # sign-magnitude -> int
    return -(v & 0x7FFF) if v & 0x8000 else v


def enc(v):
    return ((-v) & 0x7FFF) | 0x8000 if v < 0 else v & 0x7FFF


def pos():
    return dec(r2(56, 'read position'))


def eeprom(min_lim, max_lim, mode):
    w1(40, 0, 'torque off'); w1(55, 0, 'unlock EEPROM')
    w2(9, min_lim, 'min limit'); w2(11, max_lim, 'max limit'); w1(33, mode, 'mode')
    w1(55, 1, 'lock EEPROM'); time.sleep(0.05)


def move_to(target, label, timeout=20.0):
    w2(42, enc(target), 'goal position'); time.sleep(0.15)
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = pos(); print(f'\r   {label}: position {p:+7d}  (target {target:+7d})   ', end='', flush=True)
        if r1(66, 'moving') == 0 and abs(p - target) < 30: break
        time.sleep(0.1)
    p = pos(); print(f'\r   {label}: position {p:+7d}  (target {target:+7d}, error {p - target:+d})          ')
    return p


def ask(q):
    return input(f'   {q} [y/n] ').strip().lower().startswith('y')


model, res, err = pk.ping(port, args.id)
if res != scs.COMM_SUCCESS:
    sys.exit(f'no servo with ID {args.id} on {args.port}: {pk.getTxRxResult(res)}')
orig = dict(min=r2(9, 'min limit'), max=r2(11, 'max limit'), mode=r1(33, 'mode'))
print(f'\nServo ID {args.id}: model {model}, limits {orig["min"]}..{orig["max"]}, mode {orig["mode"]}, position {pos()}')
print('This test turns the horn up to 2 full turns each way. Nothing must be attached except a marker.')
if not ask('Ready?'):
    sys.exit()
results = {}
try:
    # 1) multi-turn: both angle limits 0, position mode
    eeprom(0, 0, 0)
    print(f'\n1) limits now {r2(9, "min")}..{r2(11, "max")}, mode {r1(33, "mode")} (0..0 = multi-turn)')
    w1(41, 30, 'acceleration'); w2(46, 1500, 'speed (steps/s)'); w1(40, 1, 'torque on')
    p0 = pos(); print(f'   start position {p0}')
    # 2) +2 turns, then back to -1 turn (from the start)
    p = move_to(p0 + 2 * TURN, '+2 turns')
    results['reached +2 turns'] = abs(p - (p0 + 2 * TURN)) < 30
    results['horn turned 2 full turns'] = ask('Did the marker go round exactly 2 full turns (forward)?')
    p = move_to(p0 - TURN, '-1 turn')
    results['reached -1 turn'] = abs(p - (p0 - TURN)) < 30
    results['horn came back 3 turns'] = ask('Did it come back 3 full turns (ending 1 turn before the start)?')
    # 3) what survives a power cycle?
    before = pos(); w1(40, 0, 'torque off')
    print(f'\n3) Position before power-off: {before}.')
    input('   Unplug the servo POWER (keep USB), wait 3 s, plug it back in, then press Enter.')
    time.sleep(1.0); after = pos()
    print(f'   Position after power-on: {after}  (before: {before}; {before % TURN} within one turn)')
    results['keeps the turn count after power-off'] = abs(after - before) < 30
finally:
    try:
        eeprom(orig['min'], orig['max'], orig['mode'])
        print(f'\nRestored limits {r2(9, "min")}..{r2(11, "max")}, mode {r1(33, "mode")}; torque off.')
    except Exception as e:
        print(f'\n!! could not restore the limits ({e}). Set them back: min {orig["min"]}, max {orig["max"]}, mode {orig["mode"]}')
    port.closePort()

print('\nResults:')
for k, v in results.items():
    print(f'   {"PASS" if v else "FAIL"}  {k}')
mt = all(results.get(k) for k in ('reached +2 turns', 'horn turned 2 full turns', 'reached -1 turn', 'horn came back 3 turns'))
print('\nVerdict:', 'multi-turn position control WORKS.' if mt else 'multi-turn position control does NOT work this way.')
if mt:
    print('   ' + ('It also keeps the turn count after power-off: no homing needed.' if results.get('keeps the turn count after power-off')
                 else 'It forgets the turn count after power-off (expected): the ring needs a homing mark (hall sensor + magnet).'))
