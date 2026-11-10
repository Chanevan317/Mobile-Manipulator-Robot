"""ROS 2 check of joint limits, zero pose and joint directions (the automated part of the RViz slider check).

Starts its own robot_state_publisher with urdf/mobile_manipulator_<layout>_ros.urdf, sends /joint_states, reads TF back.
Run with ROS 2 sourced and conda deactivated (do NOT run display.launch.py at the same time):
    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 scripts/ros_joint_check.py                  # prototype layout
"""
import math, os, signal, subprocess, tempfile, time, xml.etree.ElementTree as ET
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import GetParameters
import tf2_ros
import sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import argparse
import scara_ik as K
_ap = argparse.ArgumentParser(); _ap.add_argument('--layout', default='prototype', choices=('prototype',))
LAYOUT = _ap.parse_args().layout
K.use(LAYOUT)

here = os.path.dirname(os.path.abspath(__file__))
urdf_text = open(os.path.join(here, '..', 'urdf', K.urdf_name(LAYOUT, ros=True))).read()
print('layout', LAYOUT)
# The URDF is too long for a command-line argument, so pass it through a parameters file.
params = os.path.join(tempfile.mkdtemp(), 'rsp.yaml')
yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': urdf_text}}}, open(params, 'w'), width=1e9)
rsp = subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '--params-file', params],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
time.sleep(3)
rclpy.init()
node = Node('slider_test')
pub = node.create_publisher(JointState, 'joint_states', 10)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)

# robot_description from robot_state_publisher (the same one the sliders use)
cli = node.create_client(GetParameters, '/robot_state_publisher/get_parameters'); cli.wait_for_service(10)
fut = cli.call_async(GetParameters.Request(names=['robot_description'])); rclpy.spin_until_future_complete(node, fut)
root = ET.fromstring(fut.result().values[0].string_value)
joints = {j.get('name'): j for j in root.findall('joint')}
movable = [n for n, j in joints.items() if j.get('type') != 'fixed']

def lim(n):
    l = joints[n].find('limit'); return float(l.get('lower')), float(l.get('upper'))

def pos(q, frame):
    msg = JointState(); msg.name = movable; msg.position = [q.get(n, 0.0) for n in movable]
    for _ in range(40):
        msg.header.stamp = node.get_clock().now().to_msg(); pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)
    t = buf.lookup_transform('base_link', frame, rclpy.time.Time()).transform.translation
    return (t.x, t.y, t.z)

fails = []
def check(label, ok, detail):
    print(f"{'OK  ' if ok else 'FAIL'} {label:44s} {detail}")
    if not ok: fails.append(label)

D = math.radians
print('--- limits from /robot_description vs CLAUDE.md section 7')
table = {'lift': (0, 0.885), 'shoulder_pan': (-D(115), D(115)), 'elbow_pan': (-D(125), D(125)), 'wrist_pan': (-D(122), D(122)),
         'wrist_pitch': (-D(15), D(115)), 'wrist_roll': (-D(120), D(120)), 'finger_right': (0, 0.045), 'finger_left': (0, 0.045)}
for side in ('left_', 'right_', ''):
    for k, (lo, hi) in table.items():
        n = side + k if k != 'lift' else 'lift'
        if (side == '') != (k == 'lift'): continue
        a, b = lim(n)
        check(f'limit {n}', abs(a - lo) < 1e-3 and abs(b - hi) < 1e-3, f'[{a:.4f}, {b:.4f}]')

print('--- zero pose')
for s in ('left', 'right'):
    x, y, z = pos({}, f'{s}_tcp')
    ex, ey, ez, _ = K.fk(s, {'shoulder_pan': 0.0, 'elbow_pan': 0.0, 'wrist_pan': 0.0, 'lift': 0.0})
    check(f'{s}_tcp at zero (along its zero heading, level)', math.dist((x, y, z), (ex, ey, ez)) < 0.005,
          f'({x:.4f}, {y:.4f}, {z:.4f}) expected ({ex:.4f}, {ey:.4f}, {ez:.4f})')

print('--- directions (+ step from zero, frame motion in base_link)')
def move(q, frame, base_q=None):
    p0 = pos(base_q or {}, frame); p1 = pos({**(base_q or {}), **q}, frame)
    return tuple(b - a for a, b in zip(p0, p1))
d = move({'lift': 0.3}, 'carriage'); check('lift + -> carriage up 0.3 m', abs(d[2] - 0.3) < 1e-3, f'dz {d[2]:+.4f}')
for s in ('left', 'right'):
    for j in ('shoulder_pan', 'elbow_pan', 'wrist_pan'):
        d = move({f'{s}_{j}': 0.3}, f'{s}_tcp'); a = -K.MOUNT_YAW[s]           # motion in the arm's mount frame
        dy = math.sin(a) * d[0] + math.cos(a) * d[1]
        check(f'{s}_{j} + -> tcp turns CCW (+y of the arm)', dy > 0.01, f'dy {dy:+.4f}')
    d = move({f'{s}_wrist_pitch': 0.3}, f'{s}_tcp'); check(f'{s}_wrist_pitch + -> tip down (-z)', d[2] < -0.01, f'dz {d[2]:+.4f}')
    op = {f'{s}_finger_right': 0.03, f'{s}_finger_left': 0.03}
    d = move({f'{s}_wrist_roll': 0.3}, f'{s}_finger_right', op); check(f'{s}_wrist_roll + -> +y jaw goes up (+x axis)', d[2] > 0.005, f'dz {d[2]:+.4f}')
    dr = move({f'{s}_finger_right': 0.03}, f'{s}_finger_right'); dl = move({f'{s}_finger_left': 0.03}, f'{s}_finger_left')
    a = -K.MOUNT_YAW[s]                                                     # jaw motion in the arm's mount frame
    ry = math.sin(a) * dr[0] + math.cos(a) * dr[1]; ly = math.sin(a) * dl[0] + math.cos(a) * dl[1]
    check(f'{s} jaws open apart, 30 mm each', abs(ry - 0.03) < 1e-3 and abs(ly + 0.03) < 1e-3, f'right dy {ry:+.4f}  left dy {ly:+.4f} (arm frame)')

print('--- every movable joint at both limits (TF stays finite)')
bad = []
for n in movable:
    if joints[n].get('type') == 'continuous': continue
    for v in lim(n):
        p = pos({n: v}, joints[n].find('child').get('link'))
        if not all(map(math.isfinite, p)): bad.append(n)
check(f'{len([n for n in movable if joints[n].get("type") != "continuous"])} limited joints at lower/upper', not bad, str(bad))
print(f'\n{len(fails)} failures', fails)
node.destroy_node(); rclpy.shutdown()
os.killpg(rsp.pid, signal.SIGTERM)   # 'ros2 run' leaves its child running if only the wrapper is killed
raise SystemExit(1 if fails else 0)
