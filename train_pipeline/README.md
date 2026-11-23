# Training pipeline (LeRobot)

Record demonstrations in the Isaac store scene, check them, turn them into a LeRobot dataset, and train ACT.
The split (decided 2026-11-19): **our own code** for IK and motion planning (`scripts/common/scara_ik.py`,
`collide.py`, `scripts/scenes/scene_v0/lib/pick_place_mono.py`), **LeRobot** for the dataset format and the policies.

Two conda environments, as in the SO-101 project, so LeRobot never has to live in the Isaac environment:

| Step | Environment | Script |
|---|---|---|
| 0. Set up the leader arm (once) | `env_isaaclab` or `lerobot_env` | `leader/calibrate.py`, `leader/check_leader.py` |
| 1. Record episodes | `env_isaaclab` (Isaac Sim 5.1) | `record/record_sim.py` (planner), `record/teleop_sim.py` (leader arm) |
| 2. Check them | `env_isaaclab` | `tools/inspect_raw.py`, `tools/replay_episode.py` |
| 3. Convert to a LeRobotDataset | `lerobot_env` (LeRobot 0.6.1) | `convert/to_lerobot.py` |
| 4. Train ACT | `lerobot_env` | `train/train_act.sh` |
| 5. Run the policy in the sim | `env_isaaclab` + `lerobot_env` (server) | `eval/eval_sim.py` (starts `eval/policy_server.py`) |

Run every command from the repo root.

## Folders

```
train_pipeline/
  common.py              settings every step shares: joint order + units, cameras, frame rate, paths, instructions
  record/record_sim.py   Isaac: the planner does the episodes, write raw HDF5
  record/teleop_sim.py   Isaac window: a person does them with the leader arm (same files)
  record/episode_writer.py  one episode -> one HDF5 file (used by both recorders)
  tools/inspect_raw.py   list episodes, contact sheet of one episode (no Isaac)
  tools/replay_episode.py  Isaac: replay the recorded actions from the saved start, check the result
  convert/to_lerobot.py  raw HDF5 -> LeRobotDataset (AV1 videos)
  train/train_act.sh     lerobot-train with ACT
  eval/eval_sim.py       Isaac: the trained policy does the task, scored; contact sheets per test
  eval/policy_server.py  lerobot_env: serves the checkpoint over a local socket (eval_sim.py starts it)
  leader/leader_arm.py   reads the leader's 6 servos -> follower commands (rad, m); --selftest checks the maths
  leader/setup_ids.py    servo ids 1..6 (new or replaced servos only)
  leader/calibrate.py    zero pose, directions, ranges, gripper opening -> leader/calibration/<name>.json
  leader/check_leader.py live angles + follower commands
  leader/check_sim.py    the leader drives the mono arm alone in Isaac Sim (calibration check on the twin)
  sim_common.py          Isaac helpers shared by the recorder, the replay and the eval (state, command, light, product moves)
  data/raw/<name>/       episode_000000.h5 ...              (git-ignored)
  data/lerobot/<name>/   converted datasets                  (git-ignored)
  outputs/<job>/         training runs, checkpoints          (git-ignored)
```

## 0. Set up the leader arm (once per arm)

Plug the leader's servo board into USB and power it. Find its port (the name that stays the same when re-plugged):

```bash
ls -l /dev/serial/by-id/                     # e.g. usb-1a86_USB_Single_Serial_5A68012416-if00 -> ../../ttyACM0
sudo chmod 666 /dev/ttyACM0                  # after every reboot (or once: sudo usermod -aG dialout $USER, log in again)
export LEADER_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_<serial>-if00
```

`lerobot-find-port` (in `lerobot_env`) also finds it: unplug the board when it asks.

**Servo ids (skip if the servos came from the SO-100 leader and are in the SO-100 order).** Ids 1..6 = shoulder J1,
elbow J2, wrist pan J3, pitch, roll, gripper: the same ids as `lerobot-setup-motors --teleop.type=so100_leader`.

```bash
conda activate lerobot_env
python train_pipeline/leader/setup_ids.py      # one servo at a time, gripper first
```

**Calibrate** (either env; all servos stay limp, you move the arm by hand):

```bash
python train_pipeline/leader/calibrate.py
python train_pipeline/leader/calibrate.py --dummy    # try the screens without the arm (a simulated leader; never saves)
```

Every step shows a **live table of all 6 joints**, redrawn in place (like `lerobot-calibrate`'s MIN / POS / MAX
table). Press Enter to go on.

1. **Zero pose**, the follower's zero: link 1 straight ahead (away from the stop rib), link 2 and the wrist in line
   with it, gripper level, the two jaw blades side by side (left / right), finger loops closed. The table shows each
   servo's raw reading and whether it is still; press Enter when every joint says `still`.
2. **Directions:** it asks you to move each joint one way (pans to the left, pitch down, roll clockwise seen from
   behind, loops open). That sets which way each servo counts, so a servo mounted the other way round is fine.
   The gripper step also measures how far the loops open (design 40°): fully open = 90 mm on the follower.
   The table marks the joint being taught with `>`, shows how far each joint has moved from the zero pose, and warns
   if another joint moves too.
3. **Ranges:** move every joint to both ends. The table shows MIN / POS / MAX per joint next to the follower's limits. J1 must stop at about ±174°; more than ±176° means the zero pose was
   not straight ahead and nothing is saved.
4. **Check** the live numbers (leader angle and the follower command it makes; `*` = the follower stops at its
   limit there), then `y` to save `train_pipeline/leader/calibration/leader_mono.json`. Commit it: the
   calibration belongs to the arm, so every PC gets it with `git pull`.

Why not `lerobot-calibrate`: it puts each zero in the middle of whatever range you sweep, and treats the wrist roll as
a full turn (the Store-Picker-Bot needed `calibrate_wrist_roll.py` for that). Here the zero is a pose, so leader and
follower angles match 1:1, and the encoder's +180 → −180 jump is half a turn from it: behind J1's end stop, out of
reach for J2, J3 and the pitch. The **roll** has no stop on the leader, so `leader_arm.py` counts it on past ±180
instead of jumping, and the follower stops at its own ±120. **Don't run `lerobot-calibrate` on this leader**: it
rewrites the servos' homing offsets; the leader code notices and asks you to calibrate again.

Check any time (prints the read rate too, which must stay well above 30 Hz):

```bash
python train_pipeline/leader/check_leader.py
```

Commands are clamped to the follower's limits (`*` in the check): shoulder ±115°, elbow ±122°, pitch −15..115°,
roll ±120°. Recalibrate after replacing a servo or taking the arm apart.

**Check it on the digital twin** (`env_isaaclab`): the leader drives the mono arm alone in Isaac Sim (no base, lift
or scene), first slowly to the leader's pose, then 1:1. The terminal shows leader angle, command, sim angle and lag
per joint. Move one leader joint at a time: the same sim joint must turn the same way.

```bash
OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 python train_pipeline/leader/check_sim.py
```

## 1. Record

```bash
conda activate env_isaaclab
OMNI_KIT_ACCEPT_EULA=YES python train_pipeline/record/record_sim.py --episodes 50 --place --name pick_tote_v1
```

- **Who does the task:** for now the planner (`--source planner`): the same offline-planned pick and place as the
  "Pick to tote" panel. A person with the leader arm: `teleop_sim.py` (1b).
- **One episode:** from the robot parked at the station, arm folded, to the arm folded again. Driving between
  stations is not recorded (navigation stays scripted). Before every episode the moved products go back to their spots.
- **`--place`:** after each pick, putting the product back is recorded as its own episode with its own instruction,
  so one dataset teaches two tasks.
- **Stations:** every station that picks reliably (`pick_place.PICKABLE`, 16 of 19), chosen at random;
  `--stations a,b` for a few.
- **Randomised per episode** so the policy can't memorise one picture (`--no_jitter` turns it off): where the robot stops
  (±2 cm, ±3°), the product (±1 cm, ±8°), the light (±25 %).
- Only successful episodes are kept (product in the tote / back on its spot, nothing else knocked over).
- Headless by default (`--window` to watch). Plans are cached, so the first episode at a station is slower.
- **`--noise mixed`:** human-like imperfections until the leader arm exists. Each episode draws a level (clean 30 %,
  mild 40 %, noisy 30 %): a slow random wobble on the commanded joints (wrist 0.6 / 1.2°, less up the arm, lift 0.5 / 1 mm),
  slower moves (80–100 % / 60–90 %), random pauses before moves and jaw actions, and the hand aiming up to 3 / 6 mm off
  the product's centre. The action stored is the command actually sent, noise included. The level is stored in the file.

Each frame (30 per second):

| Key | What |
|---|---|
| `observation.state` | 7 measured values, see below |
| `action` | the 7 values **commanded** at that moment (not the measured ones: the policy learns to command, and the gripper squeezes) |
| `observation.images.scene` | OAK-D Pro W colour image, 640 × 400 (JPEG in the raw file, AV1 video after converting) |
| `observation.images.wrist` | wrist camera, 640 × 360 |
| `observation.depth.scene` | only with `--depth`: depth in mm, for point-cloud policies later (not converted yet) |

The 7 values, in this order and in SI units. **The real robot's driver must use the same order and units.**

| # | Name | Joint in the URDF | Unit |
|---|---|---|---|
| 0 | `lift.pos` | `lift` | m (0–0.81) |
| 1 | `ring.pos` | `left_shoulder_pan` (the ring around the column) | rad |
| 2 | `shoulder.pos` | `left_elbow_pan` | rad |
| 3 | `elbow.pos` | `left_wrist_pan` | rad |
| 4 | `wrist_pitch.pos` | `left_wrist_pitch` | rad |
| 5 | `wrist_roll.pos` | `left_wrist_roll` | rad |
| 6 | `gripper.pos` | `left_finger_right` + `left_finger_left` | m, jaw opening (0–0.09) |

(The mono arm's URDF joint names are shifted by one because the ring took the shoulder's name; see CLAUDE.md.)

Each file also stores what is needed to rebuild its start (every joint, the base pose, the product's pose, the light),
the instruction (for example "pick the milk carton and put it in the tote") and the randomisation used.

### 1b. Record with the leader arm (teleop)

```bash
conda activate env_isaaclab
OMNI_KIT_ACCEPT_EULA=YES python train_pipeline/record/teleop_sim.py --name teleop_v1      # calibrate first (step 0)
```

A "Teleop recording" panel and the scene and wrist camera views (what the policy will see) open in the Isaac window.

1. Choose the product and the task (**pick into the tote** or **place back on its shelf**), then **Go to station**: the
   robot drives there with its arm folded. For a pick, moved products go back to their spots first.
2. **Start teleop:** the arm first moves **slowly** to the leader's pose, then follows it 1:1. Put the leader in the
   travel fold before you start, so nothing has to move. **Up / Down arrows** move the lift while held (until the
   lift controller exists).
3. **R** = record. Do the task and fold the arm again (episodes span folded to folded, like the planner's). Then
   **N** = save or **X** = discard. Saving checks the result automatically (in the tote / back within 20 mm,
   nothing else knocked over). A failed episode is saved marked FAILED, and the converter skips it.
4. **Stop teleop** (the arm holds its pose), choose the next product. Driving needs the arm folded. **Fold arm** is a
   straight move to the fold that does not check for collisions, so bring the hand clear of the furniture first.

The recorded action is the leader's pose limited to the real servo speeds, the same rule as the planner's. Each
frame also stores the leader's own angles (`leader.deg`; training doesn't use it). Leader and planner recordings go
in separate sets (the converter refuses to mix them).

**Without the leader arm:** `--leader dummy` plays a recorded planner episode back as raw servo readings through a
made-up calibration (zeros beside the encoder seam, two servos counting backwards). It tests the whole chain:
servo ticks → calibration → arm → recording. `--auto` does go / teleop / record / save by itself (add
`--headless` for no window). Tested 2026-11-20: the dummy pick passed in both modes. Its file replays (step 2) and
converts (step 3).

## 2. Check

```bash
python train_pipeline/tools/inspect_raw.py pick_tote_v1               # one line per episode
python train_pipeline/tools/inspect_raw.py pick_tote_v1 --sheet 0     # 8 moments of episode 0, both cameras -> PNG
OMNI_KIT_ACCEPT_EULA=YES python train_pipeline/tools/replay_episode.py pick_tote_v1 --episodes 0,1
```

The replay rebuilds each episode's start and sends **only the recorded actions**. If the product still ends up in
the tote (or back on its spot), the actions are enough to reproduce the demonstration. The replay also reports how far
the replayed joints drift from the recorded ones.

## 3. Convert

```bash
conda activate lerobot_env
python train_pipeline/convert/to_lerobot.py --raw pick_tote_v1            # -> data/lerobot/pick_tote_v1
python train_pipeline/convert/to_lerobot.py --raw pick_tote_v1 --kind pick --out pick_only_v1
```

Failed episodes are skipped. Planner and leader recordings can't be mixed in one dataset (they would teach two
meanings of "action"); convert them separately.

## 4. Train ACT

```bash
conda activate lerobot_env          # needed: lerobot-train only exists there
DATASET=pick_tote_v1 ./train_pipeline/train/train_act.sh
DATASET=pick_tote_v1 STEPS=200 SAVE_FREQ=100 JOB=smoke ./train_pipeline/train/train_act.sh   # quick check first
```

Defaults: 100 000 steps, batch 32, chunk 100 (3.3 s at 30 fps), checkpoints every 10 000 steps in
`train_pipeline/outputs/<JOB>/`. Change them with `STEPS=`, `BATCH_SIZE=`, `CHUNK=`, `GPU=`, or add any
`lerobot-train` option at the end. Resume: see the top of `train_act.sh`.

**Speed:** by default (`FAST=1`) training runs through `train/lerobot_train_fast.py`: bf16 mixed precision +
`torch.compile`, about 2x faster on the RTX 5060 Ti (7.2 instead of 3.4 steps/s for batch 16, about 3 min of compiling at
the start). Same model, data and settings; checkpoints load as usual. `FAST=0` = plain `lerobot-train` in float32.

## 5. Run the policy in the sim

```bash
conda activate env_isaaclab
OMNI_KIT_ACCEPT_EULA=YES python train_pipeline/eval/eval_sim.py \
    --checkpoint train_pipeline/outputs/<job>/checkpoints/last/pretrained_model --station shelf_b1_milk_b01 --episodes 10
```

- Each test starts like a recorded episode (products put back, the same randomisation) with a **different `--seed`**
  than the recording. The planner drives to the station; the moment the robot is parked, the policy takes over.
- Every 1/30 s the measured state and both camera images go to the policy server, which runs in `lerobot_env` (Isaac's
  NumPy 1 and LeRobot's NumPy 2 can't share pickled arrays, so only plain lists and bytes cross). Its 7-value answer is
  sent to the joints.
- A test ends when the product has been in the tote for 3 s, or after `--max_s` (90 s).
- Scores per test: grasped (lifted > 15 mm), in the tote, nothing else knocked over. A contact sheet per test (scene over
  wrist camera, one column every 3 s) goes next to the training run (`outputs/<job>/eval_<date>/`).
- `--n_action_steps 30` or `--ensemble 0.01` change how ACT's 100-action chunk is used (default: as trained).
- `--window` to watch.

## Things to know

- **`sim_common.py` is Isaac-only:** it needs a running Isaac app; import it after the scene is loaded.

- **Real time:** the recorder runs headless and does not need to keep up with real time; the episode's clock is
  the sim's. Teleop with the leader arm will need about real time; check the frame rate with the cameras on first.
- **Cameras:** the wrist camera's 120° lens is assumed until the real one is calibrated. Data recorded before that
  might need recording again.
- **Sim-to-real gaps:** servo gains are not measured from real servos, the sim gripper is a simple slide, product masses are
  estimates and the products are rigid.
