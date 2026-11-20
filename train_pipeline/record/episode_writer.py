"""One episode -> one raw HDF5 file (shared by record_sim.py and teleop_sim.py; import after the scene is loaded).

    W = EpisodeWriter(task, items, out_dir, source='planner', tilt=40)
    W.start('pick', sid, name)        # first frame now
    W.step()                          # after every control step: a frame every 60 / FPS steps
    W.stop(success, why, extra)       # writes the file (or drops a failed one); returns its path or None

Frame = measured state, the command the drives got (the action), both camera images as JPEG. File attributes = what
replay_episode.py needs to rebuild the start (every joint, the base, the product's bodies) + the task text.
frame_extra: optional fn() -> {dataset name: array} stored per frame as well (teleop: the leader's own angles).
"""
import io

import numpy as np
import h5py
from PIL import Image

import common as C
import cameras as CAM
import pick_place as PP
import sim_common as SCM

EVERY = round(60 / C.FPS)                                                # control steps per recorded frame


class EpisodeWriter:
    def __init__(self, task, items, out_dir, source, tilt, jpeg=95, depth=False, keep_failed=False, frame_extra=None):
        self.task, self.items, self.out_dir, self.source, self.tilt = task, items, out_dir, source, tilt
        self.jpeg, self.depth_on, self.keep_failed, self.frame_extra = jpeg, depth, keep_failed, frame_extra
        out_dir.mkdir(parents=True, exist_ok=True)
        self.next_index = 1 + max([int(p.stem.split('_')[1]) for p in out_dir.glob('episode_*.h5')] or [-1])
        self.active = False
        self.k = 0

    def start(self, kind, sid, name):
        t = self.task
        self.meta = dict(kind=kind, station=sid, product=name)
        self.state, self.action, self.t, self.extra = [], [], [], {}
        self.img = {k: [] for k in C.CAMERAS}
        self.depth = []
        self.q0 = t.robot.get_joint_positions().copy()
        self.pose0 = SCM.body_poses(name)
        self.base0 = t.base_pose()
        self.active, self.k = True, 0
        self.sample()

    def frames(self):
        return len(self.state) if self.active else 0

    def sample(self):
        self.state.append(SCM.state()); self.action.append(SCM.sent()); self.t.append(self.task.t)
        fr = CAM.capture(self.task, self.task.cams)
        for key, cam in C.CAMERAS.items():
            buf = io.BytesIO(); Image.fromarray(fr[f'{cam}_rgb']).save(buf, 'JPEG', quality=self.jpeg)
            self.img[key].append(np.frombuffer(buf.getvalue(), np.uint8))
        if self.depth_on:
            d = fr.get('scene_depth')
            self.depth.append(np.zeros(fr['scene_rgb'].shape[:2], np.uint16) if d is None
                              else np.clip(d * 1000, 0, 65535).astype(np.uint16))
        if self.frame_extra:
            for k, v in self.frame_extra().items():
                self.extra.setdefault(k, []).append(np.asarray(v, np.float32))

    def step(self):
        if self.active:
            self.k += 1
            if self.k % EVERY == 0:
                self.sample()

    def stop(self, success, why, extra=None, keep=None):
        """Write the episode; returns its path, or None when dropped. keep: True / False overrides the success rule."""
        if not self.active:
            return None
        self.active = False
        if keep is False or (keep is None and not success and not self.keep_failed):
            print(f'      episode dropped ({why})', flush=True)
            return None
        m = self.meta
        prod = PP.NAMES.get(self.items[m['product']]['pkg'], m['product'])
        where = 'table' if m['station'].startswith('table') else 'shelf'
        path = self.out_dir / f'episode_{self.next_index:06d}.h5'
        with h5py.File(path, 'w') as f:
            f['observation.state'] = np.stack(self.state)
            f['action'] = np.stack(self.action)
            f['time'] = np.array(self.t) - self.t[0]
            vl = h5py.vlen_dtype(np.uint8)
            for key, frames in self.img.items():
                ds = f.create_dataset(key, (len(frames),), dtype=vl)
                for i, b in enumerate(frames):
                    ds[i] = b
            if self.depth_on:
                f.create_dataset('observation.depth.scene', data=np.stack(self.depth), compression='gzip', compression_opts=4)
            for k, v in self.extra.items():
                f[k] = np.stack(v)
            a = f.attrs
            a['fps'] = C.FPS; a['robot_type'] = C.ROBOT_TYPE; a['action_source'] = self.source
            a['joint_names'] = C.JOINT_NAMES; a['image_encoding'] = 'jpeg'
            a['task'] = C.task_text(m['kind'], prod, where); a['kind'] = m['kind']
            a['station'] = m['station']; a['product'] = m['product']; a['product_name'] = prod
            a['success'] = bool(success); a['result'] = why
            a['scene_camera'] = CAM.SCENE_MODEL; a['scene_camera_tilt_deg'] = self.tilt
            # what replay_episode.py needs to rebuild the start: every joint, the base, the product's bodies, the light
            a['q0_names'] = list(self.task.names); a['q0'] = self.q0
            a['base0'] = [float(self.base0[0][0]), float(self.base0[0][1]), float(self.base0[1])]
            a['product_pose0'] = np.array([np.concatenate([p, q]) for p, q in self.pose0])
            for k, v in (extra or {}).items():
                if v is not None:
                    a[k] = v
        self.next_index += 1
        n = len(self.state)
        print(f'      saved {path.name}: {n} frames ({n / C.FPS:.1f} s), "{C.task_text(m["kind"], prod, where)}"'
              f'{"" if success else "  (marked FAILED: the converter skips it)"}', flush=True)
        return path
