"""The robot's cameras in Isaac Sim, set to the real cameras' specs (2026-11-18).

    import cameras as CAM
    cams = task.cams                        # every proto_task.Task adds them in start() (MM_CAMERAS=0: off)
    cams = CAM.add()                        # scripts with their own robot setup: after world.reset()
    frames = CAM.capture(task, cams)        # {'scene_rgb': HxWx3 uint8, 'scene_depth': HxW float32 m (0 = no depth),
                                            #  'left_wrist_rgb': ...}

The wrist camera renders 640 x 360 (16:9, the OV2710's native 1920 x 1080 shape); the scene camera 640 x 480 (4:3,
the OAK colour sensor's full 12 MP mode, scaled down on the device; user 2026-11-18: the 16:9 mode, 63 x 38 deg, was
too narrow). Light enough to render fast. The prims sit on the URDF's optical frames (ROS convention: z forward, x right,
y down); a USD camera looks along -Z with +Y up, so each is turned 180 deg about x under its frame.

SCENE: Orbbec Gemini 2 since 2026-11-24 (user: the OAK-D Pro W was sold out in India; the Gemini 2 ordered for the reBot
    arm is reused; SCENE_GEMINI2 below: colour 86 x 55 deg, a normal (rectilinear) lens, 640 x 360 from its 16:9 modes;
    depth aligned to colour (its D2C), active stereo, 0.15 .. 10 m). Earlier, kept selectable with MM_SCENE_CAM:
    pro_w = Luxonis OAK-D Pro W, OV9782 (2026-11-19, 127 x 80 deg fisheye, 640 x 400); pro_af = OAK-D Pro, auto-focus version: OAK-D Pro, auto-focus version (docs.luxonis.com, OAK-D Pro page, 'Variant: Auto Focus'):
    colour IMX378 1/2.3", 78 / 66 / 54 deg D / H / V over the full 4056 x 3040 sensor -> f = 2028 / tan 33 deg = 3123 px.
    Used: the full sensor (4:3), 66 deg H; square pixels give 51.9 deg V (Luxonis rounds to 54). Its 16:9 modes
    (1080p / 4K) crop to 3840 x 2160 -> 63.1 x 38.2 deg. Depth = Isaac's distance to the image plane from
    the colour camera's viewpoint (like the OAK's depth aligned to colour), kept only inside the OAK's range: 0.2 m
    (closest, 400P + extended disparity) .. 12 m; outside -> 0 (no depth), as the real device reports.
WRIST: OV2710 2 MP USB board (Evontro, board 'RYS-720P/1080P.S3 2016/07/12', 32 x 32 mm, M12 lens). The seller lists
    no lens; the same board ships with 90 .. 160 deg lenses. ASSUMED until measured: 120 deg horizontal, OpenCV fisheye
    model with k1..k4 = 0 (equidistant). Replace WRIST['fx'] / 'fy' / 'cx' / 'cy' / 'k' with the checkerboard
    calibration of the real camera (cv2.fisheye.calibrate gives exactly these, scaled to 640 x 360).
"""
import math

import numpy as np

RES = (640, 360)

_F_OAK = 2028.0 / math.tan(math.radians(33.0))                 # px on the full 4056-wide IMX378 sensor
SCENE_PRO_AF = dict(link='scene_camera_optical', res=(640, 480), assumed=False,
             hfov=2 * math.degrees(math.atan(2028.0 / _F_OAK)),   # 66 deg (full 4:3 sensor, 12 MP mode)
             depth=(0.2, 12.0), forward=0.0,
             model='Luxonis OAK-D Pro AF: IMX378 colour (full 4:3 sensor) + depth aligned to it')

# Luxonis OAK-D Pro W, OV9782 version: the scene camera 2026-11-19 .. 11-24 (user, in the BOM; the AF's 66 deg was too narrow): colour OV9782 1 MP 1280 x 800,
# global shutter, fixed focus, listed 150 / 127 / 80 deg D / H / V (Luxonis spec sheet 150 / 128 / 80); stereo OV9282 with the
# same wide view. An equidistant fisheye (r = f * theta) with f = 640 / 63.5 deg = 577 px at 1280 x 800 gives exactly
# 127 / 79.4 / 149.7 deg, so k1..k4 = 0 matches the listed numbers (replace with the unit's factory calibration).
# Rendered at 640 x 400 (half resolution, same 16:10 shape).
_F_W = 320.0 / math.radians(63.5)
SCENE_PRO_W = dict(link='scene_camera_optical', res=(640, 400), assumed=False, hfov=127.0,
                   fx=_F_W, fy=_F_W, cx=320.0, cy=200.0, k=[0.0, 0.0, 0.0, 0.0], depth=(0.2, 12.0), forward=0.0,
                   model='Luxonis OAK-D Pro W (OV9782 colour, global shutter, 127 x 80 deg fisheye) + depth aligned to it')
# Orbbec Gemini 2 (G20155, datasheet v1.1 + drawing 20241107): colour 1920 x 1080 16:9 = 86 x 55 deg (rolling shutter;
# its 4:3 modes crop to 63 x 50), a normal lens -> pinhole here (replace with the unit's factory intrinsics, Orbbec SDK);
# depth 91 x 66 deg, active stereo IR, 0.15 .. 10 m (optimal 0.2 .. 5), D2C-aligned to colour. Rendered 640 x 360.
SCENE_GEMINI2 = dict(link='scene_camera_optical', res=(640, 360), assumed=False, hfov=86.0, depth=(0.15, 10.0), forward=0.0,
                     model='Orbbec Gemini 2 (colour 86 x 55 deg, 16:9) + depth aligned to it')
SCENE_MODELS = {'gemini2': SCENE_GEMINI2, 'pro_af': SCENE_PRO_AF, 'pro_w': SCENE_PRO_W}
SCENE_MODEL = __import__('os').environ.get('MM_SCENE_CAM', 'gemini2')    # the robot's camera (user 2026-11-24); pro_w / pro_af = the OAK-Ds
SCENE = SCENE_MODELS[SCENE_MODEL]

_HFOV_WRIST = 120.0                                              # ASSUMED (lens not listed)
_FX_WRIST = (RES[0] / 2) / math.radians(_HFOV_WRIST / 2)         # equidistant fisheye: r = f * theta
WRIST = dict(link='{side}_wrist_camera_optical', res=RES, assumed=True, hfov=_HFOV_WRIST,
             fx=_FX_WRIST, fy=_FX_WRIST, cx=RES[0] / 2, cy=RES[1] / 2, k=[0.0, 0.0, 0.0, 0.0],
             forward=0.0165,   # the URDF frame is at the sensor; the CAD board's lens barrel reaches 16 mm forward and
                               # blocked the view: the camera sits at the lens front (as a real lens's view starts there)
             model='OV2710 2 MP USB, M12 wide lens (FOV ASSUMED 120 deg H until calibrated)')


AUTO_EXPOSURE = dict(tau=3.0)        # adaptation speed (RTX default 0.5 = slow; real webcams settle in ~0.3-1 s)


def auto_exposure(on=True, tau=AUTO_EXPOSURE['tau']):
    """Automatic exposure, as both real cameras (OAK and the UVC board) have: RTX's histogram exposure. It is a renderer
    setting, so it applies to every render (the viewport too). Without it the OAK looking into a shelf or down on the
    table in the robot's shadow came out at a brightness of 8-32 / 255 (2026-11-18)."""
    import carb
    st = carb.settings.get_settings()
    st.set('/rtx/post/histogram/enabled', bool(on))
    st.set('/rtx/post/histogram/tau', float(tau))


def _define(stage, path, spec):
    """USD camera prim under an optical-frame link: turned 180 deg about x, pinhole apertures for the FOV."""
    from pxr import Gf, Sdf, UsdGeom
    cam = UsdGeom.Camera.Define(stage, path)
    x = UsdGeom.Xformable(cam.GetPrim())
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, spec.get('forward', 0.0)))   # along the optical frame's z (forward)
    x.AddOrientOp().Set(Gf.Quatf(0.0, 1.0, 0.0, 0.0))            # 180 deg about x: USD -Z / +Y -> optical +Z / -Y
    f = 10.0                                                     # any focal length: only aperture / f sets the FOV
    ha = 2 * f * math.tan(math.radians(spec['hfov'] / 2))
    cam.CreateFocalLengthAttr(f)
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(ha * spec['res'][1] / spec['res'][0])   # square pixels
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    # locked: looking through it in the viewport, scroll / drag would MOVE the camera off its mount (the viewport's own
    # 'Lock' option, the same attribute)
    cam.GetPrim().CreateAttribute('omni:kit:cameraLock', Sdf.ValueTypeNames.Bool).Set(True)
    return path


def enabled():
    """MM_CAMERAS=0 in the environment turns the cameras off in every script (they add rendering work in a window)."""
    import os
    return os.environ.get('MM_CAMERAS', '1') not in ('0', 'no', 'off', 'false')


def add(task=None, root=None):
    """Camera prims + render products on every robot in the stage (call after world.reset()): one OAK on each
    'scene_camera_optical' frame and one OV2710 on each '<side>_wrist_camera_optical' frame found (under `root` if
    given). Works for any script: one robot, two side by side (isaac_view), an arm alone on a stand. Returns
    {name: Camera} ('scene', 'left_wrist', ...; '<robot>/scene' etc. when several robots are in the stage). Calling it
    again reuses the cameras already made. {} when MM_CAMERAS=0."""
    if not enabled():
        print('cameras: off (MM_CAMERAS=0)', flush=True)
        return {}
    import omni.usd
    from isaacsim.sensors.camera import Camera
    stage = omni.usd.get_context().get_stage()
    from pxr import UsdPhysics
    # only the robot's links (rigid bodies): the URDF importer also makes /colliders/<link> and /visuals/<link>
    # prototype scopes with the same names (the first version put a second set of cameras there)
    frames = [p for p in stage.Traverse()
              if (p.GetName() == SCENE['link'] or p.GetName().endswith('_wrist_camera_optical'))
              and p.HasAPI(UsdPhysics.RigidBodyAPI)
              and (root is None or str(p.GetPath()).startswith(root.rstrip('/') + '/'))]
    robots = sorted({str(p.GetParent().GetPath()) for p in frames})
    auto_exposure()
    cams = {}
    for fr in frames:
        robot = str(fr.GetParent().GetPath())
        scene = fr.GetName() == SCENE['link']
        spec = SCENE if scene else WRIST
        short = 'scene' if scene else fr.GetName().replace('_camera_optical', '')     # 'left_wrist'
        name = short if len(robots) == 1 else f"{robot.strip('/').split('/')[-1]}/{short}"
        path = f"{fr.GetPath()}/{'oak_rgb' if scene else 'ov2710'}"
        cams[name] = make(stage, path, spec, name)
    for n, c in cams.items():
        spec = SCENE if n.endswith('scene') else WRIST
        print(f"camera {n}: {spec['model']}, {spec['res'][0]} x {spec['res'][1]}, {spec['hfov']:.1f} deg H"
              f"{'  (ASSUMED lens)' if spec['assumed'] else ''}", flush=True)
    if not cams:
        print('cameras: no camera frames in the stage', flush=True)
    return cams


def make(stage, path, spec, name):
    """One camera prim at `path` (created under its parent if missing) + its render product: depth if the spec has a range,
    the OpenCV fisheye model if it has 'k'."""
    from isaacsim.sensors.camera import Camera
    if not stage.GetPrimAtPath(path).IsValid():
        _define(stage, path, spec)
    c = Camera(prim_path=path, name=name.replace('/', '_'), resolution=spec['res'])
    c.initialize()
    if 'depth' in spec:
        c.add_distance_to_image_plane_to_frame()
    if 'k' in spec:
        c.set_opencv_fisheye_properties(cx=spec['cx'], cy=spec['cy'], fx=spec['fx'], fy=spec['fy'], fisheye=spec['k'])
    _LIVE.append(c)
    _hook_shutdown()
    return c


# Release the cameras BEFORE Isaac shuts down (2026-11-19, user's crash on closing the pick_to_tote window: segfault in
# omni.syntheticdata, std::bad_variant_access, from Python's atexit). Isaac's Camera destroys its annotators / render
# product in __del__; left alive, that runs during interpreter exit after Kit has torn the render graph down. Every
# script's app.close() now releases them first (SimulationApp.close is wrapped once); a script that dies on an error
# gets the same at exit (release + a proper app.close()).
_LIVE = []
_HOOKED = {'done': False}


def release():
    """Detach every camera's annotators and destroy its render product (idempotent)."""
    while _LIVE:
        c = _LIVE.pop()
        try:
            c.destroy()
        except Exception:
            pass


def _hook_shutdown():
    if _HOOKED['done']:
        return
    _HOOKED['done'] = True
    import atexit
    from isaacsim import SimulationApp
    close0 = SimulationApp.close

    def close(self, *a, **k):
        release()
        if getattr(self, '_cameras_closed', False):
            return None
        self._cameras_closed = True
        return close0(self, *a, **k)
    SimulationApp.close = close

    import sys
    _hook0 = sys.excepthook
    _err = {'raised': False}

    def excepthook(*ex):
        _err['raised'] = True
        _hook0(*ex)
    sys.excepthook = excepthook

    def at_exit():
        """A script that dies on an error never calls app.close(): Kit's own exit-time shutdown then crashes in the
        render graph (measured 2026-11-19: segfault even with the cameras released). After an error: release the
        cameras and leave at once with exit code 1 (Kit's close would exit 0 and hide the failure); otherwise close
        the app properly."""
        import gc, os
        release()
        if _err['raised']:
            sys.stdout.flush(); sys.stderr.flush()
            os._exit(1)
        for o in gc.get_objects():
            if isinstance(o, SimulationApp):
                try:
                    if not getattr(o, '_cameras_closed', False):
                        o._cameras_closed = True
                        close0(o)
                except Exception:
                    pass
    atexit.register(at_exit)          # registered after Kit's own handler, so it runs before it


def _ready(cams):
    for n, c in cams.items():
        a = c.get_rgba()
        if a is None or np.asarray(a).ndim != 3:
            return False
        if n.endswith('scene'):
            d = c.get_depth()
            if d is None or np.asarray(d).ndim != 2:
                return False
    return True


_warm = {'done': False}


def capture(task, cams, renders=3, max_renders=60, settle=90):
    """Render and read every camera. Headless runs step physics without rendering, so this renders first: `renders`
    frames, and on the first call until every camera has data (the render pipeline needs a few frames to warm up).
    renders=3 (2026-11-19, measured: snap the arm between two poses, capture once): 1 or 2 renders still return the
    image from BEFORE the last move (recorded frames trailed their joint values by one frame, the panel's shots by a
    whole stage); 3 show the current pose. Returns {'<name>_rgb': uint8 HxWx3, 'scene_depth': float32 HxW (m, 0 = no depth)}."""
    for _ in range(renders):
        task.world.render()
    k = 0
    while not _ready(cams) and k < max_renders:
        task.world.render(); k += 1
    if not _warm['done']:                     # first capture: let the auto exposure settle (a running sim adapts on its own)
        for _ in range(settle):
            task.world.render()
        _warm['done'] = True
    out = {}
    for n, c in cams.items():
        rgba = c.get_rgba()
        out[f'{n}_rgb'] = np.asarray(rgba)[..., :3].astype(np.uint8) if rgba is not None and np.asarray(rgba).ndim == 3 else None
        if n.endswith('scene'):
            d = c.get_depth()
            if d is None or np.asarray(d).ndim != 2:
                d = None
            else:
                d = np.asarray(d, np.float32).copy()
                lo, hi = SCENE['depth']
                d[~np.isfinite(d) | (d < lo) | (d > hi)] = 0.0
            out[n.replace('scene', 'scene_depth') if n != 'scene' else 'scene_depth'] = d
    return out


def _turbo_lut():
    """256 x 3 uint8 colour map, near = red ... far = blue (Google's Turbo, polynomial fit; no matplotlib needed)."""
    t = np.linspace(0.0, 1.0, 256)[:, None]
    c = np.array([[0.13572138, 4.61539260, -42.66032258, 132.13108234, -152.94239396, 59.28637943],
                  [0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604],
                  [0.10667330, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973]])
    rgb = sum(c[:, k] * t ** k for k in range(6))
    return (np.clip(rgb, 0, 1)[::-1] * 255).astype(np.uint8)       # reversed: near = warm


def depth_window(cam, title='Scene camera depth', near=0.2, far=2.0, every=4, position=None):
    """A window that shows `cam`'s depth (Isaac Camera with a depth annotator, e.g. task.cams['scene']) live, coloured from
    `near` (red) to `far` (blue) metres; black = no depth (outside the device's range, SCENE['depth'], as the real camera
    reports). Updated every `every` app frames from the annotator's latest data (no extra render), also while a pick runs.
    Keep the returned (window, subscription) alive."""
    import omni.ui as ui, omni.kit.app
    lo_dev, hi_dev = SCENE['depth']
    w, h = SCENE['res']
    if position is None:                      # just above the camera row the scripts open at y 820
        position = (20, 820 - (h + 70) - 10)
    lut = _turbo_lut()
    prov = ui.ByteImageProvider()
    win = ui.Window(title, width=w + 20, height=h + 70, position_x=position[0], position_y=position[1])
    with win.frame:
        with ui.VStack(spacing=4):
            ui.ImageWithProvider(prov, fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT)   # fits a docked window
            label = ui.Label(f'red {near:.1f} m ... blue {far:.1f} m (and beyond), black = no depth '
                             f'(device range {lo_dev:g} .. {hi_dev:g} m)', height=16)
    n = {'k': 0}

    def _update(_e):
        n['k'] += 1
        if n['k'] % every:
            return
        try:
            d = cam.get_depth()
        except Exception:
            return
        if d is None or np.asarray(d).ndim != 2:
            return
        d = np.asarray(d, np.float32)
        bad = ~np.isfinite(d) | (d < lo_dev) | (d > hi_dev)
        idx = np.clip((d - near) / (far - near) * 225.0, 0, 225)   # Turbo's last ~10 % is near-black: stop at blue
        idx[bad] = 0
        rgb = lut[idx.astype(np.uint8)]
        rgb[bad] = 0
        rgba = np.concatenate([rgb, np.full(rgb.shape[:2] + (1,), 255, np.uint8)], axis=2)
        hh, ww = rgba.shape[:2]
        try:
            prov.set_data_array(np.ascontiguousarray(rgba), [ww, hh])
        except Exception:
            prov.set_bytes_data(rgba.flatten().tolist(), [ww, hh])
        snap = __import__('os').environ.get('MM_DEPTH_SNAP')          # debug: save one coloured frame (after ~10 s of data)
        if snap and n['k'] > 600 and not n.get('saved'):
            from PIL import Image
            Image.fromarray(rgb).save(snap); n['saved'] = True; print('depth window: saved', snap, flush=True)
        ok = d[~bad]
        if ok.size:
            label.text = (f'red {near:.1f} m ... blue {far:.1f} m, black = no depth   |   nearest {ok.min():.2f} m, '
                          f'median {np.median(ok):.2f} m, {100 * ok.size / d.size:.0f} % of pixels with depth')

    sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_update, name='depth_window')
    return win, sub



def arrange(cam_windows, panel=None, gap=6, panel_next_to=('Render Settings', 'Property', 'Stage')):
    """Camera windows FLOATING in one even row along the bottom of the app window, under the main 'Viewport' (over the
    bottom panel, not the viewport), keeping the camera picture's shape; the script's panel docked as a TAB next to Render
    Settings on the right (2026-11-25, user: "make those 4 align at the bottom and the menu as a tab next to render
    settings"; floating, not docked into the bottom panel). Runs on the app's update events a few frames later (the
    workspace must be laid out first; an asyncio task never ran in these standalone scripts); keep the returned
    subscription alive. MM_UI_SNAP=file.png also saves a picture of the whole app window."""
    import os, traceback, omni.ui as ui, omni.kit.app
    wins = [w for w in cam_windows if w is not None]
    st = {'k': 0, 'docked_to': None}
    snap = os.environ.get('MM_UI_SNAP')

    def _dock_panel():
        for name in panel_next_to:
            tgt = ui.Workspace.get_window(name)
            if tgt is not None and tgt.visible:
                panel.dock_in(tgt, ui.DockPosition.SAME)
                st['docked_to'] = name
                return

    def _place():
        W_main, H_main = ui.Workspace.get_main_window_width(), ui.Workspace.get_main_window_height()
        vp = ui.Workspace.get_window('Viewport')
        if vp is not None and vp.width > 100:
            x0, width, bottom = vp.position_x, vp.width, vp.position_y + vp.height
        else:
            x0, width, bottom = 0.0, W_main * 0.78, H_main * 0.70
        n = max(1, len(wins))
        w = (width - (n + 1) * gap) / n
        h = w * SCENE['res'][1] / SCENE['res'][0] + 52                 # picture + title bar (+ the depth line)
        room = H_main - bottom - 2 * gap
        if room > 150:
            h = min(h, room)                                          # fit under the viewport when there is room
        y = H_main - h - gap
        for k, win in enumerate(wins):
            win.position_x, win.position_y = x0 + gap + k * (w + gap), y
            win.width, win.height = w, h
        print(f'arrange: {n} camera windows {w:.0f} x {h:.0f} at y {y:.0f} (viewport x {x0:.0f}..{x0 + width:.0f}, bottom '
              f'{bottom:.0f}; app {W_main} x {H_main}); panel tab next to {st["docked_to"]}', flush=True)

    def _update(_e):
        st['k'] += 1
        k = st['k']
        try:
            if k == 10 and panel is not None:
                _dock_panel()
            elif k == 15:
                _place()
            elif k == 20 and panel is not None and st['docked_to']:
                panel.focus()                                   # show the menu tab, not Render Settings
            elif snap and k == 150:
                import omni.renderer_capture
                omni.renderer_capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(snap)
            elif snap and k == 160:
                print('arrange: saved', snap, flush=True)
        except Exception:
            traceback.print_exc()
        if k >= 160:
            st.pop('sub', None)                                 # done: drop the subscription

    st['sub'] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_update, name='arrange')
    return st
