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

SCENE: Luxonis OAK-D Pro, auto-focus version (docs.luxonis.com, OAK-D Pro page, 'Variant: Auto Focus'):
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

# Luxonis OAK-D Pro W, OV9782 version (2026-11-18, candidate: the AF's 66 deg was too narrow): colour OV9782 1 MP 1280 x 800,
# global shutter, fixed focus, listed 150 / 127 / 80 deg D / H / V (Luxonis spec sheet 150 / 128 / 80); stereo OV9282 with the
# same wide view. An equidistant fisheye (r = f * theta) with f = 640 / 63.5 deg = 577 px at 1280 x 800 gives exactly
# 127 / 79.4 / 149.7 deg, so k1..k4 = 0 matches the listed numbers (replace with the unit's factory calibration).
# Rendered at 640 x 400 (half resolution, same 16:10 shape).
_F_W = 320.0 / math.radians(63.5)
SCENE_PRO_W = dict(link='scene_camera_optical', res=(640, 400), assumed=False, hfov=127.0,
                   fx=_F_W, fy=_F_W, cx=320.0, cy=200.0, k=[0.0, 0.0, 0.0, 0.0], depth=(0.2, 12.0), forward=0.0,
                   model='Luxonis OAK-D Pro W (OV9782 colour, global shutter, 127 x 80 deg fisheye) + depth aligned to it')
SCENE_MODELS = {'pro_af': SCENE_PRO_AF, 'pro_w': SCENE_PRO_W}
SCENE_MODEL = __import__('os').environ.get('MM_SCENE_CAM', 'pro_af')     # MM_SCENE_CAM=pro_w to try the wide camera
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
    return c


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


def capture(task, cams, renders=2, max_renders=60, settle=90):
    """Render and read every camera. Headless runs step physics without rendering, so this renders first: `renders`
    frames (the annotators lag a frame), and on the first call until every camera has data (the render pipeline needs
    a few frames to warm up). Returns {'<name>_rgb': uint8 HxWx3, 'scene_depth': float32 HxW (m, 0 = no depth)}."""
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
