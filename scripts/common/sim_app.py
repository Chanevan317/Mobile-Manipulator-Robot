"""Start Isaac Sim's SimulationApp the same way in every script: in a window, at the screen's full resolution.

    import sim_app
    app = sim_app.start(headless, res='')         # res 'WxH' overrides the size (scripts with a --res option)

Isaac's default renders 1280 x 720 and stretches it to the window, which looked soft on the user's 2560 x 1440 screen
(2026-11-16); the scene scripts sized the window to the screen first, every script does since 2026-11-23. The GPU has
plenty of room (the CPU-side physics sets the frame rate). Headless runs keep Isaac's defaults.
"""
import re
import subprocess


def screen_size(res=''):
    """(W, H): res 'WxH' if given, else the primary screen's size from xrandr, else 1920 x 1080."""
    if res:
        w, h = res.lower().split('x')
        return int(w), int(h)
    try:
        out = subprocess.run(['xrandr'], capture_output=True, text=True, timeout=5).stdout
        m = re.search(r' connected primary (\d+)x(\d+)', out) or re.search(r' connected (\d+)x(\d+)', out)
        return int(m.group(1)), int(m.group(2))
    except Exception:
        return 1920, 1080


def start(headless=False, res='', **extra):
    """SimulationApp({'headless': ...}), sized to the screen when it has a window. extra: more SimulationApp settings."""
    from isaacsim import SimulationApp
    cfg = {'headless': headless, **extra}
    if not headless:
        W, H = screen_size(res)
        cfg.update(width=W, height=H, window_width=W, window_height=H)
    return SimulationApp(cfg)
