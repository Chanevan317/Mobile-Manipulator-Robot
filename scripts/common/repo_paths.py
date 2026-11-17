"""Repo locations + import path for every script under scripts/.

Each entry script starts with
    sys.path.insert(0, os.path.join(<its folder>, '..', 'common'))   # ('..', '..', 'common' two levels down)
    import repo_paths as RP
after which every script folder (common, tasks/mono, tasks/dual, scenes/scene_v0, ...) is importable, and these
paths point into the repo wherever it is cloned:
"""
import os, sys

SCRIPTS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
REPO = os.path.dirname(SCRIPTS)
PKG = os.path.join(REPO, 'mobile_manipulator_description')     # the robot description (URDF, meshes, generator)
URDF_DIR = os.path.join(PKG, 'urdf')
GENERATOR = os.path.join(PKG, 'generator')
SCENES = os.path.join(REPO, 'scenes')                           # scene data: scene_v0/ (usda, stations.json), assets/
SIMREADY = os.path.join(SCENES, 'assets', 'simready')           # downloaded NVIDIA SimReady assets (git-ignored)


def urdf(layout='prototype_mono_arm', ros=False):
    """Path of the robot URDF for a layout ('prototype_mono_arm' = the official one-arm prototype, 'prototype' = dual)."""
    return os.path.join(URDF_DIR, f'mobile_manipulator_{layout}{"_ros" if ros else ""}.urdf')


def scene_dir(version='scene_v0'):
    return os.path.join(SCENES, version)


for _root, _dirs, _files in os.walk(SCRIPTS):
    _dirs[:] = sorted(d for d in _dirs if not d.startswith(('.', '__')))
    if any(f.endswith('.py') for f in _files) and _root not in sys.path:
        sys.path.append(_root)
