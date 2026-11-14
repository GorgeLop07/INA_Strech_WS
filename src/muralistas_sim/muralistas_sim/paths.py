"""Ubica el workspace y el modelo del Stretch 3 del repo de Hello Robot (third_party/stretch_mujoco)."""

import os
from pathlib import Path

_MODELS_REL = Path('third_party') / 'stretch_mujoco' / 'stretch_mujoco' / 'models'


def find_workspace_root(start=None):
    """Sube desde este archivo hasta encontrar la carpeta que contiene third_party/stretch_mujoco."""
    env = os.environ.get('INA_STRECH_WS')
    if env and (Path(env) / _MODELS_REL).is_dir():
        return Path(env)
    here = Path(start or __file__).resolve()
    for parent in [here] + list(here.parents):
        if (parent / _MODELS_REL).is_dir():
            return parent
    raise FileNotFoundError(
        'No encontré third_party/stretch_mujoco. Define INA_STRECH_WS con la ruta del workspace.')


def stretch_models_dir(override=''):
    if override:
        return Path(override)
    return find_workspace_root() / _MODELS_REL


def scene_cache_dir(override=''):
    if override:
        path = Path(override)
    else:
        path = find_workspace_root() / 'build' / 'muralistas_sim_scene'
    path.mkdir(parents=True, exist_ok=True)
    return path
