"""Canonical loader for analysis/params.yml.

All env / training / analysis code reads its tunables from a single YAML file
(no hardcoding elsewhere). Usage:

    from analysis.params import load_params
    P = load_params()
    delta = P["chance_constraint"]["delta"]

The loader searches a few sensible locations so it works whether code is run
from the project root (the usual case: ``python ppo_drone_vertical.py``) or
imported from within the ``airgym`` package.
"""
import os
import yaml

_CACHE = {}


def _search_paths():
    here = os.path.dirname(os.path.abspath(__file__))
    return [
        os.path.join(os.getcwd(), "analysis", "params.yml"),   # run from project root
        os.path.join(here, "params.yml"),                      # next to this module
        os.path.normpath(os.path.join(here, "..", "analysis", "params.yml")),
    ]


def find_params_path():
    for c in _search_paths():
        if os.path.exists(c):
            return c
    raise FileNotFoundError(
        "analysis/params.yml not found; searched: %r" % (_search_paths(),)
    )


def load_params(path=None, use_cache=True):
    """Return the parsed params dict. Cached by resolved path."""
    if path is None:
        path = find_params_path()
    if use_cache and path in _CACHE:
        return _CACHE[path]
    with open(path, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f)
    _CACHE[path] = params
    return params


if __name__ == "__main__":
    import pprint
    pprint.pprint(load_params())
