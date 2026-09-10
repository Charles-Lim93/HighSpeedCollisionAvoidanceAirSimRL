"""Shared per-episode CSV logger.

ALL method envs import THIS SAME function and call it from step() only when
``self.is_test`` is True. The logging path is fully decoupled from the policy /
training graph: it just records what happened during eval rollouts.

One CSV per (method, seed, episode):
    {log_root}/{method}/{seed}/ep{N}.csv

Columns:
    t, speed, P_coll, a_y_cmd, a_z_cmd, delta_r, collided, method, seed, episode

Notes
-----
* ``a_z_cmd`` is written as "" (N/A) by lateral-only (1-D action) envs;
  the spatial-3D env (proposed_3d) commands v_z.
* ``collided`` is 0/1.
* Files are kept open and flushed per row so partial runs are still readable;
  call close() at the end of an eval script to release handles.
"""
import os
import csv

COLUMNS = ["t", "speed", "P_coll", "a_y_cmd", "a_z_cmd",
           "delta_r", "collided", "method", "seed", "episode"]

_OPEN = {}   # (method, seed, episode) -> (file_handle, csv_writer)


def _na(v):
    return "" if v is None else v


def log(method, t, speed, p_coll, a_y_cmd, a_z_cmd, delta_r, collided,
        seed, episode, log_root="logs/episodes"):
    key = (str(method), str(seed), int(episode))
    if key not in _OPEN:
        d = os.path.join(log_root, str(method), str(seed))
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, "ep%d.csv" % int(episode))
        f = open(path, "w", newline="", encoding="utf-8")
        w = csv.writer(f)
        w.writerow(COLUMNS)
        _OPEN[key] = (f, w)
    f, w = _OPEN[key]
    w.writerow([t, speed, p_coll, _na(a_y_cmd), _na(a_z_cmd),
                delta_r, int(bool(collided)), method, seed, episode])
    f.flush()


def close():
    """Close all open log files. Call at the end of an eval script."""
    for f, _ in _OPEN.values():
        try:
            f.close()
        except Exception:
            pass
    _OPEN.clear()
