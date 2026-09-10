# High-Speed Drone Collision Avoidance — Spatial-3D (y–z) Simulation

AirSim / Gym environment and PPO training scripts for the **spatial-3D
extension** of the proposed high-speed collision-avoidance method. The agent
controls a 2-D continuous action **[v_y, v_z]** (lateral + vertical velocity)
while the forward speed is held constant; observations are a depth image plus
attitude and safety features.

This folder is self-contained: it does **not** need the rest of the
`HighSpeedCollisionAvoidanceAirSimRL` project.

## Relation to the original code

This folder is the **spatial-3D version** of the original lateral-only (1-D
action) code in the sibling folder `../HighSpeedCollisionAvoidanceAirSimRL/`
(AIAA SciTech 2025). On top of the 3-D extension it contains the **proposed
method** of the JAIS extension: fixed-weight multi-frame depth fusion (three
depth frames fused into one channel) and in-loop safety features
`[P_coll, Delta_r]` that are fed to the policy as an observation and drive a
chance-constraint reward penalty (`safety_injection` in `params.yml`).

## Contents

| Path | Role |
|---|---|
| `airgym/envs/drone_env_vertical.py` | The environment (`AirSimDroneVerticalEnv`), registered as `airsim-drone-vertical-v0` |
| `airgym/envs/airsim_env.py` | Base Gym/AirSim classes |
| `analysis/params.yml` | All tunable parameters (forward speed, action limits, reward, safety ROI) |
| `analysis/params.py` | Loader for `params.yml` |
| `analysis/safety_metrics.py` | Estimates `[P_coll, Delta_r]` from the depth image (part of the observation) |
| `analysis/episode_logger.py` | Per-episode CSV logger used by the evaluation script |
| `config.yml` | Course-section layout (`TrainEnv`) read by all three scripts |
| `smoke_vertical.py` | Quick sanity run (a few steps) — run this first |
| `ppo_drone_vertical.py` | PPO training |
| `ppo_drone_policy_run_vertical.py` | Evaluation of a trained policy |
| `settings.example.json` | AirSim settings that the environment expects |
| `setup_path.py` | Falls back to the installed `airsim` pip package |

## Environment summary

- **Action**: `Box(2,)` = `[v_y, v_z]` (m/s), bounded by
  `vertical.reference_speed` in `params.yml` (default ±10). NED convention:
  `v_z < 0` = climb. Forward speed `v_x` is held constant at
  `vertical.forward_speed` (default 8 m/s). Control step `step_length = 0.1 s`.
- **Observation** (`gym.spaces.Dict`, keys as declared by the env):
  - `"Image"` — depth image declared as `120×120×1` `uint8` (channel-last;
    `VecTransposeImage` in the scripts makes it channel-first for the CNN policy).
    Depth *d* (m) is encoded as **inverse depth** `255 / max(1, d)`, so nearer
    obstacles appear brighter. Values above 205 (obstacles closer than ~1.2 m)
    are saturated to 255 — i.e. the **near** side is clipped, not the far side.
  - `"Linear velocity"` — a `3×3` history of `[v_x, v_y, v_z]`.
  - `"Safety"` — `[P_coll, Delta_r]` from `analysis/safety_metrics.py`. This key
    is present **only when `safety_injection: true`** in `params.yml` (the
    default). Set it to `false` to train/evaluate the no-injection variant.
- **Reward / termination**: defined in the env and `params.yml`
  (goal reached at the course end, collision penalty, manoeuvre cost).

## Setup

1. **AirSim + Unreal**: build/run an AirSim Multirotor map (the paper used
   cylinder / forest / indoor scenes).
2. Copy `settings.example.json` to `~/Documents/AirSim/settings.json`
   (Windows: `C:\Users\<you>\Documents\AirSim\settings.json`). The environment
   reads camera `"0"` **DepthPerspective** at 120×120, FOV 120°, which this file
   configures.
3. Python 3.9 environment:

   ```bash
   pip install torch==2.4.0 --index-url https://download.pytorch.org/whl/cu121   # or CPU build
   pip install -r requirements.txt
   ```

## Run

Start the Unreal/AirSim map and press Play, then from this folder:

```bash
# 1) sanity check — connects to the simulator and steps the env a few times
python smoke_vertical.py

# 2) train PPO (checkpoints / evaluations go to logs_vertical/)
python ppo_drone_vertical.py

# 3) evaluate a trained policy. Expects the checkpoint written by step 2 at
#    ./logs_vertical/best_model.zip (MODEL_PATH in the script), or place a
#    pretrained model there (none is shipped in this repository). Per-episode CSV logs go to
#    logs/episodes/proposed_3d/<seed>/ep<N>.csv
python ppo_drone_policy_run_vertical.py
```

Parameters (forward speed, action limits, reward weights, safety ROI) are
edited in `analysis/params.yml`; no code change is needed.

## Notes

- The simulator must be running and reachable at `LocalHostIp` before any
  script is launched; the env connects over AirSim RPC on construction.
- `ClockSpeed` is `1.0` in the example settings (real-time). Increasing it speeds
  up training but changes control timing.

## Author

Written by **Chulsoo Lim**  
Korea Advanced Institute of Science and Technology  
Department of Aerospace  
Aerospace System and Control Laboratory
