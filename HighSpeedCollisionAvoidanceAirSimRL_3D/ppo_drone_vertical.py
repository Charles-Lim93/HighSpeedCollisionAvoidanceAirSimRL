"""Train the spatial-3D (vertical) proposed policy.

PPO training script for the vertical env:
  * env id          : airsim-drone-vertical-v0
  * attitude_shape  : (3,3)  -> [vx, vy, vz]
  * action          : (2,)   -> [v_y, v_z]     (MultiInputPolicy handles it)

safety_injection is controlled by analysis/params.yml.
"""
import setup_path
import gym
import airgym
import time
import yaml
import sys
import numpy as np
from stable_baselines3 import PPO

# Optional: pass total timesteps as argv[1] for a quick sanity run, e.g.
#   python ppo_drone_vertical.py 3000
# Omit for the full run (100000).
TOTAL_TIMESTEPS = int(sys.argv[1]) if len(sys.argv) > 1 else 100000

# NumPy 2.0 compatibility shims
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int
if not hasattr(np, 'bool'):
    np.bool = bool
if not hasattr(np, 'object'):
    np.object = object
if not hasattr(np, 'long'):
    np.long = int
if not hasattr(np, 'complex'):
    np.complex = complex

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback

# Get train environment configs
with open('./config.yml', 'r') as f:
    env_config = yaml.safe_load(f)

# Create a DummyVecEnv for the vertical (spatial-3D) airsim gym env
env = DummyVecEnv(
    [
        lambda: Monitor(
            gym.make(
                "airsim-drone-vertical-v0",
                ip_address="127.0.0.1",
                step_length=0.1,
                image_shape=(120, 120, 1),
                env_config=env_config["TrainEnv"],
                attitude_shape=(3, 3),       # vx, vy, vz
            )
        )
    ]
)

# Wrap env as VecTransposeImage (channel last -> channel first)
env = VecTransposeImage(env)

# Evaluation callback
eval_callback = EvalCallback(
    env,
    n_eval_episodes=5,
    best_model_save_path="./logs_vertical",
    log_path="./logs_vertical",
    eval_freq=1000,
    deterministic=False,
)

model = PPO(
    "MultiInputPolicy",   # auto-handles Dict obs (Image / Linear velocity / Safety) + 2D action
    env,
    learning_rate=0.0003,
    clip_range=0.2,
    verbose=1,
    device="cuda",
    tensorboard_log="./tb_logs/",
)

print("Training proposed_3d for total_timesteps =", TOTAL_TIMESTEPS)
model.learn(
    total_timesteps=TOTAL_TIMESTEPS,
    callback=eval_callback,
)

model.save("ppo_airsim_drone_policy_vertical_proposed_3d")
print("Model saved.")
