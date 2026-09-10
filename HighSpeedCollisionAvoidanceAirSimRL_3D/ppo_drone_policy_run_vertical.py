"""Evaluate the vertical (spatial-3D) proposed policy.

Evaluation script for the vertical env. Crucially it passes
``is_test=True`` so the env's eval-time logging hook fires, writing
logs/episodes/proposed_3d/{seed}/ep{N}.csv. Set ``RUN_SEED`` per eval run.
"""
import setup_path
import gym
import airgym
import time
import yaml
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.evaluation import evaluate_policy

# NumPy 2.0 compatibility shims
for _name, _val in [('float', float), ('int', int), ('bool', bool),
                    ('object', object), ('long', int), ('complex', complex)]:
    if not hasattr(np, _name):
        setattr(np, _name, _val)

from analysis import episode_logger

RUN_SEED = 0            # <-- change per eval run; becomes the {seed} log dir
N_EVAL_EPISODES = 10
MODEL_PATH = "./logs_vertical/best_model.zip"

with open('./config.yml', 'r') as f:
    env_config = yaml.safe_load(f)

env = DummyVecEnv(
    [
        lambda: Monitor(
            gym.make(
                "airsim-drone-vertical-v0",
                ip_address="127.0.0.1",
                step_length=0.1,
                image_shape=(120, 120, 1),
                env_config=env_config["TrainEnv"],
                attitude_shape=(3, 3),
                is_test=True,
                run_seed=RUN_SEED,
            )
        )
    ]
)
env = VecTransposeImage(env)

model = PPO.load(env=env, path=MODEL_PATH)

obs = env.reset()
mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=N_EVAL_EPISODES)
print("mean_reward", mean_reward, "std_reward", std_reward)

episode_logger.close()
del model
print("Done. Logs under logs/episodes/proposed_3d/%d/" % RUN_SEED)
