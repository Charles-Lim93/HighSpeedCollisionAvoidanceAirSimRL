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
from stable_baselines3.common.callbacks import EvalCallback

# NumPy 2.0 이상 호환성 패치
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


# Get train environmnet configs
with open('./config.yml', 'r') as f:
    env_config = yaml.safe_load(f)
# Create a DummyVecEnv for main airsim gym env
env = DummyVecEnv(
    [
        lambda: Monitor(
            gym.make(
                "airsim-drone-sample-continuous-1d-v0",
                ip_address="127.0.0.1",
                step_length = 0.25,
                image_shape=(120, 120, 1),
                env_config=env_config["TrainEnv"],
                attitude_shape = (3,2)
            )
        )
    ]
)
# Wrap env as VecTransposeImage (Channel last to channel first)
env = VecTransposeImage(env)

# Load an existing model
# model = PPO.load(env=env, path="./1dlogs/20230606/10000_2ms.zip")
# model = PPO.load(env=env, path="./ppo_airsim_drone_policy_20251104_episode200000_speed8ms_1d_indoor.zip")
# model = PPO.load(env=env, path="./ppo_airsim_drone_policy_20251118_episode120000_speed8ms_1d_forest.zip")
model = PPO.load(env=env, path="./1dlogs/best_model.zip")

# RUN
obs = env.reset()
# for i in range(1000):
#     action, _ = model.predict(obs, deterministic=True)
#     obs, _, dones, info = env.step(action)

mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)
print(mean_reward, std_reward)

