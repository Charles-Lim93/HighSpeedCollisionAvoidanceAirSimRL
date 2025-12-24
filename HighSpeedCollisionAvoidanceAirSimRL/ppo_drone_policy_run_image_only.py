import setup_path
import gym
import airgym
import time
import yaml

from stable_baselines3 import PPO

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback

# Get train environmnet configs
with open('./config.yml', 'r') as f:
    env_config = yaml.safe_load(f)
# Create a DummyVecEnv for main airsim gym env

env = DummyVecEnv(
    [
        lambda: Monitor(
            gym.make(
                "airsim-drone-continuous-image-v0",
                ip_address="127.0.0.1",
                step_length = 0.1,
                image_shape=(120, 120,1),
                env_config=env_config["TrainEnv"]
            )
        )
    ]
)
# Wrap env as VecTransposeImage (Channel last to channel first)
env = VecTransposeImage(env)

# Load an existing model
# model = PPO.load(env=env, path="./imageonly_logs/best_model.zip")
model = PPO.load(env=env, path="20230608_episode100000_speed8ms_static_obs_imageonly.zip")

# RUN
obs = env.reset()
# for i in range(1000):
#     action, _ = model.predict(obs, deterministic=True)
#     obs, _, dones, info = env.step(action)

mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)
print(mean_reward, std_reward)
del model


