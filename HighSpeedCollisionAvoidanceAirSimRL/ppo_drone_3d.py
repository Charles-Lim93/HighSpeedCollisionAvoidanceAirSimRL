import setup_path
import gym
import airgym
import time
import yaml
import numpy as np
from stable_baselines3 import PPO

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
                "airsim-drone-sample-continuous-v0",
                ip_address="127.0.0.1",
                step_length = 0.1,
                image_shape=(120, 120, 3),
                env_config=env_config["TrainEnv"],
                attitude_shape = (3,2)
            )
        )
    ]
)

# Wrap env as VecTransposeImage (Channel last to channel first)
env = VecTransposeImage(env)

# Evaluation callback
callbacks = []
eval_callback = EvalCallback(
    env,
    n_eval_episodes=5,
    best_model_save_path="./logs",
    log_path="./logs",
    eval_freq=1000,
    deterministic=False
)

model = PPO(
    "MultiInputPolicy",
    env,
    learning_rate=0.0003,
    clip_range= 0.2,
    verbose = 1,
    device="cuda",
   tensorboard_log="./tb_logs/",
)


model.learn(
    total_timesteps=100000,
    callback=eval_callback
)

model.save("ppo_airsim_2025112_100000_8ms_3d_indoor")
print("Model saved.")
