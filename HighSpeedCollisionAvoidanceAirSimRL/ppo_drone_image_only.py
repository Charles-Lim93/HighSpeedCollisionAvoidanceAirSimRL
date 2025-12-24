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
                step_length = 0.25,
                image_shape=(120, 120,1),
                env_config=env_config["TrainEnv"]
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
    best_model_save_path="./imageonly_logs",
    log_path="./imageonly_logs",
    eval_freq=1000,
    deterministic=False
)

model = PPO(
    "CnnPolicy",
    env,
    learning_rate=0.0003,
    clip_range= 0.2,
    verbose = 1,
    device="cuda",
    tensorboard_log="./tb_logs/",
)


# model = PPO(
#     "MultiInputPolicy",
#     env,
#     learning_rate=0.003,
#     clip_range= 0.2,
#     verbose = 1,
#     device="cuda",
#    tensorboard_log="./tb_logs/",
# )



# callbacks.append(eval_callback)
# kwargs = {}
# kwargs["callback"] = callbacks

# log_name = "20230521_ppo_run_eps50000_5ms_static_obs_continuous_ImageOnly" + str(time.time())

model.learn(
    total_timesteps=100000,
    callback = eval_callback
)

model.save("20230608_episode100000_speed8ms_static_obs_imageonly")

