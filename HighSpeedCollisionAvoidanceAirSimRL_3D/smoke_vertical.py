"""1-episode smoke test for the vertical (spatial-3D) env.

Drives the env directly with random actions (no trained model) to check that:
  * the env connects to the simulator and is constructed
  * action.shape == (2,)            [v_y, v_z]
  * obs includes vz (Linear velocity (3,3)) and, when injection is on, a Safety key
  * reward computation and episode CSV logging work

Run:  (start the AirSim Unreal map first)
    (activate your Python environment)
    python smoke_vertical.py
"""
import setup_path
import gym
import airgym
import yaml
import numpy as np

# NumPy 2.0 compatibility shim (same as the other scripts)
for _n, _v in [('float', float), ('int', int), ('bool', bool),
               ('object', object), ('long', int), ('complex', complex)]:
    if not hasattr(np, _n):
        setattr(np, _n, _v)

from analysis import episode_logger
from analysis.params import load_params

P = load_params()
print("safety_injection =", P["safety_injection"])

with open('./config.yml', 'r') as f:
    env_config = yaml.safe_load(f)

env = gym.make(
    "airsim-drone-vertical-v0",
    ip_address="127.0.0.1",
    step_length=0.1,
    image_shape=(120, 120, 1),
    env_config=env_config["TrainEnv"],
    attitude_shape=(3, 3),
    is_test=True,        # logging ON -> logs/episodes/proposed_3d/999/ep1.csv
    run_seed=999,
)

print("action_space :", env.action_space)
print("obs_space keys:", list(env.observation_space.spaces.keys()))
assert env.action_space.shape == (2,), "action is not 2-D!"

obs = env.reset()
print("reset obs keys:", list(obs.keys()))
print("  Image          :", obs["Image"].shape, obs["Image"].dtype)
print("  Linear velocity:", obs["Linear velocity"].shape, "(includes vz -> last column)")
if "Safety" in obs:
    print("  Safety [P_coll, Delta_r]:", obs["Safety"])

MAX_STEPS = 60
total_reward = 0.0
for t in range(MAX_STEPS):
    action = env.action_space.sample()        # random [v_y, v_z]
    obs, reward, done, info = env.step(action)
    total_reward += reward
    if t < 3 or done:
        print("t=%2d action=[% .2f % .2f] reward=% .3f done=%s collision=%s"
              % (t, action[0], action[1], reward, done, info.get("collision")))
    if done:
        print("episode terminated at t=%d" % t)
        break

print("total_reward over smoke episode = %.3f" % total_reward)
episode_logger.close()
print("log written: logs/episodes/proposed_3d/999/ep1.csv")
print("SMOKE DONE")
