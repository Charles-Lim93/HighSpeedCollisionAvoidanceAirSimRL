"""Spatial-3D extension of the proposed (1-channel temporal-fusion) algorithm.

The proposed 1D lateral-avoidance policy is extended to a 2D
y-z continuous-acceleration controller (NOT full 6-DOF). It extends the
lateral-only (1-D) proposed environment with:

  * Action 1D -> 2D                     (v_y, v_z)
  * Reward  Delta_y -> Delta_r          (radial y-z progress)
  * Obs adds v_z: attitude (3,2)->(3,3) [vx, vy, vz]
  * reset() randomizes the initial altitude

The 1-channel temporal fusion (0.2/0.3/0.5 -> (120,120,1)) and the whole depth
pipeline are kept UNCHANGED, and the corrected reward / normalization scheme is
applied here.

Safety injection (P_coll obs key + chance-constraint penalty) is wired
in and gated by analysis/params.yml -> safety_injection. Eval-time
episode logging is wired into step().

method_id = "proposed_3d".
"""
import setup_path
import airsim
import numpy as np
import math
import time
from time import time
from argparse import ArgumentParser
import tempfile
import pprint
import cv2
import os
import sys

import gym
from gym import spaces
import queue
from queue import Queue
from airgym.envs.airsim_env import AirSimEnv
from airgym.envs.airsim_env import AirSimDictObsEnv
from PIL import Image

# Make the project-root `analysis` package importable regardless of cwd.
_PROJ_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
if _PROJ_ROOT not in sys.path:
    sys.path.insert(0, _PROJ_ROOT)
from analysis.params import load_params
from analysis import safety_metrics, episode_logger


class AirSimDroneVerticalEnv(AirSimDictObsEnv):
    def __init__(self, ip_address, step_length, image_shape, env_config, attitude_shape,
                 is_test=False, run_seed=0):
        super().__init__(image_shape, attitude_shape)
        self.is_test = is_test
        self.run_seed = run_seed
        self.sections = env_config["sections"]
        number_of_batch = 3

        # ---- tunables (single source: analysis/params.yml) -----------------
        self._params = load_params()
        self.method_id = "proposed_3d"
        self.safety_injection = bool(self._params.get("safety_injection", True))
        reference_speed = float(self._params["vertical"]["reference_speed"])

        self.drone = airsim.MultirotorClient(ip=ip_address)
        # Continuous 2D action space: [v_y, v_z]
        self.action_space = spaces.Box(low=-reference_speed, high=reference_speed,
                                       shape=(2,), dtype=np.float32)
        self.image_request = airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False)
        self.drone_state = self.drone.getMultirotorState()
        self.step_length = step_length
        self.image_shape = image_shape
        self.attitude_shape = attitude_shape
        self.start_ts = 0
        self.image_queue = Queue(number_of_batch)
        self.attitude_queue = Queue(number_of_batch)

        # Extend the (base-defined) Dict obs with a Safety key,
        # ONLY when injection is on. The env owns its own observation_space;
        # MultiInputPolicy auto-handles the extra key.
        if self.safety_injection:
            self.observation_space = spaces.Dict({
                **self.observation_space.spaces,
                "Safety": spaces.Box(low=0.0, high=1.0, shape=(2,), dtype=np.float64),  # [P_coll, Delta_r]
            })

        # episode-log bookkeeping (eval-time logging only)
        self.episode = 0
        self.t = 0
        self._last_p_coll = 0.0
        self._last_delta_r = 1.0

        print("------------ AIRSIM VERTICAL (SPATIAL-3D) ENVIRONMENT ------------")
        print("------------ OBS: 1ch FUSED DEPTH / [vx,vy,vz] / (Safety) ------------")
        print("------------ ACTION: [v_y, v_z]  method_id=proposed_3d ------------")
        print("------------ safety_injection =", self.safety_injection, "------------")

        self.state = {
            "position": np.zeros(3),
            "prev_position": np.zeros(3),
            "pose": None,
            "prev_pose": None,
            "Angular velocity": np.zeros(3),
            "collision": False,
        }
        self.info = {"collision": False}
        self._setup_drone()

    def _setup_drone(self):
        self.drone.reset()
        self.drone.enableApiControl(True)
        self.drone.armDisarm(True)
        self.collision_time = self.drone.simGetCollisionInfo().time_stamp

    def step(self, action):
        self._do_action(action)
        obs, info = self._get_obs()
        reward, done = self._compute_reward(obs)

        # Eval-time episode logging (decoupled from training graph).
        if self.is_test:
            a_y_cmd = round(float(action[0]), 3)
            a_z_cmd = round(float(action[1]), 3)
            speed = round(float(self.drone_state.kinematics_estimated.linear_velocity.x_val), 3)
            episode_logger.log(
                self.method_id, self.t, speed,
                self._last_p_coll, a_y_cmd, a_z_cmd, self._last_delta_r,
                self.info.get("collision", False), self.run_seed, self.episode,
                log_root=self._params.get("episode_log", {}).get("log_root", "logs/episodes"),
            )
            self.t += 1

        return obs, reward, done, info

    def reset(self):
        number_of_batch = 3
        self._setup_drone()
        self.drone.simFlushPersistentMarkers()
        self.image_queue = Queue(number_of_batch)
        self.attitude_queue = Queue(number_of_batch)

        # Randomize initial altitude about z_target (NED), clipped to range.
        z_target = float(self._params["vertical"]["z_target"])
        z_lo, z_hi = self._params["vertical"]["z_init_range"]
        z_off = float(np.random.uniform(float(z_lo), float(z_hi)))
        x_pos = 1.0
        y_pos = (np.random.randint(11) - 5)
        z_pos = z_target + z_off
        pose = airsim.Pose(airsim.Vector3r(x_pos, y_pos, z_pos))
        self.drone.simSetVehiclePose(pose=pose, ignore_collision=True)

        self.last_plot_pos = self.drone.getMultirotorState().kinematics_estimated.position
        self.info["is_success"] = False
        if self.is_test:
            self.episode += 1
        self.t = 0

        # settle at the randomized altitude, then begin 2D velocity control
        self.drone.moveByVelocityZAsync(0, 0, z_pos, 1).join()
        initial_action = np.array([float(np.random.randint(11) - 5), 0.0], dtype=np.float32)
        self._do_action(initial_action)
        obs, _ = self._get_obs()
        return obs

    def __del__(self):
        # Guard against shutdown noise: at interpreter exit the RPC socket may
        # already be closed, so reset() can raise (WinError 10038). Harmless.
        try:
            self.drone.reset()
        except Exception:
            pass

    def _do_action(self, action):
        ts = 0.1
        vcfg = self._params["vertical"]
        fwd = float(vcfg["forward_speed"])
        v_y = round(float(action[0]), 3)
        v_z = round(float(action[1]), 3)   # NED convention: v_z < 0 -> climb (up)

        # Optional command clipping (gate + logging via analysis script)
        act_cfg = self._params.get("action", {})
        if act_cfg.get("clip_commands", False):
            v_y = float(np.clip(v_y, -act_cfg["a_y_max"], act_cfg["a_y_max"]))
            v_z = float(np.clip(v_z, -act_cfg["a_z_max"], act_cfg["a_z_max"]))

        # Altitude hard bounds: stop vertical escape. NED -> negative = up,
        # v_z < 0 climbs, v_z > 0 descends. At/above the ceiling we kill any
        # further climb; at/below the floor we kill any further descent.
        if vcfg.get("enforce_altitude_bounds", True):
            z_now = self.drone.getMultirotorState().kinematics_estimated.position.z_val
            z_ceiling = float(vcfg["z_ceiling"])   # most negative (highest) allowed
            z_floor = float(vcfg["z_floor"])       # least negative (lowest) allowed
            if z_now <= z_ceiling and v_z < 0:
                v_z = 0.0
            if z_now >= z_floor and v_z > 0:
                v_z = 0.0

        self.drone.moveByVelocityAsync(
            fwd, v_y, v_z, ts,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            # ABSOLUTE heading lock at 0 deg (+x): rate-0 default let collision
            # bumps leave the body-fixed camera pointing off-track.
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0),
        ).join()

    def transform_obs(self, response):
        img1d = np.array(response.image_data_float, dtype=np.float)
        try:
            img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
        except ZeroDivisionError:
            print("ZeroDivisionError")
            print("WINTER CONTINGENCY PLAN")
            img1d = np.ones(img1d.size)
        if img1d.size == 14400:
            img2d = np.reshape(img1d, (120, 120))
        else:
            img2d = np.zeros([120, 120])
            print("ERROR____WINTER CONTRINGENCY")
        image = Image.fromarray(img2d)
        im_final = np.array(image.convert("L"))
        try:
            return im_final
        except:
            return np.zeros((self.image_shape[0], self.image_shape[1]))

    def _image_queue(self, image_to_queue):
        queue = self.image_queue
        if queue.full() == True:
            return queue
        else:
            queue.put(image_to_queue)
            return queue

    def _attitude_queue(self, attitude_to_queue):
        queue = self.attitude_queue
        if queue.full() == True:
            return queue
        else:
            queue.put(attitude_to_queue)
            return queue

    def _get_obs(self):
        # normalization references (single source: params.yml).
        norm_vx = float(self._params["vertical"]["forward_speed"])
        norm_vy = float(self._params["vertical"]["reference_speed"])
        norm_vz = float(self._params["action"]["a_z_max"])

        image_batch = np.zeros([3, self.image_shape[0], self.image_shape[1]])
        # Attitude batch is (3,3): [vx, vy, vz]
        attitute_batch = np.zeros([3, 3])
        responses = self.drone.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True)])

        attitute_responses = self.drone.getMultirotorState()
        lv = attitute_responses.kinematics_estimated.linear_velocity
        attitute = [round(lv.x_val, 3), round(lv.y_val, 3), round(lv.z_val, 3)]

        attitute_responses_queue = self._attitude_queue(attitute)
        if attitute_responses_queue.full() == True:
            attitute = attitute_responses_queue.get()
            attitute_batch[0, :] = np.array(list(attitute))
            attitute = attitute_responses_queue.get()
            attitute_batch[1, :] = np.array(list(attitute))
            attitute = attitute_responses_queue.get()
            attitute_batch[2, :] = np.array(list(attitute))

            self._attitude_queue(attitute_batch[1, :])
            self._attitude_queue(attitute_batch[2, :])
            input_attitute = attitute_batch
        else:
            input_attitute = np.zeros([3, 3])
            for i in range(3):
                input_attitute[i, :] = attitute

        # ---- 1ch temporal fusion (UNCHANGED depth pipeline) ----------------
        image = self.transform_obs(responses[0])
        image3d = np.zeros([1, self.image_shape[0], self.image_shape[1]])
        image3d[0, :, :] = image
        image_queue = self._image_queue(image3d)
        if image_queue.full() == True:
            for i in range(image_queue.maxsize):
                image = image_queue.get()
                image_batch[i, :, :] = np.array(list(image))
            self._image_queue(image_batch[1, :, :])
            self._image_queue(image_batch[2, :, :])
            final_image = image_batch[0, :, :] * 0.2 + image_batch[1, :, :] * 0.3 + image_batch[2, :, :] * 0.5
        else:
            for i in range(3):
                final_image = image

        input_image = final_image
        input_image = np.where(final_image > 205, 255, final_image)
        input_image = input_image.reshape(120, 120, 1)
        input_image = input_image.astype(np.uint8)

        self.drone_state = self.drone.getMultirotorState()
        self.state["prev_pose"] = self.state["pose"]
        self.state["pose"] = self.drone_state.kinematics_estimated
        self.state["collision"] = self.drone.simGetCollisionInfo().has_collided
        self.info["collision"] = self.is_collision()
        if self.info["collision"] == True:
            input_attitute[attitute_responses_queue.maxsize - 1, :] = input_attitute[attitute_responses_queue.maxsize - 2, :]

        # Proper per-column scaling (no row/col mixup,
        # no re-reference of an already-normalized column).
        input_attitute[:, 0] = input_attitute[:, 0] / norm_vx   # vx
        input_attitute[:, 1] = input_attitute[:, 1] / norm_vy   # vy
        input_attitute[:, 2] = input_attitute[:, 2] / norm_vz   # vz

        obs = {"Image": input_image, "Linear velocity": input_attitute}

        # Safety estimate from the FUSED depth the policy observes.
        p_coll, delta_r = safety_metrics.estimate(input_image, self._params)
        self._last_p_coll = p_coll
        self._last_delta_r = delta_r
        # Expose to the policy only when injection is on.
        if self.safety_injection:
            obs["Safety"] = np.array([p_coll, delta_r], dtype=np.float64)

        return obs, self.info

    def _compute_reward(self, obs):
        reward = 0
        done = 0
        P = self._params
        goal_point = P["goal_point"]
        reference_speed = float(P["vertical"]["reference_speed"])
        z_target = float(P["vertical"]["z_target"])
        z_scale = float(P["vertical"]["z_scale"])
        cc = P["chance_constraint"]
        delta = float(cc["delta"])
        lambda_cc = float(cc["lambda_cc"])

        self.state["position"] = self.drone_state.kinematics_estimated.position
        pos = self.state["position"]
        traveled_distance = math.sqrt(pos.x_val ** 2)
        forward_speed = self.drone_state.kinematics_estimated.linear_velocity.x_val

        # Delta_y -> Delta_r : radial progress to the y-z center line.
        dy = pos.y_val / goal_point[1]
        dz = (pos.z_val - z_target) / z_scale
        go_to_center = round(1 - (dy ** 2 + dz ** 2), 5)

        # Distance term is SQUARED (not sqrt); weights
        # k1=10 / k2=5 / k3=2.5; the legacy *1000 residue is removed.
        reward_distance = round((traveled_distance / goal_point[0]) ** 2, 5)
        action_accelerator = round((forward_speed / reference_speed) ** 2, 5)
        k1, k2, k3 = 10.0, 5.0, 2.5

        if self.state["collision"]:
            done = 1
            collision_penalty = -20
            reward = reward_distance * k1 + action_accelerator * k2 + go_to_center * k3 + collision_penalty

        if pos.x_val > 105:
            done = 1
            goal_incentive = 50
            reward = reward_distance * k1 + action_accelerator * k2 + go_to_center * k3 + goal_incentive
            self.info["is_success"] = True
            print("Reached Goal, TOTAL Reward :", reward)

        # Optional dense altitude-bound penalty (soft, complements the hard
        # clamp in _do_action). Off when z_penalty_w == 0.
        z_pen_w = float(P["vertical"].get("z_penalty_w", 0.0))
        if z_pen_w > 0.0:
            z_ceiling = float(P["vertical"]["z_ceiling"])
            z_floor = float(P["vertical"]["z_floor"])
            over = max(0.0, z_ceiling - pos.z_val) + max(0.0, pos.z_val - z_floor)
            reward = reward - z_pen_w * over

        # Dense chance-constraint penalty -> in-loop coupling.
        if self.safety_injection:
            reward = reward - lambda_cc * max(0.0, self._last_p_coll - delta)

        return reward, done

    def is_collision(self):
        current_collision_time = self.drone.simGetCollisionInfo().time_stamp
        return True if current_collision_time != self.collision_time else False


class TestEnv(AirSimDroneVerticalEnv):

    def __init__(self, ip_address, step_length, image_shape, env_config, attitude_shape, run_seed=0):
        self.eps_n = 0
        super(TestEnv, self).__init__(ip_address, step_length, image_shape, env_config,
                                      attitude_shape, is_test=True, run_seed=run_seed)
        self.agent_traveled = []

    def setup_flight(self):
        super(TestEnv, self)._setup_drone()
        self.eps_n += 1
