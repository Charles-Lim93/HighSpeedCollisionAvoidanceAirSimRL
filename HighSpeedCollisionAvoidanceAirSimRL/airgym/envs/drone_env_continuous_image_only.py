import setup_path
import airsim
import numpy as np
import math
import time
from argparse import ArgumentParser

import gym
from gym import spaces
from airgym.envs.airsim_env import AirSimEnv
from airgym.envs.airsim_env import AirSimContinuousImageOnlyEnv
from PIL import Image


class AirSimDroneContinuousImageOnlyEnv(AirSimContinuousImageOnlyEnv):
    def __init__(self, ip_address, step_length, image_shape, env_config):
        super().__init__(image_shape)
        self.sections = env_config["sections"]
        
        self.drone = airsim.MultirotorClient(ip = ip_address)
        # Continuous Action Space
        reference_speed = 5

        self.action_space = spaces.Box(low=-reference_speed, high=reference_speed, shape=(1,),dtype=np.float32)
        self.image_request = airsim.ImageRequest("0", 
                                                 airsim.ImageType.DepthPerspective, 
                                                 True, 
                                                 False)
        self.drone_state = None

        self.step_length = step_length
        self.image_shape = image_shape
        self.start_ts = 0

        self.state = {
            "position": np.zeros(3),
            "prev_position": np.zeros(3),
            "pose": None,
            "prev_pose": None,
            "collision": False,
        }
        self.info = {"collision": False}
        
        # self.observation_space = spaces.Box(0, 255, shape=image_shape, dtype=np.uint8)

        # self.observation_space = spaces.Dict(
        #     {
        #         # Sequential Image Data
        #         "Image": spaces.Box(low=0, high=255, shape=image_shape, dtype=np.uint8),
        #         # Sequential Attitude Data
        #         # "vec": spaces.Box(low=-10, high=10, shape=(3,), dtype=np.float32),  
        #     }
        # )

        # self.observation_space = spaces.Dict(
        #     {
        #         # Sequential Image Data
        #         "Image": spaces.Box(low=0, high=255, shape=image_shape, dtype=np.uint8)
        #         # Sequential Attitude Data
        #         # "vec": spaces.Box(low=-10, high=10, shape=(3,), dtype=np.float32),  
        #     }
        # )
        print("----------------WELCOME----------------")
        print("AIRSIM_CONTINUOUS_IMAGE_ONLY_ENVIRONMENTS_VERSION_1.0")
        print("CREATED BY CHARLES LIM FROM KOREA ADVANCED INSTITUTE OF SCIENCE AND TECHNOLOGY")
        
        self._setup_drone()
    
    
    def _setup_drone(self):
        self.drone.reset()
        self.drone.enableApiControl(True)
        self.drone.armDisarm(True)        
        # Get collision time stamp
        self.collision_time = self.drone.simGetCollisionInfo().time_stamp
        
        
    def step(self, action):
        self._do_action(action)
        obs, info = self._get_obs()
        reward, done = self._compute_reward(obs)
        return obs, reward, done, info

    def reset(self):
        self._setup_drone()

        x_pos = 1.0
        y_pos = (np.random.randint(11) - 5)
        # y_pos = 0.0
        z_pos = 0.0
        pose = airsim.Pose(airsim.Vector3r(x_pos, y_pos, z_pos))
        self.drone.simSetVehiclePose(pose=pose, ignore_collision=True)
        initial_action = np.random.randint(9) -4 
        self.drone.moveByVelocityZAsync(0,0,-2.0,1).join()
        self._do_action(initial_action)
        obs, _ = self._get_obs()

        return obs

    def __del__(self):
        self.drone.reset()

    def _do_action(self, action):
        timestep = 0.1
        # reference_speed = 2

        action = round(float(action),3)
        self.drone.moveByVelocityZAsync(9, action,-2.0,timestep, drivetrain=1).join()
        # print("Action Command : ",action)
        
           
    def transform_obs(self, response):
        # img1d = np.array(response.image_data_float, dtype=np.float)
        # img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
        # img2d = np.reshape(img1d, (response.height, response.width))
        # image = Image.fromarray(img2d)
        # im_final = np.array(image.resize((120, 120)).convert("L"))
        #         # Sometimes no image returns from api
        # try:
        #     return im_final.reshape([120, 120, 1])
        # except:
        #     return np.zeros((self.image_shape))
        
        img1d = np.array(response.image_data_float, dtype=np.float64)
        try:
            img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
            # print("MODIFIED")
            # print(img1d)
            # print(img1d.shape)
        except ZeroDivisionError:
            print("ZeroDivisionError")
            print("WINTER CONTINGENCY PLAN")
            img1d = np.ones(img1d.size)
        if img1d.size == 14400:
            img2d = np.reshape(img1d, (120,120))
        else:
            img2d = np.zeros([120,120])
            print("ERROR____WINTER CONTRINGENCY")

        # print("img2d.shape")

        # print(img2d.shape)
        image = Image.fromarray(img2d)
        # print("IMAGE")
        # print(image)
        im_final = np.array(image.convert("L"))
                # Sometimes no image returns from api
        # print(im_final.shape)
        # print(im_final)
        try:
            return im_final.reshape([120, 120, 1])
        except:
            return np.zeros((self.image_shape[0], self.image_shape[1]))
    
    
    def _get_obs(self):
        responses = self.drone.simGetImages([self.image_request])
        image = self.transform_obs(responses[0])
# # /        self.drone_state1 = self.drone.getMultirotorState()
#         image2 = self.transform_obs(responses[0])
#         # self.drone_state2 = self.drone.getMultirotorState()
#         image3 = self.transform_obs(responses[0])
#         # self.drone_state3 = self.drone.getMultirotorState()
#         image = image1 * 0.5 + image2 * 0.3 + image3 * 0.2
        self.drone_state = self.drone.getMultirotorState()
        image = image.astype(np.uint8)
        self.state["prev_pose"] = self.state["pose"]
        self.state["pose"] = self.drone_state.kinematics_estimated
        self.state["collision"] = self.drone.simGetCollisionInfo().has_collided
        self.info["collision"] = self.is_collision()
        # print(image.dtype)
        
        return image, self.info

    def get_depth_image(self, thresh = 2.0):
        depth_image_request = airsim.ImageRequest(1, airsim.ImageType.DepthPerspective, True, False)
        responses = self.drone.simGetImages([depth_image_request])
        depth_image = np.array(responses[0].image_data_float, dtype = np.float32)
        depth_image = np.reshape(depth_image, (responses[0].height, responses[0].width))
        depth_image[depth_image > thresh] = thresh
        return depth_image
    
    def _compute_reward(self, obs):
        reward = 0
        done = 0
        reference_speed = 5
        goal_point = np.array([110, 10, -1])
        self.state["position"] = self.drone_state.kinematics_estimated.position
        self.state["pose"] = self.drone_state.kinematics_estimated.linear_velocity.y_val

        distanceToGoal = math.sqrt(math.pow((goal_point[0] - self.state["position"].x_val),2))
        traveled_distance = math.sqrt(self.state["position"].x_val ** 2)
        average_linear_velocity = self.state["pose"]
        
        if self.state["collision"]:
            done = 1
            reward_distance = round((traveled_distance / goal_point[0])**0.5,5)
            action_accelerator =  round((average_linear_velocity / reference_speed)**2,5)
            collision_penalty = -20
            go_to_center = round(1 - (self.state["position"].y_val / goal_point[1])**2, 5)
            # print(reward_distance * 10)
            # print(action_accelerator * 1.5)
            # print(collision_penalty)
            # print(go_to_center)
            reward = round(reward_distance * 10 + action_accelerator * 5.0 + go_to_center * 2.5 + collision_penalty, 5)
            # print("Collision Occurred, reward : ", reward)
            print("traveled_distance: ",traveled_distance)
        if self.state["position"].x_val > 105:
            done = 1
            reward_distance = round((traveled_distance / goal_point[0])**0.5,5)
            action_accelerator =  round((average_linear_velocity / reference_speed)**2,5)
            goal_incentive = 50
            go_to_center = round(1 - (self.state["position"].y_val / goal_point[1])**2, 5)

            reward = round(reward_distance * 10 + action_accelerator * 5.0 + go_to_center * 2.5 + goal_incentive, 5)
            print("Reached Goal, TOTAL Reward :",reward)
            print("traveled_distance: ",traveled_distance)
            
        return reward, done

    def is_collision(self):
        current_collision_time = self.drone.simGetCollisionInfo().time_stamp
        return True if current_collision_time != self.collision_time else False
    

class TestEnv(AirSimDroneContinuousImageOnlyEnv):
    
    def __init__(self, ip_address, image_shape, env_config):
        self.eps_n = 0
        super(TestEnv, self).__init__(ip_address, image_shape, env_config)
        self.agent_traveled = []
        
    def setup_flight(self):
        super(TestEnv, self)._setup_drone()
        self.eps_n += 1
        
    def _compute_reward(self, obs):
        reward = 0
        done = 0
                
        return reward, done
        