import setup_path
import airsim
import numpy as np
import math
import time
from argparse import ArgumentParser

import gym
import PIL
from PIL import Image
from gym import spaces
from airgym.envs.airsim_env import AirSimEnv


class AirSimDroneEnv(AirSimEnv):
    def __init__(self, ip_address, step_length, image_shape):
        super().__init__(image_shape)
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
        
        self.drone = airsim.MultirotorClient(ip = ip_address)
        self.action_space = spaces.Discrete(2)
#       self._setup_drone()
#       self.drone.moveToPositionAsync(100,0,5,1).join()

        self.image_request = airsim.ImageRequest("0", airsim.ImageType.DepthVis, True, False)
        
#       self.drone_controls = self.drone.moveByVelocityZAsync(1,0,5,1).join()
        self.drone_state = None
    
    def _setup_drone(self):
        self.drone.confirmConnection()
        print("CONNECTED")
        self.drone.reset()
        self.drone.enableApiControl(True)
        self.drone.armDisarm(True)
        # self.drone.takeoffAsync().join()
        # self.drone.hoverAsync().join()
        # print("Hovering")
        time.sleep(0.1)
        # print("time spent")
 #       self.drone.moveByVelocityZAsync(1,0,5,1).join()

#        self.drone.moveToPositionAsync(100,0,5,1).join()
#        self.drone.moveByVelocityZAsync(1,0,5,5).join()

  #      time.sleep(0.01)
    def step(self, action):
        self._do_action(action)
        obs, info = self._get_obs()
        reward, done = self._compute_reward()
        
        return obs, reward, done, info

    def __del__(self):
        self.drone.reset()

    def _do_action(self, action):
        timestep = 0.25
        if action == 0:
            # print("0")
            # self.drone.armDisarm(True)

            # self.drone.takeoffAsync().join()
            # self.drone.moveToPositionAsync(2000,0,-1,timestep).join()
            self.drone.moveByVelocityZAsync(2,2,-1,timestep).join()
            print("RIGHT")
        elif action == 1:
            # print("1")
            # self.drone.armDisarm(True)

            # self.drone.takeoffAsync().join()
            # self.drone.moveToPositionAsync(2000,0,-1,timestep).join()
            self.drone.moveByVelocityZAsync(2,-2,-1,timestep).join()
            print("LEFT")
        else:
            print(0)

#            self.drone.moveByVelocityZAsync(2,0,-1,timestep).join()
        # elif action == 2:
        #     # print("2")

        #     # self.drone.takeoffAsync().join()

        #     # self.drone.moveToPositionAsync(2000,0,-1,timestep).join()
        #     self.drone.moveByVelocityZAsync(0,5,-1,timestep).join()
        # elif action == 3:
        # #    print("3")
        # #     self.drone.takeoffAsync().join()
            
        #     # self.drone.moveToPositionAsync(2000,0,-1,timestep).join()
        #     self.drone.moveByVelocityZAsync(0,-5,-1,timestep).join()
        # else:            
        #     # self.drone.moveToPositionAsync(2000,0,-1,timestep).join()
        #     self.drone.moveByVelocityZAsync(0,0,-1,timestep).join()
        
        
           
    def transform_obs(self, response):
        img1d = np.array(response.image_data_float, dtype=np.float32)
        
        # print(img1d.shape)
        # print("image1d")
        # # print(img1d)
        # print(img1d.size)
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
            return im_final
        except:
            return np.zeros((self.image_shape[0], self.image_shape[1]))
    
    # 아래 코드는 Sequential Image에 가중치를 주어 더해, 1X84X84형태로 구축한다.
#     def _get_obs(self):
#         imageStack = 3
#         responses = self.drone.simGetImages([self.image_request])
#         image = self.transform_obs(responses[0]) * 0.5
# #        time.sleep(0.01)
#         responses = self.drone.simGetImages([self.image_request])
#         image = image + self.transform_obs(responses[0]) * 0.3
# #        time.sleep(0.01)
#         responses = self.drone.simGetImages([self.image_request])
#         image = image + self.transform_obs(responses[0]) * 0.2
#         image.astype(int)

#         # print("stack image : ",image)
#         # print("stack type: ",type(image))
#         # print("stack shape: ",image.shape)

#         self.drone_state = self.drone.getMultirotorState()

#         self.state["prev_pose"] = self.state["pose"]
#         self.state["pose"] = self.drone_state.kinematics_estimated
#         self.state["collision"] = self.drone.simGetCollisionInfo().has_collided
        
#         return image

    # 아래 코드는 Sequential image를 3X84X84 형태로 구축한다.
    # def _get_obs(self):
    #     imageStack = 3
    #     responses = self.drone.simGetImages([self.image_request])
    #     stackImage = self.transform_obs(responses[0])
    #     print(stackImage)
    #     for i in range(imageStack-1):
    #         responses = self.drone.simGetImages([self.image_request])
    #         image = self.transform_obs(responses[0])
    #         stackImage = np.concatenate((stackImage, image), axis=0)
    #     print("stack image : ",stackImage)
    #     print("stack type: ",type(stackImage))
    #     print("stack shape: ",stackImage.shape)

    #     self.drone_state = self.drone.getMultirotorState()

    #     self.state["prev_pose"] = self.state["pose"]
    #     self.state["pose"] = self.drone_state.kinematics_estimated
    #     self.state["collision"] = self.drone.simGetCollisionInfo().has_collided
        
    #     return image

    def _get_obs(self):
        responses = self.drone.simGetImages([self.image_request])
        image = self.transform_obs(responses[0])
        # print("image : ",image)
        # print("type: ",type(image))
        # print("shape: ",image.shape)
        self.drone_state = self.drone.getMultirotorState()

        self.state["prev_pose"] = self.state["pose"]
        self.state["pose"] = self.drone_state.kinematics_estimated
        self.state["collision"] = self.drone.simGetCollisionInfo().has_collided
        return image, self.state["collision"]

    def _compute_reward(self):
        thresh_dist = 7
        beta = 1
        MAX_SPEED = 5
        MIN_SPEED = -5
        goal_point = [50, 10, -1]
        self.state["position"] = self.drone_state.kinematics_estimated.position
        distanceToGoal = math.sqrt(math.pow((goal_point[0] - self.state["position"].x_val),2)+ math.pow((goal_point[1] - self.state["position"].y_val),2))
        reward = 0
        done = 0
        if reward < -1:
            done = 1
        if self.state["collision"]:
            done = 1
            reward_collision = -5
            if self.state["position"].x_val < 10:
                reward_dist = 1
            elif self.state["position"].x_val < 20:
                reward_dist = 2
            elif self.state["position"].x_val < 30:
                reward_dist = 3
            elif self.state["position"].x_val < 40:
                reward_dist = 4
            elif self.state["position"].x_val < 50:
                reward_dist = 5
            else:
                reward_dist = 0 
            reward = reward_dist + reward_collision
            print("Collision Occurred, reward : ", reward)
        if self.state["position"].x_val > 50:
            done = 1
            reward_dist = 12
            reward = reward_dist
            print("Reached Goal, Reward :",reward)
            
        return reward, done
        
    def reset(self):
        self._setup_drone()
        self._do_action(1)
        obs,_ = self._get_obs()
        return obs
        
# class AirSimDroneEnv(AirSimEnv):
#     def __init__(self, ip_address, step_length, image_shape):
#         super().__init__(image_shape)
#         self.step_length = step_length
#         self.image_shape = image_shape

#         self.state = {
#             "position": np.zeros(3),
#             "collision": False,
#             "prev_position": np.zeros(3),
#         }

#         self.drone = airsim.MultirotorClient(ip=ip_address)
#      #   self.action_space = spaces.Box(low=-1, high=1, shape=(2,)) # Left Right
#         self.action_space = spaces.Discrete(2)
#         # Discrete (Left, Right)   
#         self._setup_flight()
#         self.image_request = airsim.ImageRequest(
#             3, airsim.ImageType.DepthPerspective, True, False
#         )

#     def __del__(self):
#         self.drone.reset()

#     def _setup_flight(self):
#         #initialize
#         self.drone.reset()
#         self.drone.enableApiControl(True)
#         self.drone.armDisarm(True)

#         # Waypoint set up
#         self.drone.moveToPositionAsync(20, 2.5, 2, 1).join()
#         # x축 2km, y축 50미터, z축 10미터, 컨맨드 10초간
#         self.drone.moveByVelocityAsync(0.25, 0.1, 0, 5).join()

    
#     def interpret_action(self, action):
#         if action == 0:
#             cmd = (0.5, 0, 0)
#         elif action == 1:
#             cmd = (-0.5, 0, 0)
        
#         return cmd

#     def _do_action(self, action):
#         cmd = self.interpret_action(action)
#         velocity = self.drone.getMultirotorState().kinematics_estimated.linear_velocity
#         self.drone.moveByVelocityAsync(
#             velocity.x_val + cmd[0],
#             velocity.y_val + cmd[1],
#             velocity.z_val + cmd[2],
#             10,
#         ).join()


#     def reset(self):
#         self._setup_flight()
#         return self._get_obs()
        

#     def step(self, action):
#         self._do_action(action)
#         obs = self._get_obs()
#         reward, done = self._compute_reward()

#         return obs, reward, done, self.state

#     def _get_obs(self):
#         #관측값 정의
#         image = self.drone.simGetImages([self.image_request])
#         self.drone_state = self.drone.getMultirotorState()

#         # 포지션 정의
#         self.state["prev_position"] = self.state["position"]
#         self.state["position"] = self.drone_state.kinematics_estimated.position
#         self.state["velocity"] = self.drone_state.kinematics_estimated.linear_velocity

#         #충돌 정의
#         collision = self.drone.simGetCollisionInfo().has_collided
#         self.state["collision"] = collision

#         return image, self.state
    
#     def _compute_reward(self):
#         thresh_dist = 7 # 뭐지?
#         beta = 1
#         z = 0
#         done = 0
#         if self.state["collision"]:
#             reward = -100
#             done = 1
#             self.drone.reset()
#         else:
#             reward = 0
#             done = 0

#         if reward <= 50:
#             done = 1
        
#         return reward, done

#     def step(self, action):
#         self._do_action(action)
#         obs = self._get_obs()
#         reward, done =self._compute_reward()

#         return obs, reward, done, self.state
# class AirSimDroneEnv(AirSimEnv):
#     def __init__(self, ip_address, step_length, image_shape):
#         super().__init__(image_shape)
#         self.step_length = step_length
#         self.image_shape = image_shape

#         self.state = {
#             "position": np.zeros(3),
#             "collision": False,
#             "prev_position": np.zeros(3),
#         }

#         self.drone = airsim.MultirotorClient(ip=ip_address)
#         self.action_space = spaces.Discrete(7)
#         self._setup_flight()

#         self.image_request = airsim.ImageRequest(
#             3, airsim.ImageType.DepthPerspective, True, False
#         )

#     def __del__(self):
#         self.drone.reset()

#     def _setup_flight(self):
#         self.drone.reset()
#         self.drone.enableApiControl(True)
#         self.drone.armDisarm(True)
#         print("INITIALIZING")

#         # Set home position and velocity
#         self.drone.moveToPositionAsync(1000, 0, 10, 10).join()
#         self.drone.moveByVelocityZAsync(0, 0, 0, 10).join()

#     def transform_obs(self, responses):
#         img1d = np.array(responses[0].image_data_float, dtype=np.float)
#         img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
#         img2d = np.reshape(img1d, (responses[0].height, responses[0].width))

#         from PIL import Image

#         image = Image.fromarray(img2d)
#         im_final = np.array(image.resize((84, 84)).convert("L"))

#         return im_final.reshape([84, 84, 1])

#     def _get_obs(self):
#         responses = self.drone.simGetImages([self.image_request])
#         image = self.transform_obs(responses)
#         self.drone_state = self.drone.getMultirotorState()

#         self.state["prev_position"] = self.state["position"]
#         self.state["position"] = self.drone_state.kinematics_estimated.position
#         self.state["velocity"] = self.drone_state.kinematics_estimated.linear_velocity

#         collision = self.drone.simGetCollisionInfo().has_collided
# #        self.state["collision"] = collision
#         print(collision)
#         return image

#     def _do_action(self, action):
#         quad_offset = self.interpret_action(action)
#         quad_vel = self.drone.getMultirotorState().kinematics_estimated.linear_velocity
#         self.drone.moveByVelocityAsync(
#             quad_vel.x_val + quad_offset[0],
#             quad_vel.y_val + quad_offset[1],
#             quad_vel.z_val + quad_offset[2],
#             5,
#         ).join()

#     def _compute_reward(self):
#         thresh_dist = 7
#         beta = 1

#         z = -10
#         pts = [
#             np.array([-0.55265, -31.9786, -19.0225]),
#             np.array([48.59735, -63.3286, -60.07256]),
#             np.array([193.5974, -55.0786, -46.32256]),
#             np.array([369.2474, 35.32137, -62.5725]),
#             np.array([541.3474, 143.6714, -32.07256]),
#         ]

#         quad_pt = np.array(
#             list(
#                 (
#                     self.state["position"].x_val,
#                     self.state["position"].y_val,
#                     self.state["position"].z_val,
#                 )
#             )
#         )

#         if self.drone.collision_info.has_collided is True:
#             reward = -100
#             done = 1
#             print("COLLISON, DONE =1")
#             self.drone.reset()

        # else:
        #     dist = 10000000
        #     for i in range(0, len(pts) - 1):
        #         dist = min(
        #             dist,
        #             np.linalg.norm(np.cross((quad_pt - pts[i]), (quad_pt - pts[i + 1])))
        #             / np.linalg.norm(pts[i] - pts[i + 1]),
        #         )

        #     if dist > thresh_dist:
        #         reward = -10
        #     else:
        #         reward_dist = math.exp(-beta * dist) - 0.5
        #         reward_speed = (
        #             np.linalg.norm(
        #                 [
        #                     self.state["velocity"].x_val,
        #                     self.state["velocity"].y_val,
        #                     self.state["velocity"].z_val,
        #                 ]
        #             )
        #             - 0.5
        #         )
        #         reward = reward_dist + reward_speed

        
        # if reward <= -10:
        #     done = 1
        #     print("Reward, DONE = 1")


        # return reward, done

    # def step(self, action):
    #     self._do_action(action)
    #     obs = self._get_obs()
    #     reward, done = self._compute_reward()
    #     print("one step---")
    #     return obs, reward, done, self.state

    # def reset(self):
    #     self._setup_flight()
    #     return self._get_obs()

    # def interpret_action(self, action):
    #     if action == 0:
    #         quad_offset = (self.step_length, 0, 0)
    #     elif action == 1:
    #         quad_offset = (0, self.step_length, 0)
    #     elif action == 2:
    #         quad_offset = (0, 0, self.step_length)
    #     elif action == 3:
    #         quad_offset = (-self.step_length, 0, 0)
    #     elif action == 4:
    #         quad_offset = (0, -self.step_length, 0)
    #     elif action == 5:
    #         quad_offset = (0, 0, -self.step_length)
    #     else:
    #         quad_offset = (0, 0, 0)

    #     return quad_offset
