
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

import gym
from gym import spaces
import queue
from queue import Queue
from airgym.envs.airsim_env import AirSimEnv
from airgym.envs.airsim_env import AirSimContinuousEnv
from PIL import Image


class AirSimDroneContinuousEnv(AirSimContinuousEnv):
    def __init__(self, ip_address, step_length, image_shape, env_config, attitude_shape):
        super().__init__(image_shape, attitude_shape)
        self.sections = env_config["sections"]
        number_of_batch = 3
        reference_speed = 5
        self.drone = airsim.MultirotorClient(ip = ip_address)
        # Continuous Action Space
        self.action_space = spaces.Box(low=-reference_speed, high=reference_speed, shape=(1,),dtype=np.float32)
        self.image_request = airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False)
        self.drone_state = self.drone.getMultirotorState()
        self.step_length = step_length
        self.image_shape = image_shape
        self.attitude_shape = attitude_shape
        self.start_ts = 0
        self.image_queue = Queue(number_of_batch)
        self.attitude_queue = Queue(number_of_batch)
        
        print("------------ AIRSIM CONTINUOUS ENVIRONMENTS ------------")
        print("------------ OBSERVATION SPACE : IMAGE 3D / ATTITUTE DATA ------------")
        print("------------ CREATED BY CHARLES LIM ------------")
        print("------------ KOREA ADVANCED INSTITUTE OF SCIENCE AND TECHNOLOGY ------------")
        print("------------ AEROSPACE SYSTEMS AND CONTROL LABORATORY ------------")

        print("-----INITIALIZATING-----")
        # angular_velocity = np.array(self.drone_state.kinematics_estimated.angular_velocity.x_val,
        #                             self.drone_state.kinematics_estimated.angular_velocity.y_val,
        #                             self.drone_state.kinematics_estimated.angular_velocity.z_val)
        self.state = {
            "position": np.zeros(3),
            "prev_position": np.zeros(3),
            "pose": None,
            "prev_pose": None,
            "Angular velocity": np.zeros(3),
            "collision": False,
        }
        self.info = {"collision": False}
        
        # self.observation_space = spaces.Dict(
        #     {
        #         # Sequential Image Data
        #         "Image": self.image_request,
        #         # Sequential Attitude Data
        #         "angular_velocity": angular_velocity,  
        #     }
        # )
        self._setup_drone()
    
    
    def _setup_drone(self):
        # self.drone.confirmConnection()
        # print("CONNECTED")
        self.drone.reset()
        self.drone.enableApiControl(True)
        self.drone.armDisarm(True)
        # Get collision time stamp
        self.collision_time = self.drone.simGetCollisionInfo().time_stamp
        # x_pos = 0.0
        # y_pos = (np.random.randint(11) - 5)
        # z_pos = 0.0
        # pose = airsim.Pose(airsim.Vector3r(x_pos, y_pos, z_pos))
        # self.drone.simSetVehiclePose(pose=pose, ignore_collision=True)
        # print("---------------------RESET AGENT---------------------")
        
    def step(self, action):
 
        self._do_action(action)
        obs, info = self._get_obs()
        reward, done = self._compute_reward(obs)
        return obs, reward, done, info

    def reset(self):
        number_of_batch = 3
        self._setup_drone()
        self.image_queue = Queue(number_of_batch)
        self.attitude_queue = Queue(number_of_batch)

        x_pos = 1.0
        y_pos = (np.random.randint(11) - 5)
        # y_pos = 0.0
        z_pos = 0.0
        pose = airsim.Pose(airsim.Vector3r(x_pos, y_pos, z_pos))
        self.drone.simSetVehiclePose(pose=pose, ignore_collision=True)
        initial_action = np.random.randint(11) - 5 
        self.drone.moveByVelocityZAsync(0,0,-2.5,1).join()
        self._do_action(0)
        obs, _ = self._get_obs()
        return obs

    def __del__(self):
        self.drone.reset()

    def _do_action(self, action):
        timestep = 0.1
        # referenceSpeed = 8
        action = round(float(action),3)
        # Action Command
        self.drone.moveByVelocityZAsync(8,action,-2.5,duration = timestep, drivetrain = 1).join()
            # # Prevent swaying
        # self.drone.moveByVelocityAsync(vx=0, vy=0, vz=0, duration=0.001)

        # print("Action Command : ",action)
        
           
    def transform_obs(self, response):
        
        img1d = np.array(response.image_data_float, dtype=np.float)
        
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

    
    
    
    def _image_queue(self, image_to_queue):
        queue = self.image_queue
        if queue.full() == True:
            # print("FULL")
            return queue
        else:
            # print("NOT FULL")
            queue.put(image_to_queue)
            return queue
        
    def _attitude_queue(self, attitude_to_queue):
        queue = self.attitude_queue
        if queue.full() == True:
            # print("FULL")
            return queue
        else:
            # print("NOT FULL")
            queue.put(attitude_to_queue)
            return queue            
        
    
    def _get_obs(self):
        reference_speed = 8
        reference_speed_y = 5
        image_batch = np.zeros([3,self.image_shape[0],self.image_shape[1]])
        attitute_batch = np.zeros([3, 2])
        attitute = np.zeros([2,1])
        responses = self.drone.simGetImages([
        # airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
        airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True)]) #depth in perspective projection
        #scene vision image in uncompressed RGBA array
        # airsim.ImageRequest("0", airsim.ImageType.Scene), #scene vision image in png format
        # airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        
        attitute_responses = self.drone.getMultirotorState()
        attitute = [round(attitute_responses.kinematics_estimated.linear_velocity.x_val,3), round(attitute_responses.kinematics_estimated.linear_velocity.y_val,3)]
        
        attitute_responses_queue = self._attitude_queue(attitute)
        if attitute_responses_queue.full() == True:
            # print("---BATCH ATTITUTE INPUT START")
            attitute = attitute_responses_queue.get()
            attitute_np = np.array(list(attitute))
            attitute_batch[0,:] = attitute_np
            attitute = attitute_responses_queue.get()
            attitute_np = np.array(list(attitute))
            attitute_batch[1,:] = attitute_np
            attitute = attitute_responses_queue.get()
            attitute_np = np.array(list(attitute))
            attitute_batch[2,:] = attitute_np
            
            self._attitude_queue(attitute_batch[1,:])
            self._attitude_queue(attitute_batch[2,:])
    
            # attitute_state = attitute_batch[0,:,:]*0.4 + attitute_batch[1,:,:]*0.3 + attitute_batch[2,:,:]*0.2 + attitute_batch[3,:,:]*0.1
            input_attitute = attitute_batch
            # print(input_attitute)
        # print('Retrieved images: %d' % len(responses))
        # tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
        # print ("Saving images to %s" % tmp_dir)
        else:
            # print("---SINGLE ATTITUTE MODE...REQUIRE MORE ATTITUTE DATA---")
            # print(image_queue.qsize())
            # final_image = image.reshape([100,100,1])
            input_attitute = np.zeros([3,2])
            for i in range(3):
                input_attitute[i,:] = attitute
        
        image = self.transform_obs(responses[0])
        image3d = np.zeros([1,self.image_shape[0], self.image_shape[1]])
        image3d[0,:,:] = image
        image_queue = self._image_queue(image3d)
        if image_queue.full() == True:
            # print("---BATCH INPUT START---")
            for i in range(image_queue.maxsize):
                image = image_queue.get()
                # print(image_queue.qsize())
                image_np = np.array(list(image))
                image_batch[i,:,:] = image_np
            self._image_queue(image_batch[1,:,:])
            self._image_queue(image_batch[2,:,:])
            # self._image_queue(image_batch[3,:,:])
            # final_image = image_batch[0,:,:]*0.2 + image_batch[1,:,:]*0.3 + image_batch[2,:,:]*0.5
            # or 100 X 100 X 3 FRAME
            final_image = image_batch
            # Image_batch[2,:,:] is more close to future
            # final_image = final_image.reshape([100,100,1])
            # print(final_image)
            # filename = os.path.join(tmp_dir, str(idx + float(time())))
            # print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            # airsim.write_file(os.path.normpath(filename + '_converged.png'), response.image_data_uint8)

        else:
            # print("---SINGLE IMAGE MODE...REQUIRE MORE IMAGE---")
            # print(image_queue.qsize())
            # final_image = image.reshape([100,100,1])
            for i in range(3):
                image_batch[i,:,:] = image
                final_image = image_batch

        input_image = final_image            
        # input_image = np.where(final_image > 200, 255, final_image)
        input_image =input_image.reshape(120,120,3)
        input_image = input_image.astype(np.uint8)
        
        self.drone_state = self.drone.getMultirotorState()
        self.state["prev_pose"] = self.state["pose"]
        self.state["pose"] = self.drone_state.kinematics_estimated
        self.state["collision"] = self.drone.simGetCollisionInfo().has_collided
        self.info["collision"] = self.is_collision()
        # print(self.info["collision"])
        if self.info["collision"] == True:
            # print("collsion_occur----eliminate outlier and replace with previous data")
            input_attitute[attitute_responses_queue.maxsize-1,:] = input_attitute[attitute_responses_queue.maxsize-2,:]
            # print(input_attitute)
        # Normalization
        input_attitute[:][1] = input_attitute[:][1] / reference_speed
        input_attitute[:][0] = input_attitute[:][1] / reference_speed_y

        # print(input_attitute)
        input_attitute = input_attitute.astype(np.float64)
        obs = {"Image": input_image, "Linear velocity": input_attitute}
        # print(obs.get('Linear velocity'))
        return obs, self.info
    
    def _compute_reward(self, obs):
        reward = 0
        done = 0
        reference_speed = 5
        goal_point = np.array([110, 10, -1])
        self.state["position"] = self.drone_state.kinematics_estimated.position
        distanceToGoal = math.sqrt(math.pow((goal_point[0] - self.state["position"].x_val),2))
        traveled_distance = math.sqrt(self.state["position"].x_val ** 2 )
        average_linear_velocity = (obs.get('Linear velocity')[0][1] + obs.get('Linear velocity')[1][1] + obs.get('Linear velocity')[2][1]) / 3

        if self.state["collision"]:
            done = 1
            reward_distance = round((traveled_distance / goal_point[0])**0.5, 5)
            action_accelerator =  round((average_linear_velocity / reference_speed)**2,7)
            collision_penalty = -20
            go_to_center = round(1 - (self.state["position"].y_val / goal_point[1])**2,5)
            # print(reward_distance * 10)
            # print(action_accelerator * 100)
            # print(collision_penalty )
            # print(go_to_center * 2.5)
            reward = reward_distance * 10 + action_accelerator * 5 + go_to_center * 2.5 + collision_penalty
            # print("Collision Occurred, reward : ", reward)
            print("Reached Distance : ", traveled_distance)

        if self.state["position"].x_val > 105:
            done = 1
            reward_distance = round((traveled_distance / goal_point[0])**0.5, 5)
            action_accelerator =  round((average_linear_velocity / reference_speed)**2,5)
            goal_incentive = 50
            go_to_center = round(1 - abs(self.state["position"].y_val / goal_point[1])**2,5)

            reward = reward_distance * 10 + action_accelerator * 5 + go_to_center * 2.5 + goal_incentive
            # print(reward_distance * 10)
            # print(action_accelerator * 5)
            # print(goal_incentive )
            # print(go_to_center * 2.5)
            print("Reached Distance : ", traveled_distance)
            print("Reached Goal, TOTAL Reward :",reward)
            
        return reward, done


    def is_collision(self):
        current_collision_time = self.drone.simGetCollisionInfo().time_stamp
        return True if current_collision_time != self.collision_time else False
    

class TestEnv(AirSimDroneContinuousEnv):
    
    def __init__(self, ip_address, image_shape, env_config, attitude_shape):
        self.eps_n = 0
        super(TestEnv, self).__init__(ip_address, image_shape, env_config, attitude_shape)
        self.agent_traveled = []
        
    def setup_flight(self):
        super(TestEnv, self)._setup_drone()
        self.eps_n += 1
        
    def _compute_reward(self, obs):
        reward = 0
        done = 0
                
        return reward, done
        
        
    # def _get_obs(self):
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
    
