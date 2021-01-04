import gym
from gym import error,spaces,utils
from gym.utils import seeding
import pybullet as p
import pybullet_data
import time
import numpy as np
import random
import os
import math

class AdversarialCars(gym.Env):
    metadata = {'render.modes':['human', 'rgb_array'],
                'video.frames_per_second':50
                }
    
    def __init__(self, direct=1):
        """__init__ This is a constructor function for the gym environment.

        This function initializes the environment with all the necessary variables
        such as observation space, action space and the arena dimensions. It also 
        connects to Pybullet.

        Parameters
        ----------
        direct : int, optional
            [0 to be passed if want to visualize in GUI], by default 1
        """
        if not direct:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)
    
        #The max_dist variables define the size of the arena
        #n defines the number of agents in the world.
        self.max_dist_x = 4
        self.max_dist_y = 4
        self.n = 2

        #This defines the max force that can be applied
        #to the motor joints of our robot.
        self.maxForce = 30 
        
        self.observation_space = np.array([[1.0,1.0],[1.0,1.0]])

        #Action Space
        #For all the agents, the action space is same.
        #The first action is a continous action which determines the speed of the wheels.
        #The second action is a continous action which determines the angle by which the front wheel turns.
        self.action_space = np.array([spaces.Box(low=np.array([0,-0.5]), high=np.array([30,0.5]))]*self.n)
        self.reduce = 1
        self.step_counter=0

    def step(self, actions):
        """step Function to take a single step of the environment.

        This function performs a single step of the environment
        by taking actions for both the agents, returning observations,
        rewards and checking if terminal state has been reached or not.

        Parameters
        ----------
        actions : list of list of ints, required
            This is a 2x2 list, where both the 1x2 lists are actions
            to be taken by the respective agents. The first action in each
            list is the velocity of the rear wheels whereas the second
            action is the angle by which the front wheel is to be rotated.

        Returns
        -------
        observations: a numpy 2x2 array
        rewards: a list of rewards - [agent1, agent2]
        dones: list of bools - to signify terminal state for both agents 

        """
        observations = []
        rewards = []
        dones = []
        self.perform_action(self.car1, actions[0])
        self.perform_action(self.car2, actions[1])
        p.stepSimulation()
        observations = self._get_obs()
        for i in range(2):
            rewards.append(self._get_rewards(i, observations[i]))
        self.step_counter+=1
        self.reduce = self.reduce * 0.9993
        for i in range(2):
            done, cause = self._check_done(observations[i])
            dones.append(done)
            if(done == 1):
                if(cause==1):
                    rewards[1] += 72000
                    rewards[0] -= 72000
                elif(cause==0):
                    rewards[0] += 72000
                    rewards[1] -= 72000       	  
        return observations, rewards, dones, None

    def reset(self):
        """reset function to reset the environment.

        This function reloads the entire environment along with the
        arena plane, walls, gravity and the physical bots. This must be
        called before using because the init function does not load these
        things.

        Returns
        -------
        observations: a numpy 2x2 array

        """
        p.resetSimulation() 
        dir_path = os.path.dirname(os.path.realpath(__file__))
        path_to_rsc = dir_path + '/rsc/'
        p.loadURDF(path_to_rsc+"plane.urdf")

        #self.wall1 = p.loadURDF(path_to_rsc+"wall.urdf", basePosition = [0,self.max_dist_y,1], baseOrientation = p.getQuaternionFromEuler([1.5707,0,0]), useFixedBase = True)
        #self.wall2 = p.loadURDF(path_to_rsc+"wall.urdf", basePosition = [0,-self.max_dist_y,1], baseOrientation = p.getQuaternionFromEuler([1.5707,0,0]), useFixedBase = True)
        #self.wall3 = p.loadURDF(path_to_rsc+"wall.urdf", basePosition = [self.max_dist_x,0,1], baseOrientation = p.getQuaternionFromEuler([1.5707,0,1.5707]), useFixedBase = True)
        #self.wall4 = p.loadURDF(path_to_rsc+"wall.urdf", basePosition = [-self.max_dist_x,0,1], baseOrientation = p.getQuaternionFromEuler([1.5707,0,1.5707]), useFixedBase = True)

        p.setGravity(0, 0, -10)
        self.car1pos = [random.uniform(-self.max_dist_x,0),random.uniform(-self.max_dist_y,0), 0.03]
        self.car2pos = [random.uniform(0,self.max_dist_x),random.uniform(0,self.max_dist_y), 0.03]

        self.car1 = p.loadURDF(path_to_rsc+"/car/AUTO1.urdf", self.car1pos[0], self.car1pos[1], self.car1pos[2])
        self.car2 = p.loadURDF(path_to_rsc+"/car/AUTO2.urdf", self.car2pos[0], self.car2pos[1], self.car2pos[2])
        self.reduce = 1
        self.step_counter=0
        return self._get_obs()

    def render(self, mode='human'):
        pass

    def perform_action(self, car, action):
        """perform_action function to give actions to the robots.

        This function gives actions to the joint motors of the
        robot, which makes the robot move. We have used position control,
        for the angle of the front wheel and velocity control for the speed
        of the rear wheels.

        Parameters
        ----------
        car : int, required
            0 if action is to be passed to agent_0
            1 if action is to be passed to agent_1
        action : list/array of actions for the agent
            Both the actions in the list should be normalized
            between -1 to 1. This is done because most frameworks
            normalize the actions.
        """
        action[0]=action[0]*10+20
        action[1]=action[1]*0.5
        p.setJointMotorControl2(car, 3, p.POSITION_CONTROL, targetPosition =action[1],force = self.maxForce)
        for i in [0,1]:
            p.setJointMotorControl2(car, i, p.VELOCITY_CONTROL, targetVelocity =action[0],force = self.maxForce)
        p.setJointMotorControl2(car, 7, p.VELOCITY_CONTROL, targetVelocity =action[0]*7,force = self.maxForce)
        pos1, ori1 = p.getBasePositionAndOrientation(car)
        lin, ang = p.getBaseVelocity(car)
        if(pos1[0]<-self.max_dist_x):
            p.resetBasePositionAndOrientation(car, [pos1[0]+2*self.max_dist_x,pos1[1],pos1[2]], ori1)
            vel = p.resetBaseVelocity(car, lin)
        if(pos1[0]>self.max_dist_x):
            p.resetBasePositionAndOrientation(car, [pos1[0]-2*self.max_dist_x,pos1[1],pos1[2]], ori1)
            vel = p.resetBaseVelocity(car, lin)
        if(pos1[1]<-self.max_dist_y):
            p.resetBasePositionAndOrientation(car, [pos1[0],pos1[1]+2*self.max_dist_y,pos1[2]], ori1)
            vel = p.resetBaseVelocity(car, lin)
        if(pos1[1]>self.max_dist_y):
            p.resetBasePositionAndOrientation(car, [pos1[0],pos1[1]-2*self.max_dist_y,pos1[2]], ori1)
            vel = p.resetBaseVelocity(car, lin)
        
    
    def _get_obs(self):
        """_get_obs Function to get observations for both the agents.

        This function is used to calculate and return the observations
        for both the agents in the environment.

        Returns
        -------
        observation: a numpy 2x2 array
            The first element of both the 2x1 lists
            is the distance between the agents, and the
            second element is the angle the corresponding agent
            makes with the line connecting the two agents.
        """
        pos1, orn1 = p.getBasePositionAndOrientation(self.car1)
        pos2, orn2 = p.getBasePositionAndOrientation(self.car2)
        theta1 = p.getEulerFromQuaternion(orn1)[2]
        theta2 = p.getEulerFromQuaternion(orn2)[2]
        x1 = pos1[0]
        y1 = pos1[1]
        x2 = pos2[0]
        y2 = pos2[1]
        dis = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        vec_dis1 = np.array([x2-x1, y2-y1])
        vec_dis2 = np.array([x1-x2, y1-y2])
        wall1 = min([abs(self.max_dist_x-x1),abs(-self.max_dist_x-x1),abs(self.max_dist_y-y1),abs(-self.max_dist_y-y1)])
        wall2 = min([abs(self.max_dist_x-x2),abs(-self.max_dist_x-x2),abs(self.max_dist_y-y2),abs(-self.max_dist_y-y2)])
        xp1 = x1 + math.sin(theta1)
        yp1 = y1 - math.cos(theta1)
        vec1 = np.array([xp1 - x1, yp1 - y1])
        vec1_len = np.sqrt((x1-xp1)**2 + (y1-yp1)**2)
        cross1 = np.cross(vec1, vec_dis1)
        dot1 = np.dot(vec1, vec_dis1)
        angle1 = math.asin(cross1/(dis*vec1_len))
        if(dot1<0 and cross1<0):
            angle1 = -(np.pi + angle1)
        if(dot1<0 and cross1>0):
            angle1 = np.pi - angle1
        xp2 = x2 + math.sin(theta2)
        yp2 = y2 - math.cos(theta2)
        vec2 = np.array([xp2 - x2, yp2 - y2])
        vec2_len = np.sqrt((x2-xp2)**2 + (y2-yp2)**2)
        cross2 = np.cross(vec2, vec_dis2)
        dot2 = np.dot(vec2, vec_dis2)
        angle2 = math.asin(cross2/(dis*vec2_len))
        if(dot2<0 and cross2<0):
            angle2 = -(np.pi + angle2)
        if(dot2<0 and cross2>0):
            angle2 = np.pi - angle2 
        return np.array([[dis,angle1],[dis,angle2]])

    def _get_rewards(self,carNum, obs):
        """_get_rewards function to return rewards for each agent.

        Parameters
        ----------
        carNum : int, required
            0 for agent_0
            1 for agent_1
        obs : A numpy 2x1 array
            observation as received from get_observation

        Returns
        -------
        reward: int
            The reward for the particular agent, given the observation.
        """
        if(carNum == 0):
            return -100*math.exp(-obs[0])
        if(carNum == 1):
            return 100*math.exp(-obs[0])
        

    def _check_done(self, obs):
        """_check_done function to check whether terminal state has been reached.

        This function checks 2 things:
        1. If the environment has been running for more than 14400 steps
        2. If the distance between the two agents is less than a threshold

        Parameters
        ----------
        obs :   A numpy 2x1 array 

        Returns
        -------
        Two integers: done and cause
        """
        if(self.step_counter==14400):
            return 1, 0
        elif(obs[0]<=0.2):
            return 1, 1
        else:
            return 0, None
