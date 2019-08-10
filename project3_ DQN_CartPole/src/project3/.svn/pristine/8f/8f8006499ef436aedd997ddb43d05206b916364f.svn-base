#!/usr/bin/env python

from __future__ import division
import numpy as np
import math
import time
import random
import rospy
from collections import namedtuple
from itertools import count
import torch
import torch.nn as nn
import torch.nn.functional as F 

from robot_sim.msg import RobotState
from robot_sim.srv import RobotAction
from robot_sim.srv import RobotActionRequest
from robot_sim.srv import RobotActionResponse
from robot_sim.srv import RobotPolicy
from robot_sim.srv import RobotPolicyRequest
from robot_sim.srv import RobotPolicyResponse

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
Transition = namedtuple('Transition',('state','action','reward','next_state'))
eps = 0.9
Batch_Size = 128
Gamma = 0.9

class ReplayMemory(object):
	def __init__(self,capacity):
		self.capacity = capacity
		self.Memory = []
		self.position = 0

	def push(self,args):  #accept all the parameters and store them into a tuple

		if len(self.Memory) < self.capacity:
			self.Memory.append(args)
		else:
			del self.Memory[0]
			self.Memory.append(args)
			#self.position = (self.position + 1) % self.capacity

	def sample(self,batch_size):
		return random.sample(self.Memory, batch_size)


class MyDQN(nn.Module):
	def __init__(self):
		super(MyDQN, self).__init__()
		self.fc1 = nn.Linear(4,16)
		self.fc1.weight.data.normal_(0,0.1)
		self.fc2 = nn.Linear(16,16)
		self.fc2.weight.data.normal_(0,0.1)
		self.fc3 = nn.Linear(16,2)  #two output q_right and q_left
		self.fc3.weight.data.normal_(0,0.1)

	def forward(self,x):
		x = F.relu(self.fc1(x))
		x = F.relu(self.fc2(x))
		x = self.fc3(x)
		return x


class LearnDQN(object):
	def __init__(self):
		self.action_net = MyDQN()
		self.target_net = MyDQN()
		self.num_random_trains = 288
		self.num_steps = 300
		self.q_max = 6*np.pi/180
		self.x_max = 1.2
		self.cartpole_action_service = rospy.ServiceProxy('cartpole_robot', RobotAction, persistent=True)
		#self.cartpole_policy_service = rospy.Service('cartpole_policy', RobotPolicy,  self.robot_policy)
		self.episodes_step = 5
		self.optimizer = torch.optim.RMSprop(self.action_net.parameters(),lr = 0.001)
		self.memory = ReplayMemory(10000)

		

	def selection(self,state):
		sample = random.random()
		#print(sample>eps)
		if sample > eps:
			#with torch.no_grad():
			action_input = torch.unsqueeze(torch.tensor(state),0)  
			action_q = self.action_net.forward(action_input)[0].tolist()
			if action_q[0]>action_q[1]:
				return [-10.0]
			else:
				return [10]
		else:
			return random.sample([10.0,-10.0],1)


	def optimize_model(self):

		if len(self.memory.Memory) >Batch_Size:
			batch=self.memory.sample(Batch_Size)
			batch = Transition(*zip(*batch))
			action_list=list(batch.action)
			#print batch.action

			for i in range(Batch_Size):

				if action_list[i] == [10.0]:
					action_list[i] = [1]
				else:
					action_list[i] = [0]

			state_batch = torch.tensor(batch.state)
			action_batch = torch.LongTensor(action_list)
			reward_batch = torch.FloatTensor(batch.reward)
			state_next_batch=torch.tensor(batch.next_state)

			action_q=self.action_net(state_batch).gather(1,action_batch)

			target_q = self.target_net(state_next_batch).detach()
			Q_value_max=list((target_q.max(1)[0].view(Batch_Size, 1) * Gamma) + reward_batch)
			for j in range(Batch_Size):
				if batch.next_state[j] == [0,0,0,0]:
					Q_value_max[j]=reward_batch[j]

			Q_value_max=torch.Tensor(Q_value_max)

			loss = F.smooth_l1_loss(action_q, Q_value_max.unsqueeze(1))
			self.optimizer.zero_grad()
			loss.backward()
			for param in self.action_net.parameters():
				#print param.data[1]
				param.grad.data.clamp_(-1, 1)
			self.optimizer.step()

		
	def get_random_sign(self):
		return 1.0 if random.random() < 0.5 else -1.0

	def train(self):

		for i in range(0,self.num_random_trains):
			req = RobotActionRequest()
			req.reset_robot = True
			episode_reward = 0
			req.reset_pole_angle = np.random.uniform(0.0, np.deg2rad(3.0))*self.get_random_sign()
			response = self.cartpole_action_service(req)
			state = list(response.robot_state)
			global eps
			eps = max(eps * 0.9, 0.2)#max(1- episode_num/self.num_random_trains,0.01)

			for k in range(self.num_steps):      #right
				req = RobotActionRequest()
				req.reset_robot = False
				action = self.selection(state)
				req.action = action

				response = self.cartpole_action_service(req)
				state_next = list(response.robot_state)

				if abs(state_next[0]) > 1.2 or abs(state_next[1]) > (6*np.pi/180):
					reward = [0]
					state_next = [0,0,0,0]
					self.memory.push([state,action,reward,state_next])
					break
				else:
					episode_reward += 1
					reward = [1]
					self.memory.push([state,action,reward,state_next])

				self.optimize_model()
				state = state_next

			print(episode_reward)
			if i % self.episodes_step ==0:
				self.target_net.load_state_dict(self.action_net.state_dict())



		return self.target_net

class Predict_Mode(object):
	def __init__(self,myDQN):
		self.myDQN = myDQN
		self.cartpole_policy_service = rospy.Service('cartpole_policy', RobotPolicy,  self.robot_policy)
		self.data = []

	def robot_policy(self,msg):
		response = RobotPolicyResponse()
		self.data = torch.tensor(msg.robot_state)
		#print(self.data)
		self.prediction = self.myDQN(self.data).tolist()

		if self.prediction[0] > self.prediction[1]:
			response.action = [-10.0]
		else:
			response.action = [10.0]
		return response



if __name__ == '__main__':
	torch.manual_seed(1)
	np.random.seed(1)
	random.seed(1)
	rospy.init_node('learn_dqn', anonymous=True)
	my_network = LearnDQN()
	DQN = my_network.train()
	Predict_Mode(DQN)
	rospy.spin()

