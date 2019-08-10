#!/usr/bin/env python
import torch
import torch.nn as nn
import torch.nn.functional as F 
import rospy
import numpy as np 
import math

from torch.utils.data.dataset import Dataset
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt

from robot_sim.msg import RobotState
from robot_sim.srv import RobotAction
from robot_sim.srv import RobotActionRequest
from robot_sim.srv import RobotActionResponse
from sklearn.neighbors import KNeighborsRegressor

class MyDNN(object):
    def __init__(self,num_random_tests,num_steps):
        self.num_random_tests = num_random_tests
        self.num_steps = num_steps
        self.sub = rospy.Subscriber('/robot_states',RobotState, self.callback,queue_size = 100)
        self.state = rospy.Service('fake_robot',RobotAction,self.state_train)
        self.real_robot_action = rospy.ServiceProxy('real_robot', RobotAction)
        self.data_fea = np.empty((self.num_random_tests*self.num_steps+1,9))
        self.label = np.empty((self.num_random_tests*self.num_steps,6))
        self.fake_state = np.array([-1.57,0,0,0,0,0])

    def training_data(self):   #get the data to train fake robot
        n=0
        for k in range(0,self.num_random_tests):
            action = np.random.rand(1,3)
            action[0,0] = (2 * action[0,0]-1.0) * 1.0
            action[0,1] = (2 * action[0,1] - 1.0) * 0.5
            action[0,2] = (2 * action[0,2] - 1.0) * 0.25
            req_real = RobotActionRequest()
            req_real.reset = True
            resp = self.real_robot_action(req_real)
            self.data_fea[k*self.num_steps,0:6] = np.array(resp.robot_state)

            print(k)
            for i in range(self.num_steps):
                req_real = RobotActionRequest()
                req_real.reset = False
                req_real.action = action.reshape((3))
                resp_real = self.real_robot_action(req_real)
                self.data_fea[n,6:9] = req_real.action
                self.label[n] = resp_real.robot_state   
                n += 1
                self.data_fea[n,0:6] = resp_real.robot_state
        print('trainning finished')
        self.data_fea = np.delete(self.data_fea,-1,axis = 0)
        #print(self.data_fea)

    def training(self):
        self.knn = KNeighborsRegressor(n_neighbors=7)
        self.knn.fit(self.data_fea,self.label)
                
    
    def callback(self,msg):
        if msg.robot_name == 'fake_robot':
            #self.a= a
            #print('call',self.a)
            self.fake_state = np.array(msg.robot_state)
           # print('1',self.fake_state)
        #print(a)


    def state_train(self,req):
        resp = RobotActionResponse()
        if req.reset == True:
            resp.robot_state = [-1.57,0.0,0.0,0.0,0.0,0.0]
            self.init_state = resp.robot_state
            self.flag = 1
            return resp
        else:
            if self.flag == 1:
                data_pred = np.hstack((np.array(self.init_state),req.action))          
                resp.robot_state = self.knn.predict(np.array([data_pred]))[0,:].tolist()
                #resp.robot_state = [float(i) for i in resp.robot_state]
                self.flag = 0
            else:
                #print('2',self.fake_state)
                data_pred = np.hstack((self.fake_state,req.action))          
                resp.robot_state = self.knn.predict(np.array([data_pred]))[0,:].tolist()
            return resp






if __name__=='__main__':
    rospy.init_node('fake_robot_DNN',anonymous=True)
    haha = MyDNN(1000,200)
    haha.training_data()
    haha.training()
    rospy.spin()







