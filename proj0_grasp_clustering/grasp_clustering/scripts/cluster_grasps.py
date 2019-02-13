#!/usr/bin/env python

##########################################
##### WRITE YOUR CODE IN THIS FILE #######
##########################################
'''
import numpy as np
import rospy

from grasp_clustering.msg import GraspInfo

class ClusterGrasps(object):

	def init(self):
		self.sub = rospy.Subscriber('/grasp_info',GraspInfo,self.callback)
		self.pub = rospy.Publisher('/labeled_grasp_info',GraspInfo,queue_size = 1) 
		self.glove = []
		self.labels  = []



	def callback(self,msg):
		self.glove=[]
		'''

import numpy
import rospy
import time

from grasp_clustering.msg import GraspInfo
from sklearn.cluster import KMeans

class ClusterGrasps(object):
  

    def __init__(self):
        
        self.sub = rospy.Subscriber("/grasp_info", GraspInfo, self.callback) 

        self.recv_glove_data = numpy.empty((1040,15))
        self.label = []
      
        self.pub = rospy.Publisher("/labeled_grasp_info", GraspInfo, queue_size=1)
        time.sleep(1)
    #def training(self):
        file = rospy.get_param('~train_filename')
        data = numpy.genfromtxt(fname=file, delimiter = ',', skip_header=1)
        X = data[:,8:23]
        self.kmeans = KMeans(n_clusters = 6, random_state = 42).fit(X)
        #rint(self.kmeans)
    
    def callback(self, msg):
        self.label.append(msg.label)
        #print len(self.qqq)
        self.recv_glove_data[len(self.label)-1,:] = msg.glove
        
        if (len(self.label) == 1040):
             self.glove_pre = self.kmeans.predict(self.recv_glove_data)
             #print(self.aaa)
             rate = rospy.Rate(500)
             for i in range(1040):
                 message = GraspInfo()
                 message.label = self.glove_pre[i]
                 #print(message.label)
                 self.pub.publish(message)
                 rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('cluster_grasps', anonymous=True)
    cg = ClusterGrasps()
    #cg.training()
    time.sleep(1)
    rospy.spin()


