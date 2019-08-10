#!/usr/bin/env python

import numpy
import rospy
import time
from hand_analysis.msg import GraspInfo
from sklearn.decomposition import PCA
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.neighbors import KNeighborsRegressor
from sklearn.linear_model import LogisticRegression
from sklearn.svm import SVC
class HandAnalysis(object):

	def __init__(self):

		self.sub = rospy.Subscriber('/grasp_info', GraspInfo, self.callback)
		self.recv_data = numpy.empty((1,23))
		self.pub = rospy.Publisher('/labeled_grasp_info', GraspInfo, queue_size = 100)
		self.recv_glove=[]
		self.recv_emg = []
		self.recv_low = []
		time.sleep(1)

	def training(self):
		file = rospy.get_param('~train_filename')
		data = numpy.genfromtxt(fname=file, delimiter=',', skip_header=1)
		grasp_labels = data[:,0]
		X_glove = data[:,9:24]
		X_emg = data[:,1:9]
		self.lda_glove = LinearDiscriminantAnalysis()
		self.lda_glove.fit(X_glove,grasp_labels)
		self.svc = SVC(gamma=1e-5,C=10)
		self.svc.fit(X_emg,grasp_labels)
		self.knn = KNeighborsRegressor(n_neighbors=7)
		self.knn.fit(X_emg,X_glove)
		self.pca_low=PCA(n_components=2).fit(X_glove)

	def callback(self,msg):

		# glove to labels
		if  len(msg.glove) > 1:
			self.recv_glove.append(msg.glove)
			if len(self.recv_glove)==1410:
				pred_labels = self.lda_glove.predict(self.recv_glove) 
				rate = rospy.Rate(500)            
				for i in range(1410):
					message = GraspInfo()
					message.label = pred_labels[i]
					self.pub.publish(message)
					rate.sleep()
				
		# emg data to labels and glove
		if len(msg.emg) > 1:
			self.recv_emg.append(msg.emg)
			if len(self.recv_emg)==1410:	
				glove_pred = self.knn.predict(self.recv_emg)
				labels_pred = self.svc.predict(self.recv_emg)
				rate = rospy.Rate(500)                       
				for i in range(1410):
					message = GraspInfo()
					message.label = labels_pred[i]
					message.glove = glove_pred[i]
					self.pub.publish(message)
					rate.sleep()				
				
		#low dimensional to glove
		if len(msg.glove_low_dim) > 1:
			self.recv_low.append(msg.glove_low_dim)
			rate = rospy.Rate(200)
			if len(self.recv_low)==3:
				low_to_glove = self.pca_low.inverse_transform(self.recv_low)
				for i in range(3):
					message = GraspInfo()
					message.glove = low_to_glove[i]
					self.pub.publish(message)
					rate.sleep()

if __name__=='__main__':
	rospy.init_node('analysis',anonymous=True)
	ha = HandAnalysis()
	ha.training()
	rospy.spin()
