#!/usr/bin/env python

# Columbia Engineering
# MECS 4602 - Fall 2018
from __future__ import division
import math
import numpy
import time

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData

class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))
        self.sens_vel = 0

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.01
        self.k = 0

        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
        
    # This function gets called every time the robot publishes its control 
    # input and sensory output. You must make use of what you know about 
    # extended Kalman filters to come up with an estimate of the current
    # state of the robot and covariance matrix.
    # The SensorData message contains fields 'vel_trans' and 'vel_ang' for
    # the commanded translational and rotational velocity respectively. 
    # Furthermore, it contains a list 'readings' of the landmarks the
    # robot can currently observe
    def estimate(self, sens):

        ### ----- YOUR CODE GOES HERE ----- ####
        #print(self.x)
        # F=numpy.zeros((3,3))
        # F[0,0]=1
        # F[0,2]=-self.step_size*sens.vel_trans*math.sin(self.x[2])
        # F[1,1]=1
        # F[1,2]=self.step_size*sens.vel_trans*math.cos(self.x[2])
        # F[2,2]=1
        # #print(F)

        # xr = self.x[0] + self.step_size* sens.vel_trans * math.cos(self.x[2])
        # yr= self.x[1] + self.step_size* sens.vel_trans * math.sin(self.x[2])
        # theta= self.x[2] + self.step_size* sens.vel_ang
        # x_hat=numpy.vstack((xr,yr,theta))

        # P_hat=numpy.dot(F,numpy.dot(self.P,numpy.transpose(F)))+ self.V
        # y_hat=numpy.zeros((2,1))
        # y_hat0=[]
        # H=numpy.zeros((2,3))
        # h=numpy.zeros((2,1))
        # n=0

        # if len(sens.readings)>0:
        #     for i in numpy.arange(len(sens.readings)):
        #         xl=sens.readings[i].landmark.x
        #         yl=sens.readings[i].landmark.y
        #         range1=math.sqrt((xr-xl)**2+(yr-yl)**2)
        #         bearing1=math.atan2(yl-yr,xl-xr)-theta

        #         if range1>0.1:
        #             bearing=sens.readings[i].bearing
        #             range=sens.readings[i].range

        #             y_hat0=numpy.zeros((2,1))
        #             y_hat0[0,0]=range
        #             y_hat0[1,0]=bearing
        #             y_hat=numpy.vstack((y_hat,y_hat0))


        #             h0=numpy.zeros((2,1))
        #             h0[0,0]=range1
        #             h0[1,0]=bearing1
        #             h=numpy.vstack((h,h0))

        #             H0=numpy.zeros((2,3))
        #             H0[0,0]=(xr-xl)/range1
        #             H0[0,1]=(yr-yl)/range1
        #             H0[0,2]=0
        #             H0[1,0]=(yl-yr)/(range1**2)
        #             H0[1,1]=(xr-xl)/(range1**2)
        #             H0[1,2]=-1

        #             H=numpy.vstack((H,H0))
        #             n+=1

        #         else:
        #             n=n
        #     W=numpy.zeros((2*n,2*n))
        #     for i in numpy.arange(0,2*n,2):
        #         W[i,i]=0.1
        #         W[i+1,i+1]=0.05

        #     H=numpy.delete(H,[0,1],0)
        #     h=numpy.delete(h,[0,1],0)
        #     y_hat=numpy.delete(y_hat,[0,1],0)

        #     H_trans=numpy.transpose(H)
        #     nu=y_hat-h

        #     for i in numpy.arange(0,n*2,2):
        #         if nu[i+1,0]>math.pi:
        #             while nu[i+1]>=math.pi:
        #                 nu[i+1,0]=nu[i+1,0]-2*math.pi
        #         if nu[i+1,0]<-math.pi:
        #             while nu[i+1]<=-math.pi:
        #                 nu[i+1,0]=nu[i+1,0]+2*math.pi
        #         else:
        #             nu[i+1,0]=nu[i+1,0]

        #     S = numpy.dot(H,numpy.dot(P_hat,H_trans))+W
        #         #print(S)
        #     S_inv=numpy.linalg.inv(S)
        #     R = numpy.dot(P_hat,numpy.dot(H_trans,S_inv))
        #     #print(H)

        #     self.x = x_hat + numpy.dot(R,nu)
        #     print('2,',nu)
        #     self.P = P_hat - numpy.dot(R,numpy.dot(H,P_hat))
        # else:
        #     self.x = x_hat
        #     self.P = P_hat


###################
        # t = self.step_size
        # #print(sens.vel_trans)
        # #print (self.k)
        # #self.k+=1

        # F = numpy.array([[1,0,-t * sens.vel_trans * math.sin(self.x[2][0])],
        #                  [0,1,t * sens.vel_trans * math.cos(self.x[2][0])],
        #                  [0,0,1]])
        # self.x[0][0] = self.x[0][0] + t * sens.vel_trans * math.cos(self.x[2][0])
        # self.x[1][0] = self.x[1][0] + t * sens.vel_trans * math.sin(self.x[2][0])
        # self.x[2][0] = self.x[2][0] + t * sens.vel_ang
        # x_pred = self.x
        # #self.sens_vel = sens.vel_trans


        # P_pred = numpy.dot(F,numpy.dot(self.P,numpy.transpose(F)))+ self.V
        # H = numpy.array([[0,0,0]])
        # inno = numpy.array([[0]])

        # if numpy.size(sens.readings) > 0 :

        #     #print(sens.readings[0].range)

        #     #print(sens.readings[0].bearing)
        #     for i in numpy.arange(numpy.size(sens.readings)):
        #         x_land = sens.readings[i].landmark.x
        #         y_land = sens.readings[i].landmark.y

        #         range2=math.sqrt((x_land-self.x[0][0])**2+(y_land-self.x[1][0])**2)
        #         bearing=math.atan2(y_land-x_pred[1][0],x_land-x_pred[0][0])-x_pred[2][0]
        #         if math.sqrt(range2)>0.1:
        #             Hx_pred = numpy.array([range2,bearing])
        #             y_sens = numpy.array([sens.readings[i].range,sens.readings[i].bearing])
        #             inno_i = y_sens - Hx_pred
        #             inno_i = inno_i.reshape((2,1))
        #             if inno_i[1][0]>6.28:
        #                 print('inno,bearing',inno_i[1][0])
        #             while True:
        #                 if inno_i[1][0] > numpy.pi:
        #                     inno_i[1][0] = inno_i[1][0]-2*numpy.pi
        #                 elif inno_i[1][0] < -numpy.pi:
        #                     inno_i[1][0] = inno_i[1][0]+2*numpy.pi
        #                 else:
        #                     break
        #             #print(sens.readings[i].bearing)
        #             #print(inno_i)
        #             H_i= numpy.array([[-(x_land-x_pred[0][0])/(range2),-(y_land-x_pred[1][0])/(range2),0],
        #                             [(y_land-x_pred[1][0])/(range2**2),-(x_land-x_pred[0][0])/(range2**2),-1]])
        #             H = numpy.vstack((H,H_i))
        #             inno = numpy.vstack((inno,inno_i))
        #             #print('hi',self.x[0][0])

            

        #     if H.shape[0] > 1:
        #         H = numpy.delete(H,(0),axis=0)
        #         #print('1,',H)
        #         inno = numpy.delete(inno,(0),axis=0)
        #         #print(H)
        #         W = numpy.zeros((H.shape[0],H.shape[0]))
        #         for i in numpy.arange(int(0.5*H.shape[0])):
        #             W[2*i][2*i]=0.1
        #             W[2*i+1][2*i+1]=0.05

        #         #print('w',W)
        #         #print('p',P_pred)
        #         #print('w',W)
        #         S = numpy.dot(H,numpy.dot(P_pred,numpy.transpose(H)))+W
        #         #print(S)
        #         S_inv=numpy.linalg.inv(S)
        #         R = numpy.dot(P_pred,numpy.dot(numpy.transpose(H),S_inv))

        #         #self.x = x_pred + numpy.dot(R,inno)
        #         print('1,',inno)
        #         self.P = P_pred - numpy.dot(R,numpy.dot(H,P_pred))
            # else:
            #     self.x = x_pred
            #     self.P = P_pred


        
#########################
        # while True:
        #     if self.x[2][0] > numpy.pi:
        #         self.x[2][0] = self.x[2][0]-2*numpy.pi
        #     elif self.x[2][0] < -numpy.pi:
        #         self.x[2][0] = self.x[2][0]+2*numpy.pi
        #     else:
        #         break

            #print(sens.readings[0].bearing)
        t = self.step_size
        F = numpy.array([[1,0,-t * sens.vel_trans * math.sin(self.x[2][0])],
                         [0,1,t * sens.vel_trans * math.cos(self.x[2][0])],
                         [0,0,1]])
        self.x[0][0] = self.x[0][0] + t * sens.vel_trans * math.cos(self.x[2][0])
        self.x[1][0] = self.x[1][0] + t * sens.vel_trans * math.sin(self.x[2][0])
        self.x[2][0] = self.x[2][0] + t * sens.vel_ang
        x_pred = self.x
        P_pred = numpy.dot(F,numpy.dot(self.P,numpy.transpose(F)))+ self.V
        H = numpy.array([[0,0,0]])
        inno = numpy.array([[0]])
        W = numpy.zeros((len(sens.readings)*2,len(sens.readings)*2))
        reading=numpy.array([])

        for i in range(len(sens.readings)):
            index=i
            x_land = sens.readings[i].landmark.x
            y_land = sens.readings[i].landmark.y
            range_2=(x_land-self.x[0][0])**2+(y_land-self.x[1][0])**2
            if math.sqrt(range_2)>=0.1:
                reading=numpy.append(reading,index)
            W[2*i][2*i]=0.1
            W[2*i+1][2*i+1]=0.05


        for i in range(numpy.size(reading)):                                       
            y_sens = numpy.array([sens.readings[int(reading[i])].range,sens.readings[int(reading[i])].bearing])
            x_land = sens.readings[int(reading[i])].landmark.x
            y_land = sens.readings[int(reading[i])].landmark.y
            range_2=(x_land-self.x[0][0])**2+(y_land-self.x[1][0])**2
            if math.sqrt(range_2)>0.1:
                Hx_pred = numpy.array([math.sqrt(range_2),math.atan2(y_land-self.x[1][0],x_land-self.x[0][0])-self.x[2][0]])
                inno_i = y_sens - Hx_pred
                inno_i = inno_i.reshape((2,1))
                # if inno_i[1][0]>6.28:
                #     print('inno,bearing',inno_i[1][0])
                while True:
                    if inno_i[1][0] > numpy.pi:
                        inno_i[1][0] = inno_i[1][0]-2*numpy.pi
                    elif inno_i[1][0] < -numpy.pi:
                        inno_i[1][0] = inno_i[1][0]+2*numpy.pi
                    else:
                        break
                #print(sens.readings[i].bearing)
                #print(inno_i)

                H_i= numpy.array([[-(x_land-self.x[0][0])*range_2**(-0.5),-(y_land-self.x[1][0])*range_2**(-0.5),0],
                                [(y_land-self.x[1][0])/range_2,-(x_land-self.x[0][0])/range_2,-1]])
            
                H = numpy.vstack((H,H_i))
                inno = numpy.vstack((inno,inno_i))
                #print('hi',self.x[0][0])           
        if len(reading)>0:
            H = numpy.delete(H,(0),axis=0)
            #print(H)
            inno = numpy.delete(inno,(0),axis=0)
            zero_row =  len(sens.readings)-len(reading)
            H = numpy.vstack((H,numpy.zeros((zero_row*2,3))))
            inno = numpy.vstack((inno,numpy.zeros((zero_row*2,1))))
            S = numpy.dot(H,numpy.dot(P_pred,numpy.transpose(H)))+W
            #print(S)
            S_inv=numpy.linalg.inv(S)
            R = numpy.dot(P_pred,numpy.dot(numpy.transpose(H),S_inv))

            self.x = x_pred + numpy.dot(R,inno)
            self.P = P_pred - numpy.dot(R,numpy.dot(H,P_pred))
        else:
            self.x = x_pred
            self.P = P_pred


        


        #### ----- YOUR CODE GOES HERE ----- ####
    
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0][0]
        est_msg.pose.y = self.x[1][0]
        est_msg.pose.theta = self.x[2][0]
        self.pub_est.publish(est_msg)

if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()
