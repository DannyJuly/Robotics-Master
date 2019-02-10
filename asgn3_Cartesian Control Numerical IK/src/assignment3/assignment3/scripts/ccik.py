#!/usr/bin/env python

import math
import numpy
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from cartesian_control.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import tf
from threading import Thread, Lock
import time

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(object):
    def __init__(self):
	#Load robot from parameter server
        self.robot = URDF.from_parameter_server()

	#Subscribe to current joint state of the robot
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

	#This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

	#This is a mutex; a mutex(mutual exclusion object)is a program object that is 
	#created so that multiple program thread can take turns sharing the same resource
        self.mutex = Lock()

	#Subscribers and publishers for for cartesian control
        rospy.Subscriber('/cartesian_command', CartesianCommand, self.get_cartesian_command)
        self.velocity_pub = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        rospy.Subscriber('/ik_command', Transform, self.get_ik_command)
        self.joint_command_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
        
        joint_transforms, b_T_ee = self.forward_kinematics(self.q_current)# from b to end-factor T
        jaco=self.get_jacobian(b_T_ee, joint_transforms)

        x_trans=tf.transformations.translation_matrix(numpy.array([command.x_target.translation.x,\
        	command.x_target.translation.y,command.x_target.translation.z]))
        x_rot=tf.transformations.quaternion_matrix(numpy.array([command.x_target.rotation.x,
        	command.x_target.rotation.y,command.x_target.rotation.z,command.x_target.rotation.w]))
       	ee_desired=numpy.dot(x_trans,x_rot)
       	
       	current_to_desired = numpy.dot(numpy.linalg.inv(b_T_ee),ee_desired)
       	

       	angle,axis=self.rotation_from_matrix(current_to_desired)
       	det_angle= numpy.dot(angle,axis)
       	det_trans=current_to_desired[:3,3]
    	trans_gain = 1
    	rota_gain =1
       	det_x=numpy.append(trans_gain*det_trans,rota_gain*det_angle)
       	#det_x=np.transpose(np.array(ee_trans_desired-b_T_ee[:3,3]))
        gain=1
        vee=gain*det_x
        #Scale the desired end-effector velocity
        vee_rot_max=numpy.max(abs(vee[numpy.arange(3,6)]))
        vee_tran_max=numpy.max(abs(vee[numpy.arange(0,3)]))
        
        if vee_rot_max  >1:
        	k=1/numpy.max(abs(vee_rot_max))   
        	vee[3:]=k*vee[3:]
        	
        if vee_tran_max > 0.1:
        	k=0.1/numpy.max(abs(vee_tran_max))
        	vee[:3]=k*vee[:3]

        jaco_p=numpy.linalg.pinv(jaco)
        jaco_ps=numpy.linalg.pinv(jaco,0.01)
        dq=numpy.dot(jaco_ps,vee)

        #secondary objective
        sec_gain=3
        q_sec=[sec_gain*(command.q0_target-self.q_current[0])]
        for i in range(self.num_joints-1):
        	q_sec=numpy.append(q_sec,0)
        dq_null=numpy.dot((numpy.eye(self.num_joints)-numpy.dot(jaco_p,jaco)),q_sec)
        dq = dq+dq_null

        #Scale the resulting desired q
        if numpy.max(abs(dq)) > 1:
        	k=1/numpy.max(abs(dq))
        	dq=k*dq
        else:
        	dq=dq
       


        self.joint_velocity_msg.name=self.joint_names
        #print(self.joint_names)
        self.joint_velocity_msg.velocity=dq
        self.velocity_pub.publish(self.joint_velocity_msg)
        #print(dq)

 
        #--------------------------------------------------------------------------
        self.mutex.release()
        

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def S(self,x,y,z):
        S=([0,-z,y],[z,0,-x],[-y,x,0])
        return S
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE


        for i in range(self.num_joints):
        	b_to_j = joint_transforms[i] #transform from b to j
        	j_to_b = tf.transformations.inverse_matrix(b_to_j)
        	#self.b_to_ee = b_T_ee
        	j_to_ee = numpy.dot(j_to_b,b_T_ee)
        	ee_to_j = tf.transformations.inverse_matrix(j_to_ee)
        	ee_rota_j = ee_to_j[:3,:3] #rotation part of transformation from ee to j
        	j_trans_ee = tf.transformations.translation_from_matrix(j_to_ee)
        	s = j_trans_ee
        	s_j_to_ee = numpy.array([[0,-s[2],s[1]],[s[2],0,-s[0]],[-s[1],s[0],0]])
        	vj = numpy.append(numpy.append(ee_rota_j,numpy.dot(-ee_rota_j,s_j_to_ee),axis=1),
        		numpy.append(numpy.zeros((3,3)),ee_rota_j,axis=1),axis=0)

        	#another way to calculate
        	#if numpy.size(numpy.where(self.joint_axes[i]==1))==1:
        	#	J[:i]=vj[:,3+numpy.where(self.joint_axes[i]==1)[0][0]]
        	#	print(numpy.where(self.joint_axes[i]==1)[0][0])
        	#else :
        		#J[:i]=-vj[:,3+numpy.where(self.joint_axes[i]==-1)[0][0]]
        	column=int(numpy.sum(numpy.array(self.joint_axes[i])*numpy.array([1,2,3])))

        	if column>0:
        		J[:,i]=vj[:,2+column]
        	elif column < 0:
        		J[:,i]=-vj[:,2-column]
        return J
        
    
        #--------------------------------------------------------------------------


    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE
        x_trans=tf.transformations.translation_matrix(numpy.array([command.translation.x,\
        	command.translation.y,command.translation.z]))
        x_rot=tf.transformations.quaternion_matrix(numpy.array([command.rotation.x,
        	command.rotation.y,command.rotation.z,command.rotation.w]))
       	ee_desired=numpy.dot(x_trans,x_rot)
     
       	# try three times
       	for i in range(3):
       		q_c=numpy.random.rand(self.num_joints)*2*numpy.pi
       		t_end=time.time()+10
	       	while time.time()<t_end:
	       		
	       	    joint_transforms, b_T_ee = self.forward_kinematics(q_c)
	       	    current_to_desired = numpy.dot(numpy.linalg.inv(b_T_ee),ee_desired)
	       #	print(desired_to_current)
	            angle,axis=self.rotation_from_matrix(current_to_desired)
	       	    det_angle= angle*axis
	       	    det_trans=current_to_desired[:3,3]
	       	    det_x=numpy.append(det_trans,det_angle)

	       	    x_c=tf.transformations.translation_from_matrix(b_T_ee)
	       	    x=x_c[0]
	       	    y=x_c[1]
	       	    z=x_c[2]
	       	    jaco=self.get_jacobian(b_T_ee, joint_transforms)
	       	    jaco_pinv=numpy.linalg.pinv(jaco)
	       	    diff=numpy.max(abs(det_x))

	       	    if diff < 0.001:
	       	    	self.joint_command_msg.name=self.joint_names
	       	    	self.joint_command_msg.position=q_c
	       	    	self.joint_command_pub.publish(self.joint_command_msg)

	       	    	break
	       	    else:
	       	    	det_q=numpy.dot(jaco_pinv,det_x)
	       	    
	       	    	q_c=q_c+det_q
	       	if diff < 0.001:
	       		break

       	    #jaco=self.get_jacobian(b_T_ee, joint_transforms)
       	    #jaco_pinv=numpy.linalg.pinv(jaco)

       	    #det_q=numpy.dot(jaco_pinv,det_x)
       	    
       	    #q_c=q_c+det_q
       	#self.joint_command_msg.name=self.joint_names
       	#self.joint_command_msg.position=q_c
       	




        #--------------------------------------------------------------------------
        self.mutex.release()
        #self.joint_command_pub.publish(self.joint_command_msg)

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
        R = numpy.array(matrix, dtype=numpy.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = numpy.linalg.eig(R33.T)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = numpy.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = numpy.linalg.eig(R)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (numpy.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each joint of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.transformations.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()
