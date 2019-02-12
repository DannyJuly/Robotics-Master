#!/usr/bin/env python

import numpy
import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF

"""This function will transform a 4x4 transformation matrix T into a ros message 
which can be published. In addition to the transform itself, the message
also specifies who is related by this transform, the parent and the child.
It is an optional function which you may use as you see fit."""
def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]        
    return t
    
#Our main class for computing Forward Kinematics
class ForwardKinematics(object):

    #Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)


    """This function is called every time the robot publishes its joint values. You must use
    the information you get to compute forward kinematics.

    The callback you write should iterate through the entire robot chain, and publish 
    the transform for each link you find.
    """
    def callback(self, joint_values):
	# YOUR CODE GOES HERE

        robot = URDF.from_parameter_server()
        link = robot.get_root()
        root_name = link
        link_names =[]
        joints_info = []
        all_trans = tf.msg.tfMessage()
        T1 = numpy.eye(4)
        #print (robot.joint_map[joint_values.name[1]])
        while True:
            if link not in robot.child_map:
                break
            (next_joint_name,next_link) = robot.child_map[link][0]
            #print(next_joint_name,next_link)
            #print(len(self.robot.child_map[link]))
            link_names.append(next_link)
            #print(len(link_names))
            
            joints_info.append(robot.joint_map[next_joint_name])
            link = next_link
            print(robot.joint_map)
            #print(joints_info[0].name)
            #print(robot.child_map)
        #print(joints_info[0].origin.rpy[0])
        #print(joint_values.position)
        
       # print(T1)
        
        for i in range(len(joints_info)):
            translation = tf.transformations.translation_matrix(joints_info[i].origin.xyz)
            a = joints_info[i].origin.rpy[0]
            b = joints_info[i].origin.rpy[1]
            c = joints_info[i].origin.rpy[2]
            q_rpy = tf.transformations.quaternion_from_euler(a,b,c)
            rpy = tf.transformations.quaternion_matrix(q_rpy)
            transform = numpy.dot(translation,rpy)
            #print(joints_info[i].origin.rpy)
            try:
                q = joint_values.position[joint_values.name.index(joints_info[i].name)]
            except ValueError:
                q = 0.0
            #print(joints_info[i].type) 
            if joints_info[i].type =='revolute':

                axis = joints_info[i].axis
                #print(axis)
                quaternion = tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(q,axis))
            else:
                quaternion = numpy.eye(4)

            T = numpy.dot(transform,quaternion)
            T1 = numpy.dot(T1,T) 
            all_trans.transforms.append(convert_to_message(T1,link_names[i],root_name))


       
        #print(link_names)
        # Publish all the transforms
        self.pub_tf.publish(all_trans)







       
       
if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()

