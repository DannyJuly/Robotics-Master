#!/usr/bin/env python
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg
import math

def angle_bet(v1,v2):
    v1_u = v1/numpy.linalg.norm(v1)
    v2_u = v2/numpy.linalg.norm(v2)
    return (numpy.arccos(numpy.clip(numpy.dot(v1_u,v2_u),-1.0,1.0)))
def matrix_by_vector_multi(matrix,vector):
    vector.append(1)
    x = ([sum([vector[x]*matrix[n][x] for x in range(len(vector))]) for n in range(len(matrix))])
    return (x)
def publish_transforms():
    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id ="base_frame"
    t1.child_frame_id = "object_frame"
    q1 = tf.transformations.quaternion_from_euler(0.64, 0.64, 0)
    t1.transform.rotation.x = q1[0]
    t1.transform.rotation.y = q1[1]
    t1.transform.rotation.z = q1[2]
    t1.transform.rotation.w = q1[3]
    T1 = numpy.dot(tf.transformations.quaternion_matrix(q1),tf.transformations.translation_matrix((1.5, 0.8, 0.0)))
    tr1 = tf.transformations.translation_from_matrix(T1)
    t1.transform.translation.x = tr1[0]
    t1.transform.translation.y = tr1[1]
    t1.transform.translation.z = tr1[2]
    br.sendTransform(t1)
    #print (T1)

    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_frame"
    t2.child_frame_id = "robot_frame"
    q2 = tf.transformations.quaternion_about_axis(1.5, (0,1,0))
    t2.transform.rotation.x = q2[0]
    t2.transform.rotation.y = q2[1]
    t2.transform.rotation.z = q2[2]
    t2.transform.rotation.w = q2[3]
    T2 = numpy.dot(tf.transformations.quaternion_matrix(q2),tf.transformations.translation_matrix((0,0,-2)))
    #T2 = tf.transformations.concatenate_matrices(tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(1.5,(0,0,1.0))),tf.transformations.translation_matrix((0,-1,0)))
    tr2 = tf.transformations.translation_from_matrix(T2)
    t2.transform.translation.x = tr2[0]
    t2.transform.translation.y = tr2[1]
    t2.transform.translation.z = tr2[2]
    br.sendTransform(t2)

    t3 = geometry_msgs.msg.TransformStamped()
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "robot_frame"
    t3.child_frame_id = "camera_frame"


    Move = tf.transformations.translation_matrix((0.3,0.0,0.3))
    Rota = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0,0,0))
    T3 = numpy.dot(Move,Rota)
    #print(T2)
    #print(T3)
    object = tf.transformations.translation_from_matrix(T1)
    #print(object)
    object_robot = matrix_by_vector_multi(tf.transformations.inverse_matrix(T2),object.tolist())
    object_robot = object_robot[:(len(object_robot)-1)]
    #print(object_robot)
    object_camera = matrix_by_vector_multi(tf.transformations.inverse_matrix(T3),object_robot)
    object_camera = object_camera[:(len(object_camera)-1)]
    #print(object_camera)
    x_axis = [1,0,0]
    angle = angle_bet(x_axis,object_camera)
    v_normal = numpy.cross(x_axis,object_camera)
    Rota = tf.transformations.quaternion_matrix(
        tf.transformations.quaternion_about_axis(
        angle,v_normal))
    T3 = numpy.dot(Move,Rota)

    tr3 = tf.transformations.translation_from_matrix(T3)
    t3.transform.translation.x = tr3[0]
    t3.transform.translation.y = tr3[1]
    t3.transform.translation.z = tr3[2]
    q3 = tf.transformations.quaternion_from_matrix(T3)
  #  q3 = tf.transformations.quaternion_from_matrix()
    t3.transform.rotation.x = q3[0]
    t3.transform.rotation.y = q3[1]
    t3.transform.rotation.z = q3[2]
    t3.transform.rotation.w = q3[3]
    br.sendTransform(t3)
    #print (T3)







if __name__ == '__main__':
    rospy.init_node('solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.1)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.1)
