#!/usr/bin/env python3

from logging import error
import threading
import time
import rospy
import math
import numpy as np
import tf
import std_msgs.msg
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
# import roslib; roslib.load_manifest('std_srvs')
from std_srvs.srv import Empty
import mav_msgs.msg as mav_msgs


def hovering_example():
    rospy.init_node('joverineksampol', anonymous=True)
    trajectory_pub = rospy.Publisher('command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    #trajectory_pub = rospy.Publisher('mav_msgs.default_topics/COMMAND_TRAJECTORY', MultiDOFJointTrajectory, queue_size=10)
    
    service_name = '/gazebo/unpause_physics'

    rospy.loginfo('Starting hovering example.')

    unpaused = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    rospy.wait_for_service(service_name)
    #r = rospy.Rate(20)
    i = 0
    
    # revisar unpaused y revisar el servicio revisar imprimiendo en consola
    
    while i <= 10 and not unpaused:
        rospy.loginfo('Wait for 1 second before trying to unpause Gazebo again.')
        time.sleep(1)
        #r.sleep()
        unpaused = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        i = i+1

    if not unpaused:
        rospy.loginfo('Could not wake up Gazebo.')
        raise error
    else:
        rospy.loginfo('Unpaused the gazebo simulation')

    rospy.loginfo('hola')
    
    
    #rospy.sleep(5.)
    
    rospy.loginfo('adiós')

    trajectory_msg = MultiDOFJointTrajectory()
    trajectory_msg.header.stamp = rospy.Time.now()
    trajectory_msg.header.frame_id = 'frame'
    trajectory_msg.joint_names.append('base_link')


#    desired_position = Vector3()
#   desired_yaw = 0.0

#    desired_position.x = 0.0
#    desired_position.y = 0.0
#    desired_position.z = 1.0
# create end point for trajectory

    x = 0.0
    y = 0.0
    z = 1.0
    
    #yaw = 0.0

    transforms = Transform()
    transforms.translation.x = x
    transforms.translation.y = y
    transforms.translation.z = z 

    #quat = tf.transformations.quaternion_from_euler(yaw*np.pi/180.0, 0, 0, axes = 'rzyx')
    #transforms.rotation.x = quat[0]
    #transforms.rotation.y = quat[1]
    #transforms.rotation.z = quat[2]
    #transforms.rotation.w = quat[3]

    #Point
    velocities = Twist()
    accel = Twist()
    point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accel], rospy.Time(1))
    trajectory_msg.points.append(point)

    rospy.loginfo('1')
    #rospy.sleep(1)
    rospy.loginfo('2')
    trajectory_pub.publish(trajectory_msg)
    rospy.loginfo('3')

    rospy.spin()

if __name__ == '__main__':
    hovering_example()
    
