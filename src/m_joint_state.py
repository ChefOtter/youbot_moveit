#!/usr/bin/python

import tf
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import numpy as np
from sensor_msgs.msg import JointState 
from std_msgs.msg import Header

class New_joint_state( object ):
    def __init__(self):
        # create all required variables:
        self.running_flag = False
        self.ref_time = None
        self.ref_state = None
        self.ref_func = None
        self.tbase = rospy.Time.now()
        self.X = None

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomcb, queue_size=10)
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=10)                   
        return

        #this function creates the base joint_states so that it can be relayed by arm_joint states meesage so that a 8 DOF joint state message can be created  
    def odomcb(self, data):
        self.X = self.odom_to_state(data)
        # if self.running_flag:
        joint_states= JointState()
        joint_states.header = Header()
        joint_states.header.stamp = rospy.Time.now()
        joint_states.name = ['virtual_x', 'virtual_y', 'virtual_theta']
        joint_states.position = [self.X[0], self.X[1], self.X[2]]
        joint_states.velocity = []
        joint_states.effort = []
        self.pub.publish(joint_states)
        return
        
    def odom_to_state(self, data):
        quat = data.pose.pose.orientation
        q = np.array([quat.x, quat.y, quat.z, quat.w])
        theta = tf.transformations.euler_from_quaternion(q, 'sxyz')[2]
        return np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            theta
            ])

    

def main():
    arm = New_joint_state()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('moveit_joint_state_publisher')
    main()
