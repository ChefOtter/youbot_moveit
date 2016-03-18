#!/usr/bin/env python
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib_msgs.msg import GoalID

import actionlib
import rospy
import numpy as np

import time

from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
import kbhit

numberOfArmJoints =5

class ArmTest( object ):
    def __init__(self):
    
        self.pub_youbotleap_arm = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions,queue_size=10)
        
        rospy.sleep(1)
        return 
        
    def publish(self):

        msg = JointPositions()
        
        arm_list = []
        
        armJointPositions = JointValue()
        
        armJointPositions.timeStamp = rospy.Time.now()
        armJointPositions.joint_uri = "arm_joint_1"
        armJointPositions.value = 0.1
        armJointPositions.unit = "rad"
        arm_list.append(armJointPositions)
        
        armJointPositions = JointValue()
        
        armJointPositions.timeStamp =rospy.Time.now()
        armJointPositions.joint_uri = "arm_joint_2"
        armJointPositions.value = 0.1
        armJointPositions.unit = "rad"
        arm_list.append(armJointPositions)
        
        armJointPositions = JointValue()
        
        armJointPositions.timeStamp = rospy.Time.now()
        armJointPositions.joint_uri = "arm_joint_3"
        armJointPositions.value = -2
        armJointPositions.unit = "rad"
        arm_list.append(armJointPositions)
        
        armJointPositions = JointValue()
        
        armJointPositions.timeStamp =rospy.Time.now()
        armJointPositions.joint_uri = "arm_joint_4"
        armJointPositions.value = 1
        armJointPositions.unit = "rad"
        arm_list.append(armJointPositions)
        
        armJointPositions = JointValue()
        
        armJointPositions.timeStamp = rospy.Time.now()
        armJointPositions.joint_uri = "arm_joint_5"
        armJointPositions.value = 0.111
        armJointPositions.unit = "rad"
        arm_list.append(armJointPositions)
        
        
        
        msg.positions = arm_list
        
        self.pub_youbotleap_arm.publish(msg)
        
        rospy.sleep(1)
#        msg.positions[0]= ["arm_joint_1", "arm_joint_2", "arm_joint_3","arm_joint_4","arm_joint_5"]
#        msg.positions[1] = "rad"
#        msg.positions[2] = [0, 0, 0,0,0]
        Tper = 15.0
        dt = 0.1
        tvec = np.arange(0.0, Tper+dt, dt)
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            for t in tvec:
                arm_list[0].value = 1.2+ 1*np.sin(2*np.pi*rospy.Time.now().to_sec()/Tper)        
            msg.positions = arm_list        
            self.pub_youbotleap_arm.publish(msg)
                
        return


        



def main():
    arm = ArmTest()
    arm.publish()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('arm_reference_test')
    main()
    
