import rospy
import actionlib
import actionlib_tutorials.msg

from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from actionlib_msgs.msg import GoalID

class Arm:
    def __init__(self, arm_name):
        #arm_name should be l_arm or r_arm
        self.name = arm_name
        self.jta = actionlib.SimpleActionClient('Moveit/arm_controller/follow_joint_trajectory',
                                                FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        
        
    def send_goal(self):
        action_goal = FollowJointTrajectoryAction()
        fjtag = FollowJointTrajectoryActionGoal()
        goal = FollowJointTrajectoryGoal()
        jt = JointTrajectory()
        jt.joint_names = ["virtual_x", "virtual_y", "virtual_th","arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"]

        Tper = 15.0
        dt = 0.1
        radius = 0.5
        tvec = np.arange(0.0, Tper+dt, dt)
        
        for t in tvec:
            pt = JointTrajectoryPoint()
            pt.positions = [radius*np.cos(t*2*np.pi/Tper)-radius, 0, 0,0,0,0,0,radius*np.sin(t*2*np.pi/Tper)]
            pt.time_from_start = rospy.Duration(t)
            jt.points.append(pt)
        fjtag.goal_id = GoalID(rospy.Time.now(), "test_{0:012.3f}".format(rospy.Time.now().to_sec()))
        
        fjtag.goal = goal
        fjtag.header.stamp = fjtag.goal_id.stamp

        goal.trajectory = jt
        action_goal =goal
        self.jta.send_goal_and_wait(action_goal)
        
        return
        
#    def move(self, angles):
#        goal = JointTrajectoryGoal()
#        char = self.name[0] #either 'r' or 'l'
#        goal.trajectory.joint_names = [char+'virtual_x'
#                                       char+'virtual_y'
#                                       char+'virtual_z'
#                                       char+'arm_joint_1'
#                                       char+'arm_joint_2'
#                                       char+'arm_joint_3'
#                                       char+'arm_joint_4',
#                                       char+'arm_joint_5']                
#        point = JointTrajectoryPoint()
#        point.positions = angles
#        point.time_from_start = rospy.Duration(3)
#        goal.trajectory.points.append(point)
#        self.jta.send_goal_and_wait(goal)

def main():
    arm = Arm('arm_1/arm')
    arm.send_goal()


if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()
    rospy.spin()
