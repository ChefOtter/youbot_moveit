#!/usr/bin/env python
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib_msgs.msg import GoalID
import actionlib
import rospy
import numpy as np

import kbhit


class BaseGoalTest( object ):
    def __init__(self):
        self.pub = rospy.Publisher("base_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=1)
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)
        self.kb = kbhit.KBHit()
        rospy.on_shutdown(self.kb.set_normal_term)
        return

    def keycb(self, tdat):
        # check if there was a key pressed, and if so, check it's value and
        # toggle flag:
        if self.kb.kbhit():
            c = self.kb.getch()
            if ord(c) == 27:
                rospy.signal_shutdown("Escape pressed!")
            if c == 'p':
                rospy.loginfo("You pressed 'p'.... publishing")
                self.publish()
            self.kb.flush()
        return

        
    def publish(self):
        fjtag = FollowJointTrajectoryActionGoal()
        goal = FollowJointTrajectoryGoal()
        jt = JointTrajectory()
        jt.joint_names = ["virtual_x", "virtual_y", "virtual_th"]

        Tper = 15.0
        dt = 0.1
        radius = 0.5
        tvec = np.arange(0.0, Tper+dt, dt)
        
        for t in tvec:
            print t
            pt = JointTrajectoryPoint()
            pt.positions = [radius*np.cos(t*2*np.pi/Tper)-radius, radius*np.sin(t*2*np.pi/Tper), 0]
            pt.time_from_start = rospy.Duration(t)
            jt.points.append(pt)
        fjtag.goal_id = GoalID(rospy.Time.now(), "test_{0:012.3f}".format(rospy.Time.now().to_sec()))
        
        fjtag.goal = goal
        fjtag.header.stamp = fjtag.goal_id.stamp

        goal.trajectory = jt
        self.pub.publish(fjtag)
        return



def main():
    arm = BaseGoalTest()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('base_reference_test')
    main()
