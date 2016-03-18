#!/usr/bin/env python
import tf
import actionlib
import rospy

import math

from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np
from scipy.interpolate import interp1d

import angle_utils

from sensor_msgs.msg import JointState 

# FREQ = 200 # Hz
Kx = 1
Ky = 1
Kth = 0.000

Kix = 0.01
Kiy = 0.01



MAX_X = 0.3
MAX_Y = 0.3
MAX_TH = np.pi/4.0

class BaseControllerTest( object ):
    def __init__(self):
    
        self.test = True
        
        # create all required variables:
        self.running_flag = False
        self.ref_time = None
        self.ref_state = None
        self.ref_func = None
        self.tbase = rospy.Time.now()
        self.X = None


        # create all publishers and subscribers:
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomcb, queue_size=10)
        self.traj_sub = rospy.Subscriber("base_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal,
                         self.trajcb, queue_size=1)
        self.time_gap = 0
        
        self.first_time = True
        self.get_intergral_time  = True
        
        self.Einx = 0
        self.Einy = 0
                           
        return

    def odomcb(self, data):
        self.X = self.odom_to_state(data)
        # if self.running_flag:
        self.run_controller()
        return


    def estimate_derivative(self, t, eps=0.001):
        fp = self.ref_func(t+eps)
        fn = self.ref_func(t)
        # fm = self.ref_func(t-eps)
        # return (fp - 2*fn + fm)/(eps**2.0)
        return (fp - fn)/eps


    def clamp_controls(self, twist):
        twist.linear.x = np.clip(twist.linear.x, -MAX_X, MAX_X)
        twist.linear.y = np.clip(twist.linear.y, -MAX_Y, MAX_Y)
        twist.angular.z = np.clip(twist.angular.z, -MAX_TH, MAX_TH)
        return twist
        
    def low_limit(self, twist):
        
        if  math.fabs(twist.linear.x)<0.01:
            twist.linear.x = 0
        if  math.fabs(twist.linear.y) <0.01:
            twist.linear.y = 0
        if  math.fabs(twist.angular.z)<0.01:
            twist.angular.z = 0
        return twist
    

    def run_controller(self):
        self.t = (rospy.Time.now() - self.tbase).to_sec()
        # rospy.loginfo("t = %f",t)
        if self.running_flag:
            try:
                Xref = self.ref_func(self.t)
            except ValueError:
                # let's assume this implies t is out of range:
                if self.t < self.ref_time[0] or self.t > self.ref_time[-1]:
                    rospy.loginfo("Trajectory Complete!")
                    self.running_flag = False
                    
                    self.Einx = 0
                    self.Einy = 0

                    
                    if self.first_time: 
                        self.first_time = False
                self.cmd_pub.publish(Twist())
                return
            # if we get here, we can run the controller:
            cmd = Twist()
            uff = self.estimate_derivative(self.t)

#            if self.get_intergral_time== 0: 
#                self.time_gap = rospy.Time.now().to_sec() 
#                self.get_intergral_time = 1
#            elif self.get_intergral_time == 1:
#                self.time_gap = rospy.Time.now().to_sec() - self.time_gap 
#                self.Einx = self.Einx + (Xref[0] - self.X[0])*self.time_gap            
#                self.Einy = self.Einy + (Xref[1] - self.X[1])*self.time_gap
#                self.get_intergral_time = 2
#            else: 
#                self.Einx = self.Einx + (Xref[0] - self.X[0])*self.time_gap            
#                self.Einy = self.Einy + (Xref[1] - self.X[1])*self.time_gap
                   
            cmd.linear.x = uff[0] +10*Kx*(Xref[0] - self.X[0])+self.Einx*Kix
            cmd.linear.y = uff[1] +10*Ky*(Xref[1] - self.X[1])+self.Einy*Kiy
            
            cmd.angular.z = uff[2] + Kth*angle_utils.shortest_angular_distance(Xref[2],self.X[2])
            cmd = self.clamp_controls(cmd)
            cmd = self.low_limit(cmd);
#            rospy.logdebug("t = %f, xd = %f, yd = %f, thd = %f", self.t, cmd.linear.x, cmd.linear.y, cmd.angular.z)
            self.cmd_pub.publish(cmd)
        else:

#            rospy.sleep(5)
#            Xref = np.array([0,0,0])
#            if self.test:
#                rospy.sleep(5)
#                self.test = False
            try:

#                Xref = np.array([0,0,0])
#                if self.first_time: 
#                  
#                    Xref = np.array([0,0,0])
#                else:
                Xref = np.array([self.ref_state[-1][0],self.ref_state[-1][1],self.ref_state[-1][2]])  
         
            except IndexError:
                return
            except TypeError:
                return
                       

            
            cmd = Twist()
            cmd.linear.x = 1*(Xref[0] - self.X[0])
            cmd.linear.y = 1*(Xref[1] - self.X[1])
            cmd.angular.z = angle_utils.shortest_angular_distance(self.X[2],Xref[2])
            cmd = self.clamp_controls(cmd)
            cmd = self.low_limit(cmd);
            print " odom" , self.X
            print "ref " ,Xref
            print "x",1*Kx*(Xref[0] - self.X[0])   
            print "y",1*Ky*(Xref[1] - self.X[1])     
            
            rospy.logdebug("t = %f, xd = %f, yd = %f, thd = %f", self.t, cmd.linear.x, cmd.linear.y, cmd.angular.z)
            self.cmd_pub.publish(cmd)

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

    def trajcb(self, data):
        # we just received a trajectory goal, let's fill out interpolation function:
        self.ref_time = []
        self.ref_state = []
        for pt in data.goal.trajectory.points:
           
            self.ref_time.append(pt.time_from_start.to_sec())
            self.ref_state.append(list(pt.positions))
#        
        
        self.ref_time = np.array(self.ref_time)
        self.ref_state = np.array(self.ref_state)
        self.ref_func = interp1d(self.ref_time, self.ref_state, axis=0)
        # now we can setup control variables:
        self.tbase = rospy.Time.now()
        self.running_flag = True
        
        print self.ref_state[-1]
        
        return
    
def main():
    arm = BaseControllerTest()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('base_controller_test')
    main()
