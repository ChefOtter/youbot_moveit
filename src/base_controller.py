#!/usr/bin/env python
import tf
import actionlib
import rospy
import tf.transformations as tr
import math

from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np
from scipy.interpolate import interp1d
from scipy import interpolate

import angle_utils
import angle_utils

from sensor_msgs.msg import JointState 

# FREQ = 200 # Hz
Kx = 0.1
Ky = 0.1
Kth = 0.0

Kix = 0
Kiy = 0

MAX_X = 0.1
MAX_Y = 0.1
MAX_TH = np.pi/4.0

class BaseController( object ):
    def __init__(self):
        # create all required variables:
        self.running_flag = False
        self.ref_time = None
        self.ref_state = None
        # self.ref_func = None
        self.ref_arr = None
        self.tbase = rospy.Time.now()
        self.X = None
        
        self.first_time = True  
        self.get_intergral_time  = True
        
        self.time_gap = 0
        
        self.Einx = 0
        self.Einy = 0

        # create all publishers and subscribers:
        self.cmd_pub = rospy.Publisher("cmd_vel_raw", Twist, queue_size=1)
        
        #set the transfrom broadcaster to publish the velocity transfrom from spatial frame to body frame
        self.br = tf.TransformBroadcaster()
        self.timer = rospy.Timer(rospy.Duration(0.005), self.timercb)
        #intialize the subscriber to subscribe the data from odom/, this odom/ is used as feedback for driving the base 
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomcb, queue_size=10)
        self.traj_sub = rospy.Subscriber("base_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal,
                         self.trajcb, queue_size=1) 
                         
        
        rospy.set_param('return_to_base', False)
        
                                                   
        return

    def odomcb(self, data):
        self.X = self.odom_to_state(data)
        return


    def timercb(self, event):
        self.run_controller()
        return
        
        #this function is to derive the velocity from the spline 
    def estimate_derivative(self, t, eps=0.001):
        if self.ref_arr is None:
            rospy.logerr("No spline available for derivative!")
            return
        else:
            state = np.zeros(3)
            for i in range(3):
                state[i] = interpolate.splev(t, self.ref_arr[i], der=1, ext=2)
        return state
        
        #this function is used to interpolate the discontinous base trajectory created by Moveit
        #so that the velocity can be obtained by differentiating the spline 
    def ref_func(self, t):
        if self.ref_arr is None:
            rospy.logerr("No spline available!")
            return
        else:
            state = np.zeros(3)
            for i in range(3):
                state[i] = interpolate.splev(t, self.ref_arr[i], der=0, ext=2)
        return state

        #this function clamps the moving velocity of youBot 
    def clamp_controls(self, twist):
        twist.linear.x = np.clip(twist.linear.x, -MAX_X, MAX_X)
        twist.linear.y = np.clip(twist.linear.y, -MAX_Y, MAX_Y)
        twist.angular.z = np.clip(twist.angular.z, -MAX_TH, MAX_TH)
        return twist
        
        #this function prevents the robot from dead band.
    def low_limit(self, twist):
        if  math.fabs(twist.linear.x)<0.01:
            twist.linear.x = 0
        if  math.fabs(twist.linear.y) <0.01:
            twist.linear.y = 0
        if  math.fabs(twist.angular.z)<0.01:
            twist.angular.z = 0
        return twist
    
        #controller function which output the velocities for youBot drive
    def run_controller(self):
        if self.X is None:
            return
        self.t = (rospy.Time.now() - self.tbase).to_sec()
        if self.running_flag and not rospy.get_param('return_to_base'):
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
            # uff is in the spatial frame... transform to body frame:
            Rbo = tr.rotation_matrix(self.ref_func(self.t)[2], [0,0,1])[0:2,0:2].T
            Vb = np.dot(Rbo, uff[0:2])
            Vfeedback_p = np.array([
                50*Kx*(Xref[0] - self.X[0]),
                50*Ky*(Xref[1] - self.X[1])
            ])
            
            Vfb_k = np.dot(Rbo, Vfeedback_p)
            
            if self.get_intergral_time== 0: 
                self.time_gap = rospy.Time.now().to_sec() 
                self.get_intergral_time = 1
            elif self.get_intergral_time == 1:
                self.time_gap = rospy.Time.now().to_sec() - self.time_gap 
                self.Einx = self.Einx + (Xref[0] - self.X[0])*self.time_gap            
                self.Einy = self.Einy + (Xref[1] - self.X[1])*self.time_gap
                self.get_intergral_time = 2
            else: 
                self.Einx = self.Einx + (Xref[0] - self.X[0])*self.time_gap            
                self.Einy = self.Einy + (Xref[1] - self.X[1])*self.time_gap
                
            Vfeedback_i = np.array([
                self.Einx*Kix,
                self.Einy*Kiy
            ])
            
            Vfb_i = np.dot(Rbo, Vfeedback_i)
            
            cmd.linear.x = Vb[0] + Vfb_k[0]+Vfb_i[0]
            cmd.linear.y = Vb[1] + Vfb_k[1]+Vfb_i[1]
            cmd.angular.z = uff[2] + Kth*angle_utils.shortest_angular_distance(self.X[2],Xref[2])
            cmd = self.clamp_controls(cmd)
            rospy.logdebug("t = %f, xd = %f, yd = %f, thd = %f", self.t, cmd.linear.x, cmd.linear.y, cmd.angular.z)
            self.send_transform(Xref)
            self.cmd_pub.publish(cmd)
        else:
            try:
                if not rospy.get_param('return_to_base'):
                    Xref = np.array([self.ref_state[-1][0],self.ref_state[-1][1],self.ref_state[-1][2]]) 
                else:
                    self.running_flag = False
                    self.ref_state[-1][0] = 0 
                    self.ref_state[-1][1] = 0 
                    self.ref_state[-1][2] = 0
                    Xref = np.array([0,0,0]) 
            except IndexError:
                return
            except TypeError:
                return
            cmd = Twist()
            Vfeedback = np.array([
                3*Kx*(Xref[0] - self.X[0]),
                3*Ky*(Xref[1] - self.X[1])
            ])
            Rbo = tr.rotation_matrix(Xref[2], [0,0,1])[0:2,0:2].T
            Vfb = np.dot(Rbo, Vfeedback)
            cmd.linear.x = Vfb[0]
            cmd.linear.y = Vfb[1]

            cmd.angular.z = angle_utils.shortest_angular_distance(self.X[2],Xref[2])
            cmd = self.clamp_controls(cmd)
            cmd = self.low_limit(cmd);
            rospy.logdebug("t = %f, xd = %f, yd = %f, thd = %f", self.t, cmd.linear.x, cmd.linear.y, cmd.angular.z)
            if not self.first_time:
                self.cmd_pub.publish(cmd)
            if rospy.get_param('return_to_base'):
                self.cmd_pub.publish(cmd) 
        return

    def send_transform(self, Xref):
        self.br.sendTransform(
            [Xref[0], Xref[1], 0],
            tr.quaternion_from_euler(Xref[2], 0, 0, 'szyx'),
            rospy.Time.now(), "ref_frame", "odom")
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
        #this callback function returns the base trajectory and interpolate it 
    def trajcb(self, data):
        # we just received a trajectory goal, let's fill out interpolation function:
        self.ref_time = []
        self.ref_state = []
        for pt in data.goal.trajectory.points:
            self.ref_time.append(pt.time_from_start.to_sec())
            self.ref_state.append(list(pt.positions))
        self.ref_time = np.array(self.ref_time)
        self.ref_state = np.array(self.ref_state)
        self.ref_arr = np.array([
            interpolate.splrep(self.ref_time, x, s=0) for x in self.ref_state.T])
        # self.ref_func = interp1d(self.ref_time, self.ref_state, axis=0)
        # now we can setup control variables:
        self.tbase = rospy.Time.now()
        self.running_flag = True
        return
    
def main():
    arm = BaseController()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('base_controller')
    main()
