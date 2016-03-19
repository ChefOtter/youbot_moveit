# youbot_moveit
This is the README for youbot_moveit.

# Synopsis
What this package does is to implement a mobile manipulator for KUKA youBot in Moveit. In this package, Moveit is able to do the plan for all 8 DOF for youBot (this include the plan for the nonholonomic drive of the robot).

# How to use this package
1. You should first git clone this package

2. Then git clone "robot_description" package using following links

# Package Function Description
The idea of how this package works is:

1. Based on the original youbot.urdf, a new youbot.urdf is created where three virtual joints are added. These three virtual joints are x, y and theta (which describes the DOF for the robot drive). 

2. Initialize Moveit with this new youbot.urdf so that Moviet is able to do the plan for 8 DOF trajectory.

3. Create an action server which receive the 8 DOF plan from Moviet (Moveit plan is stored in /moveit/arm_controller/follow_joint_trajectory). This plan is then split into two separate plan. One plan (arm_1/arm_controller/follow_joint_trajectory) is used to feed the Gazebo or real youBot to control 5 DOF arm movement, while the other 3 DOF plan(base_controller/follow_joint_trajectory/goal) is fed to a new controller to drive youBot.

4. As it is mentioned, a new controller for the control of youBot drive is created which can convert the position data from base_controller/follow_joint_trajectory into twist message that youBot or Gazebo knows how to move the drive. What this controller (now I use second_controller.py)
specifically does is:

-This controller firstly receives the 3 DOF trajectory from the action server, and this trajectory is described as a set of discontinuous position states in series. 

-Then this controller uses cubic interpolation to create a spline for the 3 DOF trajectory and calculates the velocity by deriving the spline. This velocity is wrapped into twist message which youBot knows how to move its base by rolling its wheel. 

-Since the twist message describes the velocity in the body frame so it is necessary to design a transform which can convert the spatial velocity obtained by deriving the spline, into the body frame velocity.

-A proportional control is added to the controller improve the transient error when youBot is executing the plan. This control takes the /odom as feedback.

5. Since the original /joint_states topic published from either Gazebo or real youBot does not contain the extra 3 DOF joint states for those virtual states because the URDF file they use only have 5 DOF. So to create a 8 DOF /joint_states topic, a node is created (by m_joint_state.py) to create the 3 DOF base joint states called /m/joint_states from /odom (this requires some calculations). Then in the launch file, another node is created to relay the original /joint_states to m/joint_states to add 5 DOF to it. This help create a whole tf tree from /odom to virtual joints and then to the end effector on the arm. 

  
# Improvements do be done in the future
1. Needs to put safety features for the action server so that once youBot is about to collide with some obstacle then the execution of plan stops immediately. 

2. The odom of youBot is drifting as youBot moving around. Currently the feedback from controlling the base is simply from odom. So a good reference needs to be added to the system in case of the error caused by the drifting of the odom.


