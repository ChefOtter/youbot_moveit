# youbot_moveit
This is the README for youbot_moveit.

# Synopsis
What this package does is to implement a mobile manipulator for KUKA youBot in Moveit. In this package, Moveit is able to do the plan for all 8 DOF for youBot (this include the plan for the nonholonomic drive of the robot).

# How to use this pacakge
1. You should first git clone this package

2. Git clone the robot_discription package using following links:

# Package Funciton Discription
The idea of how this package works is:
1. Based on the original youbot.urdf, a new youbot.urdf is created where three virtual joints are added. These three virtual joints are x, y and theta (which discribes the DOF of the robot drive). 

2. Initialize Moveit with this new youbot.urdf so that Moviet is able to do the plan for 8 DOF trajectory.

3. Create an action server which receive the plan from Moviet (Moveit plan is stored in /moveit/arm_controller/follow_joint_trajectory) and decomposit it into two pieces of plan. One plan (arm_1/arm_controller/follow_joint_trajectory) is used to feed the Gazebo or real youBot with 5 DOF arm plan, while another 3 DOF plan(base_controller/follow_joint_trajectory/goal) is feeded to a new controller to drive the youBot.

4. As it is mentioned, a new controller for the control of youBot drive is created which can convert the position data in base_controller/follow_joint_trajectory into twist message which youBot or Gazebo knows how to move the drive. Control loop is added to this controller.

# Improvements do be done in the future
1. Needs to put safety features for the action server so that once youBot is about to collide with some obstacle then the excution of plan stops immediately. 

2. The odom of youBot is drifting as youBot moving around. Currently the feedback from controling the base is simply from odom. So a good reference needs to be added to the system in case of the error caused by the drifting of the odom.

