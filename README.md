# arm-planner
Compare different planners and IK solvers using the universial robot. Please install the robot's package first.

Current files:

Note: before running these nodes, launch the demo.launch in ur10_moveit_config package first.

1. move_group_interface_test.cpp - just some test codes
2. objects_test.cpp - just another test codes
3. pick_up.cpp - create a planning scene. The ur robot will try to pick up a ball and place it to another place.
4. compare_ik.cpp - gives a set of poses in workspace and tries to solve the ik problem using ik solvers.To compare between different ik solvers, change the kinematics.yaml in ur10_moveit_config as follows:                 
                 From: ```kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin```
                 to :  ```kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin```
                 
5. launch/ur10.launch - This is a batch test of ik solver using 1000 random poses in workspace. It compares the KDL with trac_ik.              
