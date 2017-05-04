#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

bool pick(moveit::planning_interface::MoveGroupInterface* mg,
          double tolerance, double time_out, int attempts,
          geometry_msgs::Pose target_pose, 
          moveit_msgs::AttachedCollisionObject target,
          moveit::planning_interface::MoveGroupInterface::Plan* my_plan)
{
  //set tolerance
  mg->setGoalPositionTolerance(tolerance);
  mg->setGoalOrientationTolerance(tolerance);

  //set target
  mg->setPoseTarget(target_pose);
  
  //set planning time
  mg->setPlanningTime(time_out);

  ROS_INFO("**********************************************");
  ROS_INFO("Planning...");
  int iter = 0;
  while (!mg->plan(*my_plan))
  {
    iter++;
    if (iter > attempts)
    {
      ROS_INFO("Failed");
      return false;
    }
  }
  return true;
}//This function can work properly... Abort.

int main(int argc, char** argv)
{
  //ros init
  ros::init(argc, argv, "planner_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Duration sleep_one_second(1.0);//publishing needs some time.
  ros::Duration sleep_short_time(0.5);

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_one_second.sleep();
  }
  //Construct environment(a shelf witn one ball on it)  using moveit_msgs.
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;

  moveit_msgs::CollisionObject shelf_board[5];
  
  shape_msgs::SolidPrimitive level;
  level.type = level.BOX;
  level.dimensions.resize(3);
  level.dimensions[0] = 1.5;
  level.dimensions[1] = 0.5;
  level.dimensions[2] = 0.02;

  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 1.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  
  for (int i = 0; i < 5; i++)
  {
    pose.position.z = i*0.3;

    shelf_board[i].id = "board" + std::to_string(i);
    shelf_board[i].header.frame_id = "base_link";
    shelf_board[i].primitives.push_back(level);
    shelf_board[i].primitive_poses.push_back(pose);
    shelf_board[i].operation = shelf_board[i].ADD;
    planning_scene.world.collision_objects.push_back(shelf_board[i]);
  }

  shape_msgs::SolidPrimitive side;
  side.type = level.BOX;
  side.dimensions.resize(3);
  side.dimensions[0] = 0.02;
  side.dimensions[1] = 0.5;
  side.dimensions[2] = 1.22;

  moveit_msgs::CollisionObject side_board[2];

  pose.position.x = 0.76;
  pose.position.y = 1.0;
  pose.position.z = 0.6;

  for (int i = 0; i < 2; i++)
  {
    pose.position.x -= i*1.52;
    side_board[i].id = "side" + std::to_string(i);
    side_board[i].header.frame_id = "base_link";
    side_board[i].primitives.push_back(side);
    side_board[i].primitive_poses.push_back(pose);
    side_board[i].operation = side_board[i].ADD;
    planning_scene.world.collision_objects.push_back(side_board[i]);
  }
  //target: ball
  shape_msgs::SolidPrimitive ball;
  ball.type = ball.SPHERE;
  ball.dimensions.resize(3);
  ball.dimensions[0] = 0.1;

  pose.position.x = 0.5;
  pose.position.y = 1.15;
  pose.position.z = 0.12;

  moveit_msgs::AttachedCollisionObject target;
  target.link_name = "ee_link";
  target.object.header.frame_id = "base_link";
  target.object.id = "target";
  target.object.primitives.push_back(ball);
  target.object.primitive_poses.push_back(pose);
  target.object.operation = side_board[0].ADD;

  planning_scene.world.collision_objects.push_back(target.object);

  planning_scene_diff_publisher.publish(planning_scene);
  
  //Get move group interface
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup("manipulator");


  //target pose
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.5;
  target_pose.position.y = 1.0;
  target_pose.position.z = 0.12;
  target_pose.orientation.w = 0.707;
  target_pose.orientation.y = 0;
  target_pose.orientation.z = 0.707;
  target_pose.orientation.x = 0;
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;  

  //set tolerance
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.01);
  switch (*argv[1])
  {
    case 'o':
    {
      //set target
      move_group.setPoseTarget(target_pose);

      //set planning time
      move_group.setPlanningTime(5.0);

      ROS_INFO("**********************************************");
      ROS_INFO("Planning...");
      int iter = 0;
      while (!move_group.plan(my_plan))
      {
        iter++;
        if (iter > 5)
        {
          ROS_INFO("Failed");
          break;
        }
      }
      break;
    }
    case 'c':
    {
      //IsetJointPositionst seems that CHOMP can only deal with joint space target. 
      //We first solve ik problem for pose target than pass the solution to CHOMP.
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();//ik solver handle
      std::vector<double> original_positions;
      std::vector<double> goal_positions;
      current_state->copyJointGroupPositions(joint_model_group, original_positions);
      current_state->setFromIK(joint_model_group, target_pose, 3, 0.5);
      current_state->copyJointGroupPositions(joint_model_group, goal_positions);
      
      std::vector<std::string> joint_names = move_group.getJointNames();

      for (int i = 0; i < original_positions.size(); i++)
      {
        const std::string& joint_name_temp = joint_names[i];
        current_state->setJointPositions(joint_name_temp, &original_positions[i]);
        //recover joint to original values
        ROS_INFO("Origin[%d]: %f", i, original_positions[i]);
        ROS_INFO("Goal[%d]: %f", i, goal_positions[i]);
      }
      //set target
      move_group.setJointValueTarget(goal_positions);
      
      //set planning time
      move_group.setPlanningTime(5.0);

      ROS_INFO("**********************************************");
      ROS_INFO("Planning...");
      int iter = 0;
      while (!move_group.plan(my_plan))
      {
        iter++;
        if (iter > 5)
        {
          ROS_INFO("Failed");
          break;
        }
      }
      break;

    }
  } 
  sleep_one_second.sleep();
  return 0;

}
