#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

double compute_length(moveit_msgs::RobotTrajectory* traj_msg,
                      robot_model::RobotModelConstPtr r_model,
                      robot_model::JointModelGroup* j_group,
                      robot_state::RobotStatePtr ref_state)
//use this function before execution of the plan.
{
  robot_trajectory::RobotTrajectory robot_traj(r_model, j_group);
  robot_traj.setRobotTrajectoryMsg(*ref_state, *traj_msg);
  
  double length = 0.0;
  std::vector<Eigen::Vector3d> way_points;

  for (int i = 0; i < robot_traj.getWayPointCount(); i++)
  {  
    const Eigen::Affine3d& ee_pose = robot_traj.getWayPoint(i).getGlobalLinkTransform("ee_link");
    way_points.push_back(ee_pose.translation());
  }

  for (int i = 1; i < robot_traj.getWayPointCount(); i++)
  {
    Eigen::Vector3d distance = way_points[i] - way_points[i-1];
    length += sqrt(distance.dot(distance));
  }
  
  ROS_INFO("Way points number: %d", (int)robot_traj.getWayPointCount());
  
  return length;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_length");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");
  
  ROS_INFO_STREAM("End effector:"<<end_effector_state.translation());
  
  geometry_msgs::Pose pose;
  pose.position.x = 0.7;
  pose.position.y = 0.7;
  pose.position.z = 0.7;
  pose.orientation.w = 1.0;

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  move_group.setPoseTarget(pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  int iter = 0;
  while(!move_group.plan(my_plan))
  {
    iter++;
    if (iter > 5)
      break;
  }

//  double compute_length(moveit_msgs::RobotTrajectory* traj_msg,
//                      robot_model::RobotModelPtr r_model,
//                      robot_model::JointModelGroup* j_group
//                      robot_state::RobotStatePtr ref_state)

  ROS_INFO("Length: %f",compute_length(&(my_plan.trajectory_), move_group.getRobotModel(),
                joint_model_group, move_group.getCurrentState()));
  
  move_group.execute(my_plan);
   
  Eigen::Affine3d ee_state = move_group.getCurrentState()->getGlobalLinkTransform("ee_link");
  ROS_INFO_STREAM("End effector:" << ee_state.translation());
  
  
  
  return 0;
}


