#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_test");
  ros::NodeHandle node_handel;
  ros::AsyncSpinner spinner(1);
  spinner.start();
//init move group interface
  static const std::string PLANNING_GROUP = "manipulator";
  
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.5;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();

  ROS_INFO_NAMED("tests", "Reference frame: %s", move_group.getPlanningFrame().c_str());
//set target pose
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.7;
  target_pose1.position.y = 0.7;
  target_pose1.position.z = 0.7;
  move_group.setPoseTarget(target_pose1);

//add obstacles
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  collision_object.id = "Box";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.2;
  box_pose.position.y = 0.4;
  box_pose.position.z = 0.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects
;
  collision_objects.push_back(collision_object);

  ROS_INFO("Add a box into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
//planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = move_group.plan(my_plan);

  ROS_INFO_NAMED("test", "Visual plan 1 (pose goal) %s", success? " " : "Failed");

  ROS_INFO_NAMED("test", "Visual plan 1 (pose goal)");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  ros::shutdown();

  return 0;
}
