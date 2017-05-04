#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

//Note: in config/ompl_planning.yaml, there is a term called:
//longest_valid_segement_fraction. This is a very important factor
//The default setting in universial robot (0.05) is too large, 0.005~0.001
//is a better choice. Smaller value costs more time but is (a lot) more
//likely to find a solution.

double trajInfo(moveit_msgs::RobotTrajectory* traj_msg,
                robot_model::RobotModelConstPtr r_model,
                const robot_model::JointModelGroup* j_group,
                robot_state::RobotStatePtr ref_state)
//trajInfo should be used before move_group.execute because 
//we need the reference state.
{
  robot_trajectory::RobotTrajectory robot_traj(r_model, j_group);
  robot_traj.setRobotTrajectoryMsg(*ref_state, *traj_msg);

  double length = 0.0;
  std::vector<Eigen::Vector3d> way_points;

  int way_point_count = (int)robot_traj.getWayPointCount();
  for (int i = 0; i < way_point_count; i++)
  {
    const Eigen::Affine3d& ee_pose = robot_traj.getWayPoint(i).getGlobalLinkTransform("ee_link");
    way_points.push_back(ee_pose.translation());
  }

  for (int i = 1; i < way_point_count; i++)
  {
    Eigen::Vector3d distance = way_points[i] - way_points[i-1];
    length += sqrt(distance.dot(distance));
  }
  
  ROS_INFO("Way point number: %d", way_point_count);
  
  return length;
}


int main(int argc, char **argv)
{
  //Start ros
  ros::init(argc, argv, "pick_and_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Duration sleep_one_second(1.0);
  ros::Duration sleep_short_time(0.5);

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_one_second.sleep();
  }  

  //Construct planning scene message
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true; //null will be considered as the same.

  //Construct environment-table
  moveit_msgs::CollisionObject table_box;
  table_box.id = "table";
  table_box.header.frame_id = "base_link";
  
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2.0;
  primitive.dimensions[1] = 1.2;
  primitive.dimensions[2] = 0.5;

  geometry_msgs::Pose pose;//This pose refers to the center of primitives.
  pose.position.x = 0.0;
  pose.position.y = 1.1;
  pose.position.z = 0.1;
  pose.orientation.w = 1.0;

  table_box.primitives.push_back(primitive);
  table_box.primitive_poses.push_back(pose);
  table_box.operation = table_box.ADD;

  planning_scene.world.collision_objects.push_back(table_box);
  
  ROS_INFO("Publishing...");
  //publish table
  planning_scene.world.collision_objects[0].operation = table_box.REMOVE;
  planning_scene_diff_publisher.publish(planning_scene);
  planning_scene.world.collision_objects[0].operation = table_box.ADD;
  planning_scene_diff_publisher.publish(planning_scene);
  sleep_short_time.sleep();

  //Construct environment-obstacle
  moveit_msgs::CollisionObject obstacle1;
  obstacle1.id = "obstacle_1";
  obstacle1.header.frame_id = "base_link";
  
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 1.0;
  primitive.dimensions[2] = 0.3;

  pose.position.z = 0.5;

  obstacle1.primitives.push_back(primitive);
  obstacle1.primitive_poses.push_back(pose);
  obstacle1.operation = obstacle1.REMOVE;

  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(obstacle1);
  planning_scene_diff_publisher.publish(planning_scene);
  planning_scene.world.collision_objects[0].operation = obstacle1.ADD;
  planning_scene_diff_publisher.publish(planning_scene);
  
  sleep_short_time.sleep();

  //Object to pick
  moveit_msgs::AttachedCollisionObject target;
  target.link_name = "ee_link";
  target.object.header.frame_id = "base_link";
  target.object.id = "target";
  
  primitive.type = primitive.SPHERE;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;

  pose.position.x = 0.3;
  pose.position.y = 0.7;
  pose.position.z = 0.45;

  target.object.primitives.push_back(primitive);
  target.object.primitive_poses.push_back(pose);
  target.object.operation = table_box.REMOVE;
  
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(target.object);

  planning_scene_diff_publisher.publish(planning_scene);

  planning_scene.world.collision_objects[0].operation = table_box.ADD;
  
  planning_scene_diff_publisher.publish(planning_scene);

  //Get move group interface
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup *joint_model_group = 
    move_group.getCurrentState()->getJointModelGroup("manipulator");
  
  //Target pose 
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.6;
  target_pose.position.z = 0.45;
  target_pose.orientation.x = 0;
  target_pose.orientation.y = 0;
  target_pose.orientation.z = 0.707;
  target_pose.orientation.w = 0.707;//Pick from the edge of table
  
  //Tolerance
  double tolerance_position = 0.01;
  double tolerance_orientation = 0.01;
  move_group.setGoalPositionTolerance(tolerance_position);
  move_group.setGoalOrientationTolerance(tolerance_orientation);
  
  move_group.setPoseTarget(target_pose);
  move_group.setPlanningTime(10.0);//Let's hope we can find a trajectory
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  ROS_INFO("********************************");
  ROS_INFO("Planning...");
  bool success = move_group.plan(my_plan);
  int iter = 1;
  while(!success)
  {
    success = move_group.plan(my_plan);
    iter++;
    if (iter > 20)
      break;
  }
  ROS_INFO("Planning %s after %d attempts", success ? "successed" : "failed", iter);
/*double trajInfo(moveit_msgs::RobotTrajectory* traj_msg,
                moveit_msgs::RobotModelConsPtr r_model,
                robot_model::JointModelGroup* j_group,
                robot_state::RobotStatePrt ref_state)
*/
  if (success)
  {
    ROS_INFO("Planning time: %6.3f", my_plan.planning_time_);
    double length = trajInfo(&(my_plan.trajectory_), move_group.getRobotModel(),
                              joint_model_group, move_group.getCurrentState());
    ROS_INFO("Trajectory length: %f", length);
  }
  ROS_INFO("********************************");
  
  //Execute
  ROS_INFO("Executing the plan......");

  success = move_group.execute(my_plan);
  if (!success)
    ROS_INFO("Somehow execution failed");
  else
    ROS_INFO("Execution successed");

  char just_to_pause;
  ROS_INFO("Press 'g' to continue");
  std::cin >> just_to_pause;

  //Pick up, attach the target to robot
  ROS_INFO("Picking up......");

  //First remove the target from the world
  target.object.operation = target.object.REMOVE;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(target.object);
  planning_scene_diff_publisher.publish(planning_scene);
  sleep_short_time.sleep();

  //Second, add the target to robot model
  target.object.operation = target.object.ADD;
  planning_scene.world.collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(target);
  planning_scene_diff_publisher.publish(planning_scene);

  sleep_short_time.sleep();

  //Now take the ball to another place.
  target_pose.position.x = -0.3;
  target_pose.position.y += 0.01;
  move_group.setPoseTarget(target_pose);
  ROS_INFO("********************************");
  ROS_INFO("Planning...");
  iter = 0; 
  do
  {
    success = move_group.plan(my_plan);
    iter++;
    if (iter > 20)
      break;
  }
  while (!success);
  ROS_INFO("Planning %s after %d attempts", success ? "successed" : "failed", iter);
  if (success)
  {
    ROS_INFO("Planning time: %6.3f", my_plan.planning_time_);
    double length = trajInfo(&(my_plan.trajectory_), move_group.getRobotModel(),
                              joint_model_group, move_group.getCurrentState());
    ROS_INFO("Trajectory length: %f", length);
  }
  ROS_INFO("********************************");
  
  if (success)
  {
    ROS_INFO("Executing the plan......");
    success = move_group.execute(my_plan);
    ROS_INFO("Execution %s", success ? "successed" : "failed");  
  }
  else
    ROS_WARN("Planning failed after 20 attempts");
  
  //Put the ball on the table
  target.object.operation = target.object.REMOVE;
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(target);
  planning_scene_diff_publisher.publish(planning_scene);
  sleep_short_time.sleep();

  target.object.primitive_poses[0].position.x = -0.3;  
  target.object.operation = target.object.ADD;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(target.object);
  planning_scene_diff_publisher.publish(planning_scene);

  sleep_one_second.sleep(); //Sleep for a while is necessary 
  return(0);

}


