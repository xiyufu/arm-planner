#include <cmath>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/Constraints.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

typedef Eigen::Matrix<double, 6, 1> vec6d;

void trajInfo(moveit_msgs::RobotTrajectory* traj_msg,
                robot_model::RobotModelConstPtr r_model,
                const robot_model::JointModelGroup* j_group,
                robot_state::RobotStatePtr ref_state,
                double* info_pair)
//trajInfo should be called before move_group.execute because 
//we need the reference state.
{
  robot_trajectory::RobotTrajectory robot_traj(r_model, j_group);
  robot_traj.setRobotTrajectoryMsg(*ref_state, *traj_msg);

  info_pair[0] = 0.0;
  std::vector<Eigen::Vector3d> way_points;

  int way_point_count = (int)robot_traj.getWayPointCount();
  for (int i = 0; i < way_point_count; i++)
  {
    const Eigen::Affine3d& ee_pose = robot_traj.getWayPoint(i).getGlobalLinkTransform("ee_link");
    way_points.push_back(ee_pose.translation());
//    ROS_INFO_STREAM("iteration " << i <<": " << way_points[i]);
  }

  for (int i = 1; i < way_point_count; i++)
  {
    Eigen::Vector3d distance = way_points[i] - way_points[i-1];
    info_pair[0] += sqrt(distance.dot(distance));
  }

  /****** smoothness ******/
  vec6d accelerations[robot_traj.getWayPointCount()];//vec6d is defined at the beginning of this file.
  std::deque<double> durations = robot_traj.getWayPointDurations();
  std::vector<trajectory_msgs::JointTrajectoryPoint> points = traj_msg->joint_trajectory.points;
  for (int i = 0; i < way_point_count; i++)
  {
    vec6d temp(points[i].accelerations.data());
    accelerations[i] = temp;
  }
  info_pair[1] = 0.0;//smoothness is measured by jerk cost 
  for (int i = 1; i < way_point_count; i++)
  {
    vec6d acc_diff = accelerations[i] - accelerations[i-1];
    double time_span = (durations[i] + durations[i-1])/2;
    vec6d jerk = acc_diff/time_span;
    info_pair[1] += jerk.dot(jerk) * time_span;
  }
  //ROS_INFO("Way point number: %d", way_point_count);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_line");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration sleep_short_time(0.5);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  const robot_state::JointModelGroup *joint_model_group = 
    move_group.getCurrentState()->getJointModelGroup("manipulator");
  robot_model::RobotModelConstPtr robot_model = move_group.getRobotModel();
  robot_state::RobotStatePtr kinematic_state = move_group.getCurrentState();
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;

//start_pose and goal pose
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose goal_pose;
  
  double start[3] = {0.0};
  for (int i = 0; i < 3; i++)
  {
    ROS_INFO("Start position x%d = ?", (i + 1));
    std::cin >> start[i];
  }
  double start_orien[4] = {0.0};
  for (int i = 0; i< 4; i++)
  {
    ROS_INFO("Start orientation x%d = ?", (i + 1));
    std::cin >>start_orien[i];
  }
  
  double goal[3] = {0.0};
  for (int i = 0; i < 3; i++)
  {
    ROS_INFO("Goal position x%d = ?", (i + 1));
    std::cin >> goal[i];
  }
  double goal_orien[4];
  for (int i = 0; i < 4; i++)
  {
    ROS_INFO("Goal orientation x%d = ?", (i + 1));
    std::cin >> goal_orien[i];
  }
  
  start_pose.position.x = start[0];
  start_pose.position.y = start[1];
  start_pose.position.z = start[2];
  start_pose.orientation.w = start_orien[0];
  start_pose.orientation.x = start_orien[1];
  start_pose.orientation.y = start_orien[2];
  start_pose.orientation.z = start_orien[3];


  goal_pose.position.x = goal[0];
  goal_pose.position.y = goal[1];
  goal_pose.position.z = goal[2];
  goal_pose.orientation.w = goal_orien[0];
  goal_pose.orientation.x = goal_orien[1];
  goal_pose.orientation.y = goal_orien[2];
  goal_pose.orientation.z = goal_orien[3];

//Set constraints
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "ee_link";
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
//  move_group.setPathConstraints(test_constraints);
//Drive the robot arm to start position
  std::vector<double> start_joint_values;
  std::vector<double> goal_joint_values;

  bool found_ik = false;
  int iter_ik = 0;
  while (!found_ik)
  {
    found_ik = kinematic_state->setFromIK(joint_model_group, goal_pose, 100, 0.01);
    std::cout << "end IK failed" << std::endl;
    iter_ik++;
    if(iter_ik > 10)
      return 1;
  }
  kinematic_state->copyJointGroupPositions(joint_model_group, goal_joint_values);
  robot_state::robotStateToRobotStateMsg(*kinematic_state, planning_scene.robot_state);
  planning_scene_interface.applyPlanningScene(planning_scene);
 
  found_ik = false;
  iter_ik = 0;
  while (!found_ik)
  { 
    found_ik = kinematic_state->setFromIK(joint_model_group, start_pose, 100, 0.01);
    std::cout << "start IK failed" << std::endl;
    iter_ik++;
    if(iter_ik > 10)
      return 1;
  }
  kinematic_state->copyJointGroupPositions(joint_model_group, start_joint_values);
  robot_state::robotStateToRobotStateMsg(*kinematic_state, planning_scene.robot_state);
  planning_scene_interface.applyPlanningScene(planning_scene);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> all_plan;

  int best_index = -1;
  double shortest_length = 1000;
  
  bool success = false;

  ROS_INFO("Choose planning mode,\n press o for ompl,\n c for chomp,\n i for cartesian interpolation,\n s for sbpl(don't do this, not finished yet)");

  char flag = 'n';
  std::cin >> flag;

  double number_success = 0.0;
  double my_planning_time = 0.0;
  double my_length = 0.0;
  double my_smoothness = 0.0;

  switch (flag)
  {
    case 'o': 
    {
      ROS_INFO("Total attempts = ?");
      int total_attempts = 1;
      std::cin >> total_attempts;
      ROS_INFO("Maximum planning time = ?");
      double max_time = 1.0;
      std::cin >> max_time;
      move_group.setPlanningTime(max_time);
      move_group.setJointValueTarget(goal_joint_values);

      for (int i = 0; i < total_attempts; i++)
      {
        ROS_INFO("********%dth iteration********", (i + 1));
        success = move_group.plan(my_plan);
  
        if (success)
        {
          all_plan.push_back(my_plan); 
          number_success += 1;
          my_planning_time += my_plan.planning_time_;
          double temp_info[2] = {0.0, 0.0};
          trajInfo(&(my_plan.trajectory_), robot_model, joint_model_group, kinematic_state, temp_info);
          my_length += temp_info[0];
          my_smoothness += temp_info[1];
          if(temp_info[0] < shortest_length)
          {
            best_index = number_success - 1;
            shortest_length = temp_info[0];
          }
        }
      }
      
      double successful_rate = number_success/((double)total_attempts);
      my_planning_time = my_planning_time/number_success;
      my_length = my_length/number_success;
      my_smoothness = my_smoothness/number_success;

      ROS_INFO("Success rate: %f\nPlanning time: %f\nLength: %f\nSmoothness: %f", successful_rate, my_planning_time, my_length, my_smoothness);
    } break;
    case 'c':
    {
      ROS_INFO("Total attempts = ?");
      int total_attempts = 1;
      std::cin >> total_attempts;
      ROS_INFO("Maximum planning time = ?");
      double max_time = 1.0;
      std::cin >> max_time;
      move_group.setPlanningTime(max_time);
      move_group.setJointValueTarget(goal_joint_values);

      for (int i = 0; i < total_attempts; i++)
      {
        ROS_INFO("********%dth iteration********", (i + 1));
        success = move_group.plan(my_plan);

        if (success)
        {
          all_plan.push_back(my_plan);
          number_success += 1;
          my_planning_time += my_plan.planning_time_;
          double temp_info[2] = {0.0, 0.0};
          trajInfo(&(my_plan.trajectory_), robot_model, joint_model_group, kinematic_state, temp_info);
          my_length += temp_info[0];
          my_smoothness += temp_info[1];
          if(temp_info[0] < shortest_length)
          {
            best_index = number_success - 1;
            shortest_length = temp_info[0];
          }
        }
      }

      double successful_rate = number_success/((double)total_attempts);
      my_planning_time = my_planning_time/number_success;
      my_length = my_length/number_success;
      my_smoothness = my_smoothness/number_success;

      ROS_INFO("Successful rate: %f\nPlanning time: %f\nLength: %f\nSmoothness: %f", successful_rate, my_planning_time, my_length, my_smoothness);
    } break;
    case 'i':
    {
      ROS_INFO("jump_threshold = ?");
      double jump_threshold = 0.0;
      std::cin >> jump_threshold;
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(goal_pose);
      
      ros::Time begin;
      ros::Time end;

      begin = ros::Time::now();
      double fraction = move_group.computeCartesianPath(waypoints, 0.01, jump_threshold, my_plan.trajectory_);
      end = ros::Time::now();
     
      ROS_INFO("Cartesian path %f%% acheved", fraction*100.0); 
      my_planning_time = end.toSec() - begin.toSec();
      double temp_info[2] = {0.0, 0.0};
      trajInfo(&(my_plan.trajectory_), robot_model, joint_model_group, kinematic_state, temp_info);
      my_length += temp_info[0];
      my_smoothness += temp_info[1];


      ROS_INFO("Planning time: %f\nLength: %f\nSmoothness: %f", my_planning_time, my_length, my_smoothness);
    } break;
  }

  //visualize the trajectory
  visual_tools.deleteAllMarkers();
  if (best_index == -1)//Cartesian case
  {
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, kinematic_state->getLinkModel("ee_link"), joint_model_group, rvt::colors::BLUE);
    visual_tools.trigger();
    sleep_short_time.sleep();
  }
  else
  {
    visual_tools.publishTrajectoryLine(all_plan[best_index].trajectory_, kinematic_state->getLinkModel("ee_link"), joint_model_group, rvt::colors::BLUE);
    visual_tools.trigger();
    sleep_short_time.sleep();
  }
  return (0);

}

