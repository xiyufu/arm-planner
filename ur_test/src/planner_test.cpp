#include <cmath>
#include <stdio.h>

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
  //ros init
  ros::init(argc, argv, "planner_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Duration sleep_one_second(1.0);//publishing needs some time.
  ros::Duration sleep_short_time(0.5);

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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
  /* Let's foucs on OMPL first 
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
  } */

  robot_model::RobotModelConstPtr robot_model = move_group.getRobotModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));

  ROS_INFO("********First attempt with max_time = 10.0********");
  move_group.setPlanningTime(10.0);
  move_group.setPoseTarget(target_pose);
  
  bool success = false;
  do
  {
    success = move_group.plan(my_plan);
    ROS_INFO("Failed");
  }
  while (!success);

  double largest_jerk_cost = 0.0;
  double shortest_length = 0.0;
  double longest_length = 0.0;
  double smallest_jerk_cost = 0.0;
  int best_index = 0;
  int worst_index = 0;

  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> all_plan;

  double first_info[2] = {0.0, 0.0};
  trajInfo(&(my_plan.trajectory_), robot_model, joint_model_group, kinematic_state, first_info);
  ROS_INFO("Planning successed");
  ROS_INFO("Planning time: %f", my_plan.planning_time_);
  ROS_INFO("Path length: %f", first_info[0]);
  ROS_INFO("Path smoothness: %f", first_info[1]);
  
  all_plan.push_back(my_plan);
  longest_length = first_info[0];
  shortest_length = first_info[0];

  ROS_INFO("Press b to start test, press any other key to quit");
  char is_test = 'n';
  std::cin >> is_test;
  if ( is_test == 'b' )
  {
    ROS_INFO("Show all planning time? y/n");
    char is_show = 'n';
    std::cin >> is_show;
    bool show_flag = (is_show == 'y');    
    std::vector<double> all_planning_time;
    std::vector<double> all_path_length;

    move_group.clearPoseTargets();
    move_group.setPoseTarget(target_pose);
    ROS_INFO("Set total attempt times");
    int total_attempt = 1;
    std::cin >> total_attempt;

    double max_time = 2.0;
    ROS_INFO("Set maxium planning time");
    std::cin >> max_time;
    move_group.setPlanningTime(max_time);

    double number_success = 0.0;
    double my_planning_time = 0.0;
    double my_length = 0.0;
    double my_smoothness = 0.0;

    ROS_INFO("Test started");
    for (int i = 0; i < total_attempt; i++)
    {
      ROS_INFO("********%dth iteration********", (i + 1));
      success = move_group.plan(my_plan);
      
      if (success)
      {
        number_success += 1;
        my_planning_time += my_plan.planning_time_;
        
        double temp_info[2] = {0.0, 0.0};
        trajInfo(&(my_plan.trajectory_), robot_model, joint_model_group, kinematic_state, temp_info);
        my_length += temp_info[0];
        my_smoothness += temp_info[1];
        
        if (show_flag)
        {
          all_planning_time.push_back(my_plan.planning_time_);
          all_path_length.push_back(temp_info[0]);
        }
        all_plan.push_back(my_plan);
        if (temp_info[0] > longest_length)
        {
          worst_index = number_success;
          longest_length = temp_info[0];
        }
        if (temp_info[0] < shortest_length)
        {
          best_index = number_success;
          shortest_length = temp_info[0];
        }
      }

    }
      
    if (number_success == 0)
    {
      ROS_INFO("number_success = 0");
      return (0);
    }

    double success_rate = number_success/((double)total_attempt);
    my_planning_time = my_planning_time/number_success;
    my_length = my_length/number_success;
    my_smoothness = my_smoothness/number_success;

    ROS_INFO("Rate: %f\nTime: %f\nLength: %f\nSmoothness: %f",
              success_rate, my_planning_time, my_length, my_smoothness);

    if (show_flag)
    {
      ROS_INFO("Press any key to show all planning time");
      char out_pause;
      std::cin >> out_pause;
      
      FILE *pFileTime;
      FILE *pFileLength;

      pFileTime = fopen("/home/xy-fu/RRTConnectPlanningTime1000-50s.txt", "w");
      pFileLength = fopen("/home/xy-fu/RRTConnectPathLength1000-50s.txt", "w");
 
      for (double a_time : all_planning_time)
      {
        fprintf(pFileTime, "%6.4f \n",a_time);
        printf("%6.4f\n", a_time);
      }
      for (double a_length : all_path_length)
      {
        fprintf(pFileLength, "%6.4f \n", a_length);
      }
      fclose(pFileTime);
      fclose(pFileLength);
    }

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    ROS_INFO("Now subscribe to /rviz_visual_tools in Rviz GUI and then press any key + enter to continue");
    char vis_pause;
    std::cin >> vis_pause;
    
    ROS_INFO("BEST LENGTH: %f, WORST LENGTH: %f", shortest_length, longest_length);

    visual_tools.deleteAllMarkers();
    rvt::colors my_color = rvt::colors::BLUE;
    visual_tools.publishTrajectoryLine(all_plan[best_index].trajectory_, kinematic_state->getLinkModel("ee_link"), joint_model_group, my_color);
    visual_tools.trigger();
    sleep_one_second.sleep();
    my_color = rvt::colors::RED;
    visual_tools.publishTrajectoryLine(all_plan[worst_index].trajectory_, kinematic_state->getLinkModel("ee_link"), joint_model_group, my_color);
    visual_tools.trigger();
    sleep_one_second.sleep();
    sleep_one_second.sleep();
    sleep_one_second.sleep();

  }
  sleep_one_second.sleep();
  return 0;

}
