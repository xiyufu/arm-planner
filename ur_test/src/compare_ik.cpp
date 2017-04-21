#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>

int main( int argc, char **argv )
{
  ros::init(argc, argv, "compare_ik");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Duration sleep_one_second(1.0);
  ros::Duration sleep_short_time(0.3);
  
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene",1);

  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    sleep_one_second.sleep();

  //construct a planning scene msg
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame : %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

  //Get joint values
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  //Set goal pose in work space
  geometry_msgs::Pose target_pose[5][5];

  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++)
    {
      target_pose[i][j].position.y = 0.0;
      target_pose[i][j].position.z = 0.7 - 0.1*i;
      target_pose[i][j].position.x = 0.7 + 0.1*j;
      target_pose[i][j].orientation.w = 1.0;
    }
  
  ROS_INFO("************************");
  ROS_INFO("Start solving IK problems");  
  
  
  //Found ik solution
  bool success = false;
  int failure_counter = 0;
  double mean_time = 0;
  int iter = 0;
  ros::Time ts = ros::Time::now();
  ros::Time te = ros::Time::now();
  for (iter = 0; iter < 5; iter++)
  {
    for (int i = 0; i < 5; i++)
    {
      for (int j = 0; j < 5; j++)
      { 
        ts = ros::Time::now();
        success = kinematic_state->setFromIK(joint_model_group, target_pose[i][j], 10, 0.1);
        te = ros::Time::now();
        mean_time += te.toSec() - ts.toSec(); 
        if (success)
        {
          /*kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          for (std::size_t i = 0; i < joint_values.size(); i++)
          {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }*/
          robot_state::robotStateToRobotStateMsg(*kinematic_state, planning_scene.robot_state);
          planning_scene_diff_publisher.publish(planning_scene);
          sleep_short_time.sleep();
        }
        else
          ROS_INFO("IK failed times: %d", ++failure_counter);
      }
    }
  }
  mean_time = mean_time/double(5)/25.0;
  ROS_INFO("IK solver testing finished, failure number: %d/25. mean time = %f", failure_counter, mean_time);
  ROS_INFO("************************");
  sleep_one_second.sleep();
  
  return 0;

}



