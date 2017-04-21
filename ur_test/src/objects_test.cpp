#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv, "objects_test");

  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration sleep_time(2);
  ros::Duration short_sleep_time(1);
  sleep_time.sleep();

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }

  //Define an object - a BOX
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  //Define a pose
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.5;
  pose.position.y = 0.5;
  pose.position.z = 0.5;

  //Define a collision object
  moveit_msgs::CollisionObject target_object;
  target_object.id = "box";
  target_object.header.frame_id = "ee_link";
  target_object.primitives.push_back(primitive);
  target_object.primitive_poses.push_back(pose);

  target_object.operation = target_object.ADD;

  //Define an attached object
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "ee_link";
  attached_object.object = target_object;

  ROS_INFO("Adding the object into the world at the end effector.");
  
  //Define a planning scene msg
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(target_object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  short_sleep_time.sleep();

  //Attache the object to robot
  //First, remove it from the world
  moveit_msgs::CollisionObject remove_object;
  remove_object = target_object;
  remove_object.operation = remove_object.REMOVE;
  ROS_INFO("Removing object from world.");  
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  short_sleep_time.sleep();
  //Second, attach it to the end effector
  ROS_INFO("Attaching object to the robot.");
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  
  planning_scene_diff_publisher.publish(planning_scene);

  short_sleep_time.sleep();
  
  //Detach the object from the robot
  //detach
  ROS_INFO("Detaching the object from the robot.");
  attached_object.object.operation = target_object.REMOVE;
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  
//  planning_scene_diff_publisher.publish(planning_scene);
  short_sleep_time.sleep();
  
  //add to the world
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(target_object);
  
//  planning_scene_diff_publisher.publish(planning_scene);

  return 0;
}
