#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
  //ros init
  ros::init(argc, argv, "publish_shelf");
  ros::AsyncSpinner spinner(1);
  spinner.start();
    
  ros::NodeHandle node_handle;
  ros::Duration sleep_one_second(1.0);

  //get planning interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  const robot_state::JointModelGroup *joint_model_group = 
    move_group.getCurrentState()->getJointModelGroup("manipulator");
  
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;

// Construct the shelf and the ball target

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
/* For now, we don't pick the ball
  moveit_msgs::AttachedCollisionObject target;
  target.link_name = "ee_link";
  target.object.header.frame_id = "base_link";
  target.object.id = "target";
  target.object.primitives.push_back(ball);
  target.object.primitive_poses.push_back(pose);
  target.object.operation = side_board[0].ADD;
*/
  moveit_msgs::CollisionObject ball_object;
  ball_object.header.frame_id = "base_link";
  ball_object.id = "ball_object";
  ball_object.primitives.push_back(ball);
  ball_object.primitive_poses.push_back(pose);
  ball_object.operation = ball_object.ADD;

  planning_scene.world.collision_objects.push_back(ball_object);

  planning_scene_interface.applyPlanningScene(planning_scene);

  sleep_one_second.sleep();

  return (0);

}
