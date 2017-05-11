#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
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

//get planning interfaces
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup("manipulator");

  planning_scene::PlanningScene planning_scene_obj(move_group.getRobotModel());
//BE CAREFUL: This planning scene has no connection to the one in move group. We have to maintain it by ourselves.

  //Construct planning scene message
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true; //null will be considered as the same.

  //******Construct environment-table******
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
  //planning_scene.world.collision_objects[0].operation = table_box.REMOVE;
  //planning_scene_diff_publisher.publish(planning_scene);
  planning_scene.world.collision_objects[0].operation = table_box.ADD;
  //planning_scene_diff_publisher.publish(planning_scene);
  //sleep_short_time.sleep();

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
  obstacle1.operation = obstacle1.ADD;

  planning_scene.world.collision_objects.push_back(obstacle1);

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
  target.object.operation = table_box.ADD;
  
  //planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(target.object);

  //planning_scene_diff_publisher.publish(planning_scene);

  //planning_scene.world.collision_objects[0].operation = table_box.ADD;
  
  //planning_scene_diff_publisher.publish(planning_scene);
  sleep_short_time.sleep();

  planning_scene_interface.applyPlanningScene(planning_scene);
  
  //Target pose 
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.59;
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
  double info_pair[2] = {0.0, 0.0};

  if (success)
  {
    ROS_INFO("Planning time: %6.3f", my_plan.planning_time_);
    trajInfo(&(my_plan.trajectory_), move_group.getRobotModel(),
               joint_model_group, move_group.getCurrentState(), info_pair);
    ROS_INFO("Trajectory length: %f\nTrajectory smoothness: %f", info_pair[0], info_pair[1]);
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
  planning_scene.world.collision_objects.pop_back();
  planning_scene.world.collision_objects.push_back(target.object);
  planning_scene_interface.applyPlanningScene(planning_scene);
//  planning_scene_diff_publisher.publish(planning_scene);
  sleep_short_time.sleep();

  //Second, add the target to robot model
  target.object.operation = target.object.ADD;
  planning_scene.world.collision_objects.pop_back();
  planning_scene.robot_state.attached_collision_objects.push_back(target);
//  planning_scene_diff_publisher.publish(planning_scene);
  planning_scene_interface.applyPlanningScene(planning_scene);
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
    trajInfo(&(my_plan.trajectory_), move_group.getRobotModel(),
             joint_model_group, move_group.getCurrentState(), info_pair);
    ROS_INFO("Trajectory length: %f\nTrajectory smoothness: %f", info_pair[0], info_pair[1]);
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
//  planning_scene_diff_publisher.publish(planning_scene);
  planning_scene_interface.applyPlanningScene(planning_scene);
  sleep_short_time.sleep();

  target.object.primitive_poses[0].position.x = -0.3;  
  target.object.operation = target.object.ADD;
  //planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(target.object);
//  planning_scene_diff_publisher.publish(planning_scene);
  planning_scene_interface.applyPlanningScene(planning_scene);



  //*****Batch test*****//
  robot_model::RobotModelConstPtr robot_model = move_group.getRobotModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
  //planning_scene is used for collision checking. 
  ROS_INFO("Press b to start batch test, press any other keys to quit");
  char command;
  std::cin >> command;
  if ( command == 'b' )
  {
    ROS_INFO("Set attempt times (of type int)");
    int total_attempt = 1;
    std::cin >> total_attempt;
    ROS_INFO("The test will take some time; find something else to do");

    geometry_msgs::Pose pose1;
    geometry_msgs::Pose pose2;
    
    pose1.position.x = 0.3;
    pose1.position.y = 0.59;
    pose1.position.z = 0.45;
    pose1.orientation.x = 0;
    pose1.orientation.y = 0;
    pose1.orientation.z = 0.707;
    pose1.orientation.w = 0.707;

    pose2.position.x = -0.3;
    pose2.position.y = 0.59;
    pose2.position.z = 0.45;
    pose2.orientation.x = 0;
    pose2.orientation.y = 0;
    pose2.orientation.z = 0.707;
    pose2.orientation.w = 0.707;
  
    double number_success[2] = {0.0, 0.0};
    double my_planning_time[2] = {0.0, 0.0};
    double my_length[2] = {0.0, 0.0};
    double my_smoothness[2] = {0.0, 0.0};
    success = false;
    ROS_INFO("Using multi-query algorithm? y/n");
    char is_multi = 'n';
    std::cin >> is_multi;
    double time_multi = 2.0;
    if (is_multi == 'y')
    {
      ROS_INFO("Maxim planning time set as __ ?");
      std::cin >> time_multi;
    }//We will allow multi-query planner to build the map in 2 seconds. And set the planning time later.
    move_group.setPlanningTime(2.0);
/*
    //prepare to check collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    //Compute the goal and start position in advance 
    std::vector<double> joint_values1;
    std::vector<double> joint_values2;
    bool found_ik = false;
    bool no_collision = false;
    while ( !(found_ik && no_collision) )
    {
      //This is not very efficient...
      found_ik = kinematic_state->setFromIK(joint_model_group, pose1, 100, 0.001);
      //publish new planning_scene
      robot_state::robotStateToRobotStateMsg(*kinematic_state, planning_scene.robot_state);
      //check collision
      collision_result.clear();
      planning_scene_obj.setPlanningSceneDiffMsg(planning_scene);
      planning_scene_obj.checkCollision(collision_request, collision_result);
      no_collision = !(collision_result.collision);
    }
    planning_scene_interface.applyPlanningScene(planning_scene);
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values1);
    
    found_ik = false;
    no_collision = false;
    while ( !(found_ik && no_collision) )
    {
      //Get Inverse Kinematic solutions
      found_ik = kinematic_state->setFromIK(joint_model_group, pose2, 100, 0.001);
      //Check collision
      collision_result.clear();
      robot_state::robotStateToRobotStateMsg(*kinematic_state, planning_scene.robot_state);
      //Update our own planning scene so that it matches the one in move group.
      planning_scene_obj.setPlanningSceneDiffMsg(planning_scene);
      planning_scene_obj.checkCollision(collision_request, collision_result);
      no_collision = !(collision_result.collision);
    }
    planning_scene_interface.applyPlanningScene(planning_scene);
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values2);
    
    //Put the ball to original position
    target.object.operation = target.object.REMOVE;
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.robot_state.attached_collision_objects.push_back(target);
    planning_scene_diff_publisher.publish(planning_scene);
    sleep_short_time.sleep();

    target.object.primitive_poses[0].position.x = 0.3;
    target.object.primitive_poses[0].position.y += 0.1;
    target.object.operation = target.object.ADD;
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(target.object);
    planning_scene_diff_publisher.publish(planning_scene);
    sleep_short_time.sleep();

*/
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    moveit::planning_interface::MoveGroupInterface::Plan plan2;

    do
    {
      move_group.clearPoseTargets();
      move_group.setPoseTarget(pose1);
      success = move_group.plan(plan1);
    }
    while (!success);
    move_group.execute(plan1);

    if (is_multi == 'y')
    {
      ROS_INFO("planning time set as %f", time_multi);
      move_group.setPlanningTime(time_multi);
    }
    for (int i = 0; i < total_attempt; i++)
    {
      std::cout << "********" << (i + 1) << "th iteration********" << std::endl;
      move_group.clearPoseTargets();
      move_group.setPoseTarget(pose2);
      success = move_group.plan(plan2);

      if (success)
      {
        number_success[0] += 1;
        my_planning_time[0] += plan2.planning_time_;
        double temp_info[2] = {0.0, 0.0};
        trajInfo(&(plan2.trajectory_), robot_model, joint_model_group, kinematic_state, temp_info);
        my_length[0] += temp_info[0];
        my_smoothness[0] += temp_info[1];
      }
    }

    do 
    {
      move_group.clearPoseTargets();
      move_group.setPoseTarget(pose2);
      success = move_group.plan(plan2);
    }
    while (!success);
    move_group.execute(plan2);

    for (int i = 0; i < total_attempt; i++)
    {
      std::cout << "********" << (i + 1) << "th iteration********" << std::endl;

      move_group.clearPoseTargets();
      move_group.setPoseTarget(pose1);
      success = move_group.plan(plan1);

      if (success)
      {
        number_success[1] += 1;
        my_planning_time[1] += plan1.planning_time_;
        double temp_info[2] = {0.0, 0.0};
        trajInfo(&(plan1.trajectory_), robot_model, joint_model_group, kinematic_state, temp_info);
        my_length[1] += temp_info[0];
        my_smoothness[1] += temp_info[1];
      }

    }
 


/*
    for (int i = 0; i < total_attempt; i++)
    {
      kinematic_state->setVariablePositions(joint_values1);

      robot_state::robotStateToRobotStateMsg(*kinematic_state, planning_scene.robot_state);
      planning_scene_diff_publisher.publish(planning_scene);
      sleep_short_time.sleep();

      move_group.clearPoseTargets();
      move_group.setPoseTarget(pose2);
      success = move_group.plan(plan2);
      if (success)
      {
        number_success[0] += 1;
        my_planning_time[0] += plan2.planning_time_;
        double temp_info[2] = {0.0, 0.0};
        trajInfo(&(plan2.trajectory_), robot_model, joint_model_group, kinematic_state, temp_info);
        my_length[0] += temp_info[0];
        my_smoothness[0] += temp_info[1];
      }

ROS_INFO("********%dth iteration********", i);

      kinematic_state->setVariablePositions(joint_values2);

      robot_state::robotStateToRobotStateMsg(*kinematic_state, planning_scene.robot_state);
      planning_scene_diff_publisher.publish(planning_scene);
      sleep_short_time.sleep();

      move_group.clearPoseTargets(); 
      move_group.setPoseTarget(pose1);
      success = move_group.plan(plan1);
      if (success)
      {
        number_success[1] += 1;
        my_planning_time[1] += plan1.planning_time_;
        double temp_info[2] = {0.0, 0.0};
        trajInfo(&(plan1.trajectory_), robot_model, joint_model_group, kinematic_state, temp_info);
        my_length[1] += temp_info[0];
        my_smoothness[1] += temp_info[1];
      }

    }
*/  
    if (number_success[0] == 0 || number_success[1] == 0)
    {
      ROS_ERROR("number_success = 0");
      return (0);
    }
  
    double success_rate1 = number_success[0]/((double)total_attempt);
    my_planning_time[0] = my_planning_time[0]/number_success[0];
    my_length[0] = my_length[0]/number_success[0];
    my_smoothness[0] = my_smoothness[0]/number_success[0];
    double success_rate2 = number_success[1]/((double)total_attempt);
    my_planning_time[1] = my_planning_time[1]/number_success[1];
    my_length[1] = my_length[1]/number_success[1];
    my_smoothness[1] = my_smoothness[1]/number_success[1];

    ROS_INFO("From free to ball\nRate: %f\nTime: %f\nLength: %f\nSmoothness: %f", 
              success_rate1, my_planning_time[0], my_length[0], my_smoothness[0]);
    ROS_INFO("From ball to free\nRate: %f\nTime: %f\nLength: %f\nSmoothness: %f",
              success_rate2, my_planning_time[1], my_length[1], my_smoothness[1]); 
  }
  
  sleep_one_second.sleep(); //Sleep for a while is necessary 
  return(0);

}


