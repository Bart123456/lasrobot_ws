// Core ros functionality like ros::init and spin
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>

//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>

// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>

#include <math.h>

//Library for utilities
#include <descartes_tutorials/utilities.h>

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

//*********************
//**     Main        **
//*********************
int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;  
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup group("robotarm");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    geometry_msgs::Pose start_pose;
    start_pose = group.getCurrentPose().pose;
    ROS_INFO("x or of current pose: %f", start_pose.orientation.x);

    //Welding workobject
    Eigen::Vector3d objectscale(0.001,0.001,0.001);
    Eigen::Affine3d objectpose;

    double objectX, objectY, objectZ, objectrX, objectrY, objectrZ;
    std::string objectID;
    objectID = "tube_on_plate";
    objectX = 0.8;
    objectY = 0.0;
    objectZ = 0.112;
    objectrX = 0.0;
    objectrY = 0.0;
    objectrZ = 0.0;

    moveit_msgs::PlanningScene planning_scene;
    //Create collision objects
    utilities::addEnvironment(planning_scene);


    objectpose = descartes_core::utils::toFrame(objectX, objectY, objectZ, objectrX, objectrY, objectrZ, descartes_core::utils::EulerConventions::XYZ);
    planning_scene.world.collision_objects.push_back(utilities::makeCollisionObject("package://descartes_tutorials/Scenarios/Meshes/tube_on_plate.stl", objectscale, objectID, objectpose));

    //Planning scene colors
    planning_scene.object_colors.push_back(utilities::makeObjectColor(objectID, 0.5, 0.5, 0.5, 0.8));

    //Define publisher
    ros::Publisher planning_scene_diff_publisher;
    planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    planning_scene.is_diff = true;

    //Wait for subscribers
    ros::Rate loop_rate(10);
    ROS_INFO("Waiting for planning_scene subscriber.");
    if(waitForSubscribers(planning_scene_diff_publisher, ros::Duration(2.0)))
    {
    planning_scene_diff_publisher.publish(planning_scene);
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO("Object added to the world.");
    } else {
    ROS_ERROR("No subscribers connected, collision object not added");
    }

    // // planning cartesian path
    // std::vector<geometry_msgs::Pose> waypoints;

    // geometry_msgs::Pose start_pose2;
    // start_pose2.orientation.w = 1.0;
    // start_pose2.position.x = 0.55;
    // start_pose2.position.y = -0.05;
    // start_pose2.position.z = 0.8;

    // geometry_msgs::Pose target_pose3 = start_pose2;
    // target_pose3.position.x += 0.2;
    // target_pose3.position.z += 0.2;
    // waypoints.push_back(target_pose3);  // up and out

    // target_pose3.position.y -= 0.2;
    // waypoints.push_back(target_pose3);  // left

    // target_pose3.position.z -= 0.2;
    // target_pose3.position.y += 0.2;
    // target_pose3.position.x -= 0.2;
    // waypoints.push_back(target_pose3);  // down and right (back to start)

    // moveit_msgs::RobotTrajectory trajectory;
    // double fraction = group.computeCartesianPath(waypoints,
    //                                             0.01,  // eef_step
    //                                             0.0,   // jump_threshold
    //                                             trajectory);

    // ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
    //     fraction * 100.0);
    // /* Sleep to give Rviz time to visualize the plan. */
    // sleep(10.0);

    // bool test2 = executeTrajectory(trajectory.joint_trajectory);
    // ROS_INFO("Executing to_start trajectory %s",test2?"":"FAILED");

    // planning to start of circle weld
    // (no trajectory to go to weld in the right way...)
    geometry_msgs::Pose weld_start;
    weld_start = start_pose;
    // float alpha = - 5.0 * M_PI / 4.0;
    // weld_start.orientation.x = 0.0 * sin(alpha / 2.0);
    // weld_start.orientation.x = 1.0 * sin(alpha / 2.0);
    // weld_start.orientation.x = 0.0 * sin(alpha / 2.0);
    // weld_start.orientation.w = 1.0 * cos(alpha / 2.0);
    weld_start.position.x += 0.1;//0.8 - 0.054;
    weld_start.position.y += 0.0; //0.0;
    weld_start.position.z += -0.1; //0.112 + 0.014;
    group.setPoseTarget(weld_start);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    if (success) {
        trajectory_msgs::JointTrajectory to_start;
        to_start = my_plan.trajectory_.joint_trajectory;
        bool test = executeTrajectory(to_start);
        ROS_INFO("Executing to_start trajectory %s",test?"":"FAILED");
    }
    //group.move();
    //sleep(2.0);

    

    // circle path
    // ***********

    //std::vector<Eigen::Affine3d> poses;
    //Eigen::Affine3d centerPose;
    //centerPose = descartes_core::utils::toFrame(objectX, objectY, objectZ + 0.014, objectrX, -(M_PI / 2), objectrZ, descartes_core::utils::EulerConventions::XYZ);
    //poses = poseGeneration::circle(centerPose, 0.054, 10, -(M_PI / 4), 2 * M_PI);

    // back to home
    group.setNamedTarget("home_pose");
    success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    if (success) {
        trajectory_msgs::JointTrajectory to_home;
        to_home = my_plan.trajectory_.joint_trajectory;
        bool test3 = executeTrajectory(to_home);
        ROS_INFO("Executing to_home trajectory %s",test3?"":"FAILED");
    }

	ros::shutdown();  
	return 0;
}

//*********************
//**     FUNCTIONS   **
//*********************
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  ROS_INFO("Waiting for action server to start.");
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }
  ROS_INFO("Action server started.");

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);

  ac.sendGoal(goal);
  
  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
  
}

// get basic info, use pass by reference for group object
void getInfo(const moveit::planning_interface::MoveGroup &group)
{
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
}

geometry_msgs::Pose createPose(float w, float x, float y, float z)
{
	geometry_msgs::Pose pose;
	
	pose.orientation.w = w;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;
	
	return pose;
}