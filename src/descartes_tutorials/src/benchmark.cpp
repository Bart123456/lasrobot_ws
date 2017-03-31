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
#include <descartes_core/utils.h>
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

    // planning to start of circle weld
    // I put the path heigher above the ground plane with offset because
    // move group does not find a plan otherwise...
    double offset = 0.5;
    geometry_msgs::Pose weld_start;
    weld_start = start_pose;
    weld_start.position.x = objectX;
    weld_start.position.y = objectY;
    weld_start.position.z = objectZ + offset;
    group.setPoseTarget(weld_start);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(2.0);

    // execute plan with action server
    if (success) {
        trajectory_msgs::JointTrajectory to_start;
        to_start = my_plan.trajectory_.joint_trajectory;
        bool test = executeTrajectory(to_start);
        ROS_INFO("Executing to_start trajectory %s",test?"":"FAILED");
    }

    // create circular welding path
    std::vector<Eigen::Affine3d> poses;
    Eigen::Affine3d centerPose;
    centerPose = descartes_core::utils::toFrame(objectX, objectY, objectZ + offset, objectrX, -(M_PI / 2), objectrZ, descartes_core::utils::EulerConventions::XYZ);
    poses = poseGeneration::circle(centerPose, 0.054, 10, -(M_PI / 4), 2 * M_PI);

    int npoints = poses.size();
    Eigen::Affine3d temp_pose;
    geometry_msgs::Pose target_pose;
    std::vector<geometry_msgs::Pose> waypoints2;

    for (int i = 0; i < npoints; ++i) {
        // get current pose
        temp_pose = poses[i];

        // get translation data
        target_pose.position.x = temp_pose.translation()[0];
        target_pose.position.y = temp_pose.translation()[1];
        target_pose.position.z = temp_pose.translation()[2];

        // get orientation data
        Eigen::Quaterniond temp_q(temp_pose.rotation());

        // I would expect the w coeff in the eigen quaternion to be ad index 0
        // but it seems that this order is right
        target_pose.orientation.x = temp_q.coeffs().data()[0];
        target_pose.orientation.y = temp_q.coeffs().data()[1];
        target_pose.orientation.z = temp_q.coeffs().data()[2];
        target_pose.orientation.w = temp_q.coeffs().data()[3];

        // add to waypoints vector
        waypoints2.push_back(target_pose);
    }

    ROS_INFO("z position of circle: %f", waypoints2[0].position.z);

    // plan how to execute circular welding path
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints2,
                                                0.01,  // eef_step
                                                0.0,   // jump_threshold
                                                trajectory);

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    // execute plan with action server
    bool test2 = executeTrajectory(trajectory.joint_trajectory);
    ROS_INFO("Executing to_start trajectory %s",test2?"":"FAILED");

    // back to home
    group.setNamedTarget("home_pose");
    success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(2.0);

    // execute plan with action server
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