// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>
//Include Eigen geometry header for rotations
#include </usr/include/eigen3/Eigen/Geometry>
//Include visualization markers for RViz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// file reading
#include <iostream>
#include <fstream>

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

/**
* Writes a joint trajectory to a csv file
 */
  bool jointTrajectoryToCsvFile(trajectory_msgs::JointTrajectory input, std::string filepath);


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "descartes_tutorial");
    ros::NodeHandle nh;

    // Required for communication with moveit components
    ros::AsyncSpinner spinner (1);
    spinner.start();

    //***************************
    // put some code to test here
    // get local path to save scene results
    std::string local_path;
        
    if (nh.getParam("/local_path", local_path))
    {
        std::cout << local_path << std::endl;
    }
    else
    {
        std::cout << "Path not found" << std::endl;
    }

    trajectory_msgs::JointTrajectory dummy;
    bool succes = jointTrajectoryToCsvFile(dummy, local_path);
    if (succes)
    {
        ROS_INFO("Reading was ok");
    }
    //***************************


    // Wait till user kills the process (Control-C)

    ROS_INFO("Done!");
    return 0;
}

// functions
bool jointTrajectoryToCsvFile(trajectory_msgs::JointTrajectory input, std::string filepath)
{
  // If file exists, its content is deleted.
  std::string full_path = filepath + "trajectory.csv";
  std::ofstream file(full_path, std::ios::trunc);
  if (!file.is_open())
  {
    ROS_WARN("Output csv file could not be opened");
    return false;
  }
  else
  {
    // do stuff
    file << "Test file writing.";
    file.close();
    return true;
  }
}

