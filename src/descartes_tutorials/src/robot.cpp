// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using; <moveit/planning_scene/planning_scene.h> already included
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>

//Includes for collision objects
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>


// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>
//Include visualization markers for RViz
//#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//Convert affine3d poses to pose messages
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
//Custom library for trajectory visualization in Rviz
#include <descartes_tutorials/trajvis.h>
//Library for utilities
#include <descartes_tutorials/utilities.h>

#include <math.h>

//For saving msgs to file.
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;


/**
 * Translates a descartes trajectory to a ROS joint trajectory
 */
trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory, const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names, double time_delay);

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

//Waits for a subscriber to subscribe to a publisher
//Used to wait for RViz to subscribe to the Marker topic before publishing them
bool waitForSubscribers(ros::Publisher & pub, ros::Duration timeout);

bool readTrajectoryFile = false;
bool writeTrajectoryFile = true;
//std::string bagReadFilePath = "/home/bart/lasrobot_ws/src/descartes_tutorials/Scenarios/trajectories/trajectory.bag";
//std::string bagWriteFilePath = "/home/bart/lasrobot_ws/src/descartes_tutorials/Scenarios/trajectories/trajectory.bag";

//****************************
//**     main loop          **
//****************************
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "robot");
  ros::NodeHandle nh;

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();

  trajectory_msgs::JointTrajectory readTrajectory;
  trajectory_msgs::JointTrajectory joint_solution;

  geometry_msgs::PoseArray trajPoses;
  geometry_msgs::PoseArray robotPoses;
  
  // get local path to save scene results
  std::string local_path;
    
  if (nh.getParam("/local_path", local_path))
  {
    ROS_INFO_STREAM("Local path found: " << local_path);
  }
  else
  {
    ROS_ERROR_STREAM("Path not found");
  }

  std::string bagReadFilePath = local_path;
  std::string bagWriteFilePath = local_path;
  
  // 1. Define sequence of points
  TrajectoryVec points;
  std::vector<visualization_msgs::Marker> markerVec;

  //Used to store both cartesian waypoints and their visualization markers
  trajvis::visualizedTrajectory trajectory;

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
  if(!readTrajectoryFile)
  {
    //Create collision objects
    utilities::addEnvironment(planning_scene);


    objectpose = descartes_core::utils::toFrame(objectX, objectY, objectZ, objectrX, objectrY, objectrZ, descartes_core::utils::EulerConventions::XYZ);
    planning_scene.world.collision_objects.push_back(utilities::makeCollisionObject("package://descartes_tutorials/Scenarios/Meshes/tube_on_plate.stl", objectscale, objectID, objectpose));
    
    //Planning scene colors
    planning_scene.object_colors.push_back(utilities::makeObjectColor(objectID, 0.5, 0.5, 0.5, 0.8));
  } else {
    rosbag::Bag bag2(bagReadFilePath, rosbag::bagmode::Read);
    rosbag::View view(bag2, rosbag::TopicQuery("planningscene"));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        moveit_msgs::PlanningScene::ConstPtr i = m.instantiate<moveit_msgs::PlanningScene>();
        if (m.getTopic() == "planningscene")
        {
          planning_scene = *i;
          ROS_INFO("Scene .bag file read.");
        }
    }
    bag2.close();
    ROS_INFO("Planning scene read");
  }

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

  //Define Poses
  std::vector<Eigen::Affine3d> poses;
  Eigen::Affine3d centerPose;
  centerPose = descartes_core::utils::toFrame(objectX, objectY, objectZ + 0.014, objectrX, -(M_PI / 2), objectrZ, descartes_core::utils::EulerConventions::XYZ);
  poses = poseGeneration::circle(centerPose, 0.054, 30, -(M_PI / 4), 2 * M_PI);

  int tempSize;
  tempSize = poses.size();

  //Copy poses into poseArray message
  trajPoses.poses.resize(tempSize);
  trajPoses.header.stamp = ros::Time::now();
  trajPoses.header.frame_id = "Ideal trajectory poses";
  geometry_msgs::Pose tempPose;

  for(int j = 0; j < tempSize; ++j)
  {
    tf::poseEigenToMsg(poses[j], tempPose);
    trajPoses.poses[j] = tempPose;
  }

  //Define tolerance sizes
  trajectory.setRotStepSize(M_PI/12);
  double rxTolerance, ryTolerance, rzTolerance;
  rxTolerance = M_PI/4;
  ryTolerance = M_PI/4;
  rzTolerance = 2*M_PI;

  for(int i = 0; i < tempSize; ++i)
  {
    //trajectory.addPoint(poses[i], trajvis::AxialSymmetricPoint);
    trajectory.addTolerancedPoint(poses[i], rxTolerance, ryTolerance, rzTolerance);
  }
  
  //Get both the trajectory and the markers
  markerVec = trajectory.getMarkers();
  points = trajectory.getTrajectory();

  
  //Copy vector to array so we can publish it as a MarkerArray type.
  int size = markerVec.size();
  visualization_msgs::MarkerArray ma;
  ma.markers.resize(size);
  for(int i = 0;i < size;i++)
  {
    ma.markers[i] = markerVec[i];
  }
  
  //Start the publisher for the Rviz Markers
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 1 );
  //Wait for subscriber and publish the markerArray once the subscriber is found.
  
  ROS_INFO("Waiting for marker subscribers.");
  if(waitForSubscribers(vis_pub, ros::Duration(2.0)))
  {
    ROS_INFO("Subscriber found, publishing markers.");
    vis_pub.publish(ma);
    ros::spinOnce();
    loop_rate.sleep();
  } else {
    ROS_ERROR("No subscribers connected, markers not published");
  }

  // 2. Create a robot model and initialize it
  descartes_core::RobotModelPtr model (new descartes_moveit::IkFastMoveitStateAdapter);
  
  //Enable collision checking
  model->setCheckCollisions(true);

  // Name of description on parameter server. Typically just "robot_description".
  const std::string robot_description = "robot_description";

  // name of the kinematic group you defined when running MoveitSetupAssistant
  const std::string group_name = "robotarm";

  // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
  const std::string world_frame = "base_link";

  // tool center point frame (name of link associated with tool)
  const std::string tcp_frame = "endpoint";

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }

  //Update the internal planning scene, so that the collision objects will be added
  //model->updateInternals();

  // 3. Create a planner and initialize it with our robot model
  descartes_planner::DensePlanner planner;
  planner.initialize(model);

  //Use the extra weldingscost or not
  planner.getPlanningGraph().setUseWeldingCost(true);
  //Set the weldingcost weight:
  planner.getPlanningGraph().setWeldingCostFactor(1.0);

  //Don't plan path if it is read from file.
  if(!readTrajectoryFile)
  {
    // 4. Feed the trajectory to the planner
    double planningStart = ros::Time::now().toSec();
    if (!planner.planPath(points))
    {
      ROS_ERROR("Could not solve for a valid path");
      return -2;
    }

    double planningEnd = ros::Time::now().toSec();
    double planningTime = planningEnd - planningStart;
    ROS_INFO_STREAM("Passed planning time: " << planningTime << " seconds.");

    TrajectoryVec result;
    if (!planner.getPath(result))
    {
      ROS_ERROR("Could not retrieve path");
      return -3;
    }

    // 5. Translate the result into a type that ROS understands
    // Get Joint Names
    std::vector<std::string> names;
    
    nh.getParam("controller_joint_names", names);
    // Generate a ROS joint trajectory with the result path, robot model, given joint names,
    // a certain time delta between each trajectory point
    joint_solution = toROSJointTrajectory(result, *model, names, 1.0);

    //Translate joint solutions to poses using FK and save them in robotPoses
    Eigen::Affine3d eigenPose;
    geometry_msgs::Pose tempPose;
    robotPoses.poses.resize(joint_solution.points.size());
    robotPoses.header.stamp = ros::Time::now();
    robotPoses.header.frame_id = "Trajectory poses reached by robot";

    for(int k = 0; k < joint_solution.points.size(); ++k)
    {
      model->getFK(joint_solution.points[k].positions, eigenPose);
      tf::poseEigenToMsg(eigenPose, tempPose);
      robotPoses.poses[k] = tempPose;
    }

  } //END OF IF

  if(readTrajectoryFile)
  {
    rosbag::Bag bag(bagReadFilePath, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("trajectory"));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        trajectory_msgs::JointTrajectory::ConstPtr i = m.instantiate<trajectory_msgs::JointTrajectory>();
        if (m.getTopic() == "trajectory")
        {
          readTrajectory = *i;
          ROS_INFO("Trajectory .bag file read.");
        }
    }
    bag.close();
    joint_solution = readTrajectory;
    joint_solution.header.stamp = ros::Time::now();
    ROS_INFO("joint_solution overwritten");
  }

  if(writeTrajectoryFile && !readTrajectoryFile)
  {
    rosbag::Bag bag1;
    bag1.open(bagWriteFilePath, rosbag::bagmode::Write);
    bag1.write("trajectory", ros::Time::now(), joint_solution);
    bag1.write("planningscene", ros::Time::now(), planning_scene);
    bag1.write("trajectoryPoses", ros::Time::now(), trajPoses);
    bag1.write("robotPoses", ros::Time::now(), robotPoses);
    bag1.close();
    ROS_INFO("Trajectory and scene written to .bag file.");
  }
  

  // 6. Send the ROS trajectory to the robot for execution
	while(ros::ok()) {
		if (!executeTrajectory(joint_solution))
		{
		  ROS_ERROR("Could not execute trajectory!");
		  return -4;
		}
	}

  // Wait till user kills the process (Control-C)
  
  ROS_INFO("Done!");
  return 0;
} //main

trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay)
{
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = "world_frame";
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.points.push_back(pt);
  }

  return result;
}

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

//Waits for a subscriber to subscribe to a publisher
bool waitForSubscribers(ros::Publisher & pub, ros::Duration timeout)
{
    if(pub.getNumSubscribers() > 0)
        return true;
    ros::Time start = ros::Time::now();
    ros::Rate waitTime(0.5);
    while(ros::Time::now() - start < timeout) {
        waitTime.sleep();
        if(pub.getNumSubscribers() > 0)
            break;
    }
    return pub.getNumSubscribers() > 0;
}


