// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using; <moveit/planning_scene/planning_scene.h> already included
#include <descartes_moveit/moveit_state_adapter.h>

//Includes for collision objects
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>


// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>
//Include visualization markers for RViz
//#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//Custom library for trajectory visualization in Rviz
#include <descartes_tutorials/trajvis.h>
//Library for utilities
#include <descartes_tutorials/utilities.h>

#include <math.h>

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

//Creates a collision object from a mesh
moveit_msgs::CollisionObject makeCollisionObject(std::string filepath, Eigen::Vector3d scale, std::string ID, Eigen::Affine3d pose);


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "robot");
  ros::NodeHandle nh;

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();
  
  // 1. Define sequence of points
  TrajectoryVec points;
  std::vector<visualization_msgs::Marker> markerVec;

  //Used to store both cartesian waypoints and their visualization markers
  trajvis::visualizedTrajectory trajectory;

  //Create collision objects
  moveit_msgs::PlanningScene planning_scene;

  //Table (Tafel steekt 12mm boven oorsprong uit)
  Eigen::Vector3d tablescale(1.0,1.0,1.0);
  Eigen::Affine3d tablepose;
  tablepose = descartes_core::utils::toFrame(0.3, -0.6, 0.1, 0.0, 0.0, 0.0, descartes_core::utils::EulerConventions::XYZ);
  planning_scene.world.collision_objects.push_back(makeCollisionObject("package://kuka_description/meshes/table_clamps/table/Table_scaled.stl", tablescale, "Table", tablepose));
  
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
  

  objectpose = descartes_core::utils::toFrame(objectX, objectY, objectZ, objectrX, objectrY, objectrZ, descartes_core::utils::EulerConventions::XYZ);
  planning_scene.world.collision_objects.push_back(makeCollisionObject("package://descartes_tutorials/Scenarios/Meshes/tube_on_plate.stl", objectscale, objectID, objectpose));
  
  //Planning scene colors
  planning_scene.object_colors.resize(1);
  planning_scene.object_colors[0].color.r = 0.5;
  planning_scene.object_colors[0].color.g = 0.5;
  planning_scene.object_colors[0].color.b = 0.5;
  planning_scene.object_colors[0].color.a = 1.0;
  planning_scene.object_colors[0].id = objectID;


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

  std::vector<Eigen::Affine3d> poses;
  Eigen::Affine3d centerPose;
  centerPose = descartes_core::utils::toFrame(0.5, 0.1, 0.2, 1.0, 1.0, 0.0, descartes_core::utils::EulerConventions::XYZ);
  poses = poseGeneration::circle(centerPose, 0.1, 20, M_PI / 4, 2 * (M_PI / 3));

  int tempSize;
  tempSize = poses.size();

  for(int i = 0; i < tempSize; ++i)
  {
    trajectory.addPoint(poses[i], trajvis::AxialSymmetricPoint);
  }

  /*
  //straightline test
  std::vector<Eigen::Affine3d> poses;
  Eigen::Affine3d startPose;
  startPose = descartes_core::utils::toFrame(0.5, 0.1, 0.2, 0.0, 0.0, 0.0, descartes_core::utils::EulerConventions::XYZ);
  Eigen::Affine3d endPose;
  endPose = descartes_core::utils::toFrame(0.5, 1.0, 1.0, 0.0, 0.0, 0.0, descartes_core::utils::EulerConventions::XYZ);
  poses = poseGeneration::straightLine(startPose, endPose, 100);

  int tempSize;
  tempSize = poses.size();

  for(int i = 0; i < tempSize; ++i)
  {
    trajectory.addPoint(poses[i], trajvis::AxialSymmetricPoint);
  }
  */

  /*
  //Define points on circle
  double radius, height;
  radius = 0.052;
  height = objectY + 0.012;
  int steps;
  steps = 36;

  double stepSize;
  stepSize = (2*M_PI) / steps;

  Eigen::Matrix3d rot;
  Eigen::Vector3d trans_vec_1(objectX, objectY, objectZ + height);
  Eigen::Translation<double,3> trans1(trans_vec_1);
  Eigen::Vector3d trans_vec_2;
  Eigen::Translation<double,3> trans2(trans_vec_2);

  Eigen::Affine3d effectorPose;

  for(int i = 0; i < steps; ++i)
  {
    
		rot = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(-(M_PI/2), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(stepSize * i, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(-(M_PI/4), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

    trans_vec_2[0] = radius * cos(stepSize * i);
    trans_vec_2[1] = radius * sin(stepSize * i);
    trans_vec_2[2] = 0.0;

    Eigen::Translation<double,3> trans2(trans_vec_2);

    effectorPose = trans2 * trans1 * rot;

    trajectory.addPoint(effectorPose, trajvis::AxialSymmetricPoint);
    
  }
  */

  
  /*
  for (unsigned int i = 0; i < 10; ++i)
  {
    trajectory.addPoint(0.8, 0.3, 0.6 + i * 0.05, 0, M_PI / 2, 0, trajvis::AxialSymmetricPoint);
  }
  
  for (unsigned int i = 0; i < 10; ++i)
  {
    trajectory.addPoint(0.8, 0.3 + i * 0.05, 1.1, 0, M_PI / 2, 0, trajvis::AxialSymmetricPoint);
  }
  
  for (unsigned int i = 0; i < 10; ++i)
  {
    trajectory.addPoint(0.8, 0.8, 1.1, 3 * (M_PI / 2), (M_PI / 2) - i * 0.2, 0, trajvis::AxialSymmetricPoint);
  }
  */
  
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
  descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);
  
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

  // 4. Feed the trajectory to the planner
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -2;
  }

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
  trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 1.0);

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

moveit_msgs::CollisionObject makeCollisionObject(std::string filepath, Eigen::Vector3d scale, std::string ID, Eigen::Affine3d pose)
{
  moveit_msgs::CollisionObject co;

  ROS_INFO("Loading mesh");
  shapes::Mesh* m = shapes::createMeshFromResource(filepath, scale);
  ROS_INFO("Mesh loaded");

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  Eigen::Vector3d translations;
  translations = pose.translation();
  Eigen::Vector3d rotationsXYZ;
  rotationsXYZ = pose.rotation().eulerAngles(0,1,2);
  Eigen::Quaternion<double> quat;
  quat = utilities::eulerToQuat(rotationsXYZ[0], rotationsXYZ[1], rotationsXYZ[2]);

  co.header.frame_id = "base_link";
  co.id = ID;
  co.meshes.resize(1);
  co.mesh_poses.resize(1);
  co.meshes[0] = mesh;
  co.mesh_poses[0].position.x = translations[0];
  co.mesh_poses[0].position.y = translations[1];
  co.mesh_poses[0].position.z = translations[2];
  co.mesh_poses[0].orientation.w= quat.w();
  co.mesh_poses[0].orientation.x= quat.x();
  co.mesh_poses[0].orientation.y= quat.y();
  co.mesh_poses[0].orientation.z= quat.z();

  co.operation = co.ADD;

  return co;
}


