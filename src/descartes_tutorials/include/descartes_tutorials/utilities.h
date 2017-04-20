//Include Eigen geometry header for rotations
#include </usr/include/eigen3/Eigen/Geometry>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/ColorRGBA.h>

namespace utilities {
    //Function for constructing quaternion starting from Euler rotations XYZ
    Eigen::Quaternion<double> eulerToQuat(double rotX, double rotY, double rotZ);

	//Adds multiple environment objects to the planning scene
	//that we will need to do collision checking with in every simulation.
	void addEnvironment(moveit_msgs::PlanningScene& planningScene);

	//Creates a collision object from a mesh
	moveit_msgs::CollisionObject 
	makeCollisionObject(std::string filepath, Eigen::Vector3d scale, 
						std::string ID, Eigen::Affine3d pose);

	std_msgs::ColorRGBA makeColor(double r, double g, double b, double a);

	moveit_msgs::ObjectColor makeObjectColor(std::string id, std_msgs::ColorRGBA color);

	moveit_msgs::ObjectColor makeObjectColor(std::string id, double r, double g, double b, double a);

	void computeAngleErrors(Eigen::Affine3d referencePose, Eigen::Affine3d pose, std::vector<double>& xErrors, std::vector<double>& yErrors);
}

namespace poseGeneration {
	/**
	* Get a vector of poses on a straight line between two points.
	* @param startPose Transformation matrix of the starting pose
	* @param endPose Transformation matrix of the end pose
	* @param steps Number of trajectory points on straight line
	* @return Vector containing the points of the line trajectory
	*/
	std::vector<Eigen::Affine3d> straightLine(	Eigen::Affine3d startPose,
												Eigen::Affine3d endPose,
												int steps);
	
	std::vector<Eigen::Affine3d> circle(Eigen::Affine3d centerPose,
										double radius,
										int steps,
										double angleY,
										double arc);
} //Namespace posegeneration