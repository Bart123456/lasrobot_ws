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
}

namespace poseGeneration {
	std::vector<Eigen::Affine3d> straightLine(	Eigen::Affine3d startPose,
												Eigen::Affine3d endPose,
												int steps);
	
	std::vector<Eigen::Affine3d> circle(Eigen::Affine3d centerPose,
										double radius,
										int steps,
										double angleY,
										double arc);
} //Namespace posegeneration