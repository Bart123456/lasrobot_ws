//Include Eigen geometry header for rotations
#include </usr/include/eigen3/Eigen/Geometry>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/ColorRGBA.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>

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

	//Generates a toleranced cartesian point from a pose
	descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(	double transX, 
																	double transY, 
																	double transZ, 
																	double rotX, 
																	double rotY, 
																	double rotZ,
																	double rotStepSize);

	descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(Eigen::Affine3d pose, double rxTolerance, double ryTolerance, double rzTolerance, double rotStepSize);
	
	//Generates a cartesian point with free rotation about the Z axis of the EFF frame
	descartes_core::TrajectoryPtPtr makeAxialSymmetricPoint(double x, double y, double z, double rx, double ry, double rz, double rotStepSize);
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