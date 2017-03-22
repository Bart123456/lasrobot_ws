//Include Eigen geometry header for rotations

#ifndef MOVE_GROUP_H
#define MOVE_GROUP_H
#include </usr/include/eigen3/Eigen/Geometry>
#include <moveit/move_group_interface/move_group.h>
#include <descartes_core/utils.h>
#include <geometric_shapes/shape_operations.h>



namespace utilities {
    //Function for constructing quaternion starting from Euler rotations XYZ
    Eigen::Quaternion<double> eulerToQuat(double rotX, double rotY, double rotZ);
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

namespace collisionObjectScene
{

	class workObject
	{
		private:
		std::vector<Eigen::Affine3d> poses;
		moveit_msgs::CollisionObject co;
		static int ID_number;
		public: 
		// Constructor of the class
		workObject();


		// Make a tube_on_plate
		void makeTubeOnPlate( double objectX, double objectY, double objectZ, double objectrX, double objectrY, double objectrZ,
							double radius, 
							int steps, 
							double angleY, 
							double arc);

		// getter function for the collion object of the workobject
			// getter 
		moveit_msgs::CollisionObject getCollisionObject()
		{
			return co;
		}
		//TODO: make functions for all the other objects
	};



	moveit_msgs::CollisionObject makeCollisionObject(std::string filepath, Eigen::Vector3d scale, std::string ID, Eigen::Affine3d pose);
}

#endif 