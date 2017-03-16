//Include Eigen geometry header for rotations
#include </usr/include/eigen3/Eigen/Geometry>

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