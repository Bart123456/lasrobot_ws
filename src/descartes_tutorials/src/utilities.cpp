#include <descartes_tutorials/utilities.h>

namespace utilities {
	//Function for constructing quaternion starting from Euler rotations XYZ
    Eigen::Quaternion<double> eulerToQuat(double rotX, double rotY, double rotZ)
	{
		Eigen::Matrix3d m;
		m = Eigen::AngleAxisd(rotX, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(rotY, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(rotZ, Eigen::Vector3d::UnitZ());
	
		Eigen::AngleAxis<double> aa;
		aa = Eigen::AngleAxisd(m);
		
		Eigen::Quaternion<double> quat;
		quat = Eigen::Quaternion<double>(aa);
		return quat;
	}
} //Namespace utils

namespace poseGeneration {
	std::vector<Eigen::Affine3d> straightLine(	Eigen::Affine3d startPose
												Eigen::Affine3d endPose
												int steps)
	{
		
	}
} //Namespace posegeneration