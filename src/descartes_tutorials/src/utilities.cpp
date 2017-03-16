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
	std::vector<Eigen::Affine3d> straightLine(	Eigen::Affine3d startPose,
												Eigen::Affine3d endPose,
												int steps)
	{
		Eigen::Vector3d trans;
		trans = endPose.translation() - startPose.translation();
		trans = trans / steps;
		Eigen::Translation<double,3> translate(trans);

		std::vector<Eigen::Affine3d> poses;
		poses.push_back(startPose);
		for(int i = 0; i < (steps - 1); ++i)
		{
			poses.push_back(translate * poses.back());
		}
		return poses;
	}

	std::vector<Eigen::Affine3d> circle(Eigen::Affine3d centerPose,
										double radius,
										int steps,
										double angleY,
										double arc)
	{
		Eigen::Vector3d unitZ(0.0,0.0,1.0);
		unitZ = centerPose.rotation() * unitZ;
		unitZ = -1 * unitZ;
		unitZ = radius * unitZ;
		Eigen::Translation<double,3> trans(unitZ);

		Eigen::Affine3d startPose;
		startPose = centerPose;
		startPose = trans * startPose;

		double stepSize;
		stepSize = arc / steps;

		Eigen::Vector3d centerToEdge;
		centerToEdge = startPose.translation() - centerPose.translation();

		Eigen::Vector3d unitX(1.0,0.0,0.0);
		unitX = centerPose.rotation() * unitX;
		Eigen::AngleAxis<double> aa(stepSize, unitX);

		std::vector<Eigen::Affine3d> poses;
		Eigen::Affine3d pose;
		pose = centerPose.rotation();
		Eigen::Affine3d tempPose;
		for(int i = 0; i < steps; ++i)
		{
			centerToEdge = aa * centerToEdge;
			Eigen::Translation<double,3> c2e(centerToEdge);
			Eigen::Translation<double,3> centerTranslation(centerPose.translation());
			pose = aa * pose;
			Eigen::Vector3d unitY(0.0,1.0,0.0);
			unitY = pose * unitY;
			tempPose = Eigen::AngleAxisd(angleY, unitY) * pose;
			tempPose = c2e * centerTranslation * tempPose;
			poses.push_back(tempPose);
		}
		return poses;
	}
} //Namespace posegeneration