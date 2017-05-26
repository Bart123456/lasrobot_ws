#include <descartes_tutorials/trajvis.h>

namespace trajvis{
	
	descartes_core::TrajectoryPtPtr visualizedTrajectory::makeTolerancedCartesianPoint(	double transX, 
																						double transY, 
																						double transZ, 
																						double rotX, 
																						double rotY, 
																						double rotZ)
	{
		using namespace descartes_core;
		using namespace descartes_trajectory;
		Eigen::Affine3d pose = utils::toFrame(transX, transY, transZ, rotX, rotY, rotZ, utils::EulerConventions::XYZ);
	
		//Works
		descartes_trajectory::PositionTolerance p;
		p = ToleranceBase::zeroTolerance<PositionTolerance>(transX, transY, transZ);
		descartes_trajectory::OrientationTolerance o;
		o = ToleranceBase::createSymmetric<OrientationTolerance>(rotX, rotY, rotZ, 0, 0, 2*M_PI);
		return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose, p, o), 0.0, rotStepSize) );
	}

	descartes_core::TrajectoryPtPtr visualizedTrajectory::makeTolerancedCartesianPoint(	Eigen::Affine3d pose,
																						double rxTolerance, double ryTolerance, double rzTolerance)
	{
		using namespace descartes_core;
		using namespace descartes_trajectory;

		Eigen::Vector3d translations;
		translations = pose.translation();
		Eigen::Vector3d eulerXYZ;
		eulerXYZ = pose.rotation().eulerAngles(0,1,2);

		descartes_trajectory::PositionTolerance p;
		p = ToleranceBase::zeroTolerance<PositionTolerance>(translations(0), translations(1), translations(2));
		descartes_trajectory::OrientationTolerance o;
		o = ToleranceBase::createSymmetric<OrientationTolerance>(eulerXYZ(0), eulerXYZ(1), eulerXYZ(2), rxTolerance, ryTolerance, rzTolerance);
		return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose, p, o), 0.0, rotStepSize) );
	}

	descartes_core::TrajectoryPtPtr visualizedTrajectory::makeAxialSymmetricPoint(	double x, double y, double z, double rx, double ry,
																					double rz)
	{
		using namespace descartes_core;
		using namespace descartes_trajectory;
		return TrajectoryPtPtr( new AxialSymmetricPt(x, y, z, rx, ry, rz, rotStepSize, AxialSymmetricPt::Z_AXIS) );
	}
	
	Eigen::Affine3d visualizedTrajectory::definePose(double transX, double transY, double transZ, double rotX, double rotY, double rotZ)
	{
		Eigen::Matrix3d m;
		m = Eigen::AngleAxisd(rotX, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(rotY, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(rotZ, Eigen::Vector3d::UnitZ());
	
		Eigen::Affine3d pose;
		pose = Eigen::Translation3d(transX, transY, transZ);
		pose.linear() = m;
	
		return pose;
	}
	
	descartes_core::TrajectoryPtPtr visualizedTrajectory::addPose(	double transX, double transY, double transZ, double rotX, double rotY,
																	double rotZ, bool symmetric = true)
	{
		//Define the pose
		Eigen::Affine3d pose;
	
		descartes_core::TrajectoryPtPtr pt;
		if(symmetric){
			//Convert to axialsymmetric point
			pt = makeAxialSymmetricPoint(transX, transY, transZ, rotX, rotY, rotZ); 
		} else {
			pt = makeTolerancedCartesianPoint(transX, transY, transZ, rotX, rotY, rotZ);
		}
		return pt;
	}
	
	visualization_msgs::Marker visualizedTrajectory::createMarker(	double transX, double transY, double transZ, double rotX, double rotY, 																		double rotZ, descartes_trajectory::AxialSymmetricPt::FreeAxis axis =
																	descartes_trajectory::AxialSymmetricPt::X_AXIS)
	{
		static int count;
		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = count;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration(0);
	
		marker.pose.position.x = transX;
		marker.pose.position.y = transY;
		marker.pose.position.z = transZ;
	
		//To calculate the quaternion values we first define an AngleAxis object using Euler rotations, then convert it
		Eigen::Quaternion<double> quat;
		if(axis == descartes_trajectory::AxialSymmetricPt::X_AXIS)
		{
			quat = utilities::eulerToQuat(rotX, rotY, rotZ);
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
		} else if(axis == descartes_trajectory::AxialSymmetricPt::Y_AXIS)
		{
			quat = utilities::eulerToQuat(rotX, rotY, rotZ) * utilities::eulerToQuat(0, 0, M_PI / 2);
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		} else if(axis == descartes_trajectory::AxialSymmetricPt::Z_AXIS)
		{
			quat = utilities::eulerToQuat(rotX, rotY, rotZ) * utilities::eulerToQuat(0, 3 * (M_PI / 2), 0);
			marker.color.r = 0.0;
			marker.color.g = 0.0;
			marker.color.b = 1.0;
		}
	
		marker.pose.orientation.x = quat.x();
		marker.pose.orientation.y = quat.y();
		marker.pose.orientation.z = quat.z();
		marker.pose.orientation.w = quat.w();
		marker.scale.x = 0.02;
		marker.scale.y = 0.003;
		marker.scale.z = 0.003;
		marker.color.a = 1.0;	//Alpha

		count++;
		return marker;
	}

	void visualizedTrajectory::createVisualFrame(Eigen::Affine3d pose)
	{
		Eigen::Vector3d translations;
		translations = pose.translation();
		Eigen::Vector3d rotationsXYZ;
		rotationsXYZ = pose.rotation().eulerAngles(0,1,2);

		markervec.push_back(createMarker(	translations[0], translations[1], translations[2], 
												rotationsXYZ[0], rotationsXYZ[1], rotationsXYZ[2], descartes_trajectory::AxialSymmetricPt::X_AXIS));
		markervec.push_back(createMarker(	translations[0], translations[1], translations[2], 
												rotationsXYZ[0], rotationsXYZ[1], rotationsXYZ[2], descartes_trajectory::AxialSymmetricPt::Y_AXIS));
		markervec.push_back(createMarker(	translations[0], translations[1], translations[2], 
												rotationsXYZ[0], rotationsXYZ[1], rotationsXYZ[2], descartes_trajectory::AxialSymmetricPt::Z_AXIS));
	}
	
	void visualizedTrajectory::addPoint(double transX, double transY, double transZ, double rotX, double rotY, double rotZ,
										ToleranceOption TO)
	{
		if(TO == TolerancedCartesianPoint) {
			trajvec.push_back(addPose(transX, transY, transZ, rotX, rotY, rotZ, false));
		} else if(TO == AxialSymmetricPoint) {
			trajvec.push_back(addPose(transX, transY, transZ, rotX, rotY, rotZ, true));
		}
		markervec.push_back(createMarker(transX, transY, transZ, rotX, rotY, rotZ, descartes_trajectory::AxialSymmetricPt::X_AXIS));
		markervec.push_back(createMarker(transX, transY, transZ, rotX, rotY, rotZ, descartes_trajectory::AxialSymmetricPt::Y_AXIS));
		markervec.push_back(createMarker(transX, transY, transZ, rotX, rotY, rotZ, descartes_trajectory::AxialSymmetricPt::Z_AXIS));
	}

	void visualizedTrajectory::addPoint(Eigen::Affine3d pose, ToleranceOption TO)
	{
		Eigen::Vector3d translations;
		translations = pose.translation();
		Eigen::Vector3d rotationsXYZ;
		rotationsXYZ = pose.rotation().eulerAngles(0,1,2);
		
		if(TO == TolerancedCartesianPoint) {
			trajvec.push_back(addPose(	translations[0], translations[1], translations[2], 
										rotationsXYZ[0], rotationsXYZ[1], rotationsXYZ[2], false));	
		} else if(TO == AxialSymmetricPoint) {
			trajvec.push_back(addPose(	translations[0], translations[1], translations[2], 
										rotationsXYZ[0], rotationsXYZ[1], rotationsXYZ[2], true));
		}
		createVisualFrame(pose);
	}

	void visualizedTrajectory::addTolerancedPoint(Eigen::Affine3d pose, double rx, double ry, double rz)
	{
		trajvec.push_back(makeTolerancedCartesianPoint(pose, rx, ry, rz));
		createVisualFrame(pose);
	}
}
