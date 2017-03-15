#ifndef __TRAJVIS_H_INCLUDED__
#define __TRAJVIS_H_INCLUDED__

//Include visualization markers for RViz
#include <visualization_msgs/Marker.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
//Include Eigen geometry header for rotations
#include </usr/include/eigen3/Eigen/Geometry>
//include utils library_path
#include <descartes_tutorials/utilities.h>

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;

namespace trajvis
{
	enum ToleranceOption {
		CartesianPoint,
		TolerancedCartesianPoint,
		AxialSymmetricPoint
	};
	
	class visualizedTrajectory
	{
		private:
		TrajectoryVec trajvec;
		std::vector<visualization_msgs::Marker> markervec;
		public:
		visualizedTrajectory(){};
		
		TrajectoryVec getTrajectory() {
			return trajvec;
		}
		
		std::vector<visualization_msgs::Marker> getMarkers(){
			return markervec;
		}
		
		void addPoint(double transX, double transY, double transZ, double rotX, double rotY, double rotZ, ToleranceOption TO);
		
		void addPoint(Eigen::Affine3d pose, ToleranceOption TO);

		private:
		//Generates a toleranced cartesian point from a pose
		descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(	double transX, 
																		double transY, 
																		double transZ, 
																		double rotX, 
																		double rotY, 
																		double rotZ);
		
		//Generates a cartesian point with free rotation about the Z axis of the EFF frame
		descartes_core::TrajectoryPtPtr makeAxialSymmetricPoint(double x, double y, double z, double rx, double ry, double rz);
		
		//Function for easily defining poses
		Eigen::Affine3d definePose(double transX, double transY, double transZ, double rotX, double rotY, double rotZ);
		
		//Creates pose that can be added to the TrajectoryVec vector.
		descartes_core::TrajectoryPtPtr addPose(double transX, double transY, double transZ, double rotX, double rotY, double rotZ, 
												bool symmetric);
		
		//Define function for easy marker creation
		visualization_msgs::Marker createMarker(double transX, double transY, double transZ, double rotX, double rotY, double rotZ,
												descartes_trajectory::AxialSymmetricPt::FreeAxis axis);
		
	};
}

#endif
