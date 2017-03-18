#include <descartes_tutorials/utilities.h>
#include <descartes_core/utils.h>
#include <geometric_shapes/shape_operations.h>

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

	void addEnvironment(moveit_msgs::PlanningScene& planningScene)
	{
		//Table (Tafel steekt 12mm boven oorsprong uit)
		Eigen::Vector3d tablescale(1.0,1.0,1.0);
		Eigen::Affine3d tablepose;
		tablepose = descartes_core::utils::toFrame(0.3, -0.6, 0.1, 0.0, 0.0, 0.0, descartes_core::utils::EulerConventions::XYZ);
		planningScene.world.collision_objects.push_back(makeCollisionObject("package://kuka_description/meshes/table_clamps/table/Table_scaled.stl", tablescale, "Table", tablepose));
		planningScene.object_colors.push_back(utilities::makeObjectColor("Table", 0.2, 0.2, 0.2, 1.0));
	}

	moveit_msgs::CollisionObject makeCollisionObject(std::string filepath, Eigen::Vector3d scale, std::string ID, Eigen::Affine3d pose)
	{
		moveit_msgs::CollisionObject co;

		//ROS_INFO("Loading mesh");
		shapes::Mesh* m = shapes::createMeshFromResource(filepath, scale);
		//ROS_INFO("Mesh loaded");

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

	std_msgs::ColorRGBA makeColor(double r, double g, double b, double a)
	{
		std_msgs::ColorRGBA color;
		color.r = r;
		color.g = g;
		color.b = b;
		color.a = a;
		return color;
	}

	moveit_msgs::ObjectColor makeObjectColor(std::string id, std_msgs::ColorRGBA color)
	{
		moveit_msgs::ObjectColor oc;
		oc.id = id;
		oc.color = color;
		return oc;
	}

	moveit_msgs::ObjectColor makeObjectColor(std::string id, double r, double g, double b, double a)
	{
		moveit_msgs::ObjectColor oc;
		oc.color = makeColor(r, g, b, a);
		oc.id = id;
		return oc;
	}

} //Namespace utilities

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
