/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Dan Solomon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * cart_trajectory_pt.cpp
 *
 *  Created on: Oct 3, 2014
 *      Author: Dan Solomon
 */

#include <tuple>
#include <map>
#include <algorithm>
#include <console_bridge/console.h>
#include <ros/console.h>
#include <boost/uuid/uuid_io.hpp>
#include "descartes_trajectory/cart_trajectory_pt.h"
#include <descartes_core/utils.h>
#include <cmath>

#define NOT_IMPLEMENTED_ERR(ret)                                                                                       \
  logError("%s not implemented", __PRETTY_FUNCTION__);                                                                 \
  return ret;

const double EQUALITY_TOLERANCE = 0.0001f;

using namespace descartes_core;

namespace descartes_trajectory
{
EigenSTL::vector_Affine3d uniform(const TolerancedFrame &frame, const double orient_increment,
                                  const double pos_increment)
{
  EigenSTL::vector_Affine3d rtn;

  if (pos_increment < 0.0 || orient_increment < 0.0)
  {
    ROS_WARN_STREAM("Negative position/orientation increment: " << pos_increment << "/" << orient_increment);
    rtn.clear();
    return rtn;
  }

  Eigen::Affine3d sampled_frame;

  // Calculating the number of samples for each tolerance (position and orientation)
  size_t ntx, nty, ntz, nrx, nry, nrz;

  if (orient_increment > 0)
  {
    nrx = ((frame.orientation_tolerance.x_rel_upper - frame.orientation_tolerance.x_rel_lower) / orient_increment) + 1;
    nry = ((frame.orientation_tolerance.y_rel_upper - frame.orientation_tolerance.y_rel_lower) / orient_increment) + 1;
    nrz = ((frame.orientation_tolerance.z_rel_upper - frame.orientation_tolerance.z_rel_lower) / orient_increment) + 1;
  }
  else
  {
    nrx = nry = nrz = 1;
  }

  if (pos_increment > 0)
  {
    ntx = ((frame.position_tolerance.x_upper - frame.position_tolerance.x_lower) / pos_increment) + 1;
    nty = ((frame.position_tolerance.y_upper - frame.position_tolerance.y_lower) / pos_increment) + 1;
    ntz = ((frame.position_tolerance.z_upper - frame.position_tolerance.z_lower) / pos_increment) + 1;
  }
  else
  {
    ntx = nty = ntz = 1;
  }

  // Estimating the number of samples base on tolerance zones and sampling increment.
  size_t est_num_samples = ntx * nty * ntz * nrx * nry * nrz;

  ROS_DEBUG_STREAM("Estimated number of samples: " << est_num_samples << ", reserving space");
  rtn.reserve(est_num_samples);

  // TODO: The following for loops do not ensure that the rull range is sample (lower to upper)
  // since there could be round off error in the incrementing of samples.  As a result, the
  // exact upper bound may not be sampled.  Since this isn't a final implementation, this will
  // be ignored.
  double rx, ry, rz, tx, ty, tz;

  for (size_t ii = 0; ii < nrx; ++ii)
  {
    rx = frame.orientation_tolerance.x_rel_lower + orient_increment * ii;
    for (size_t jj = 0; jj < nry; ++jj)
    {
      ry = frame.orientation_tolerance.y_rel_lower + orient_increment * jj;
      for (size_t kk = 0; kk < nrz; ++kk)
      {
        rz = frame.orientation_tolerance.z_rel_lower + orient_increment * kk;
        for (size_t ll = 0; ll < ntx; ++ll)
        {
          tx = frame.position_tolerance.x_rel_lower + pos_increment * ll;
          for (size_t mm = 0; mm < nty; ++mm)
          {
            ty = frame.position_tolerance.y_rel_lower + pos_increment * mm;
            for (size_t nn = 0; nn < ntz; ++nn)
            {
              tz = frame.position_tolerance.z_rel_lower + pos_increment * nn;

              /*              sampled_frame = Eigen::Translation3d(tx,ty,tz) *
                                Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());*/
              sampled_frame =
                  descartes_core::utils::toLocalFrame(frame.frame, tx, ty, tz, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ);
              sampled_frame = frame.frame * sampled_frame;
              rtn.push_back(sampled_frame);
            }
          }
        }
      }
    }
  }
  ROS_DEBUG_STREAM("Uniform sampling of frame, utilizing orientation increment: "
                   << orient_increment << ", and position increment: " << pos_increment << " resulted in " << rtn.size()
                   << " samples");
  return rtn;
}

double distance(const std::vector<double> &j1, const std::vector<double> &j2)
{
  double rt = 0;
  double d;
  if (j1.size() == j2.size())
  {
    for (int i = 0; i < j1.size(); i++)
    {
      d = j1[i] - j2[i];
      rt += d * d;
    }
  }
  else
  {
    ROS_WARN_STREAM("Unequal size vectors, returning negative distance");
    return -1;
  }

  return std::sqrt(rt);
}

CartTrajectoryPt::CartTrajectoryPt(const descartes_core::TimingConstraint &timing)
  : descartes_core::TrajectoryPt(timing)
  , tool_base_(Eigen::Affine3d::Identity())
  , tool_pt_(Eigen::Affine3d::Identity())
  , wobj_base_(Eigen::Affine3d::Identity())
  , wobj_pt_(Eigen::Affine3d::Identity())
  , pos_increment_(0.0)
  , orient_increment_(0.0)
{
}

CartTrajectoryPt::CartTrajectoryPt(const Frame &wobj_base, const TolerancedFrame &wobj_pt, const Frame &tool,
                                   const TolerancedFrame &tool_pt, double pos_increment, double orient_increment,
                                   const descartes_core::TimingConstraint &timing)
  : descartes_core::TrajectoryPt(timing)
  , tool_base_(tool)
  , tool_pt_(tool_pt)
  , wobj_base_(wobj_base)
  , wobj_pt_(wobj_pt)
  , pos_increment_(pos_increment)
  , orient_increment_(orient_increment)
{
}

CartTrajectoryPt::CartTrajectoryPt(const TolerancedFrame &wobj_pt, double pos_increment, double orient_increment,
                                   const descartes_core::TimingConstraint &timing)
  : descartes_core::TrajectoryPt(timing)
  , tool_base_(Eigen::Affine3d::Identity())
  , tool_pt_(Eigen::Affine3d::Identity())
  , wobj_base_(Eigen::Affine3d::Identity())
  , wobj_pt_(wobj_pt)
  , pos_increment_(pos_increment)
  , orient_increment_(orient_increment)
{
}

CartTrajectoryPt::CartTrajectoryPt(const Frame &wobj_pt, const descartes_core::TimingConstraint &timing)
  : descartes_core::TrajectoryPt(timing)
  , tool_base_(Eigen::Affine3d::Identity())
  , tool_pt_(Eigen::Affine3d::Identity())
  , wobj_base_(Eigen::Affine3d::Identity())
  , wobj_pt_(wobj_pt)
  , pos_increment_(0)
  , orient_increment_(0)
{
}

bool CartTrajectoryPt::getClosestCartPose(const std::vector<double> &seed_state, const RobotModel &model,
                                          Eigen::Affine3d &pose) const
{
  NOT_IMPLEMENTED_ERR(false);
}

bool CartTrajectoryPt::getNominalCartPose(const std::vector<double> &seed_state, const RobotModel &model,
                                          Eigen::Affine3d &pose) const
{
  /* Simply return wobj_pt expressed in world */
  pose = wobj_base_.frame * wobj_pt_.frame;
  return true;  // TODO can this ever return false?
}

bool CartTrajectoryPt::computeCartesianPoses(EigenSTL::vector_Affine3d &poses) const
{
  EigenSTL::vector_Affine3d sampled_wobj_pts = uniform(wobj_pt_, orient_increment_, pos_increment_);
  EigenSTL::vector_Affine3d sampled_tool_pts = uniform(tool_pt_, orient_increment_, pos_increment_);

  poses.clear();
  poses.reserve(sampled_wobj_pts.size() * sampled_tool_pts.size());
  for (size_t wobj_pt = 0; wobj_pt < sampled_wobj_pts.size(); ++wobj_pt)
  {
    for (size_t tool_pt = 0; tool_pt < sampled_tool_pts.size(); ++tool_pt)
    {
      Eigen::Affine3d pose =
          wobj_base_.frame * sampled_wobj_pts[wobj_pt] * sampled_tool_pts[tool_pt].inverse() * tool_base_.frame_inv;

      poses.push_back(pose);
    }
  }

  return !poses.empty();
}

double CartTrajectoryPt::computeWeldingCost(Eigen::Affine3d referencePose, Eigen::Affine3d pose) const
{
  /*
  Poses are supposed to only differ in rotation matrix, the translations should be the same,
  unless a certain position tolerance is used.
  The reference pose is supposed to be the optimal solution, any extra rotations will result in cost increase.
  Because of the convention used, there are two welding angles:
    1) rotation around the local Y-axis, the most critical rotation in terms of weld quality
    2) rotation around the local X-axis

  The problem now is to find those angles given a reference pose and a new, rotated 'pose'.
  Method: we calculate the Z-axis of the rotated 'pose'.
  Then this Z-axis is projected onto the X-Z-plane of the reference pose. Call this vector 'projectionZ'.
  This projectionZ can then be used to calculate the Y- and X-rotation-angles, using vector dot products:
    1) the dot product between 'projectionZ' and the Z-axis of the referencepose defines the Y-rotation-angle
    2) the dot product between 'projectionZ' and the Z-axis of the rotated 'pose' defines the X-rotation-angle.
  */
  double costFactorX = 1.0;
  double costFactorY = 10.0;
  double cost;
  
  //Since translations are supposed to be the same, the result is supposed to be a rotation only transform
  
  Eigen::Vector3d zeroVec(0.0,0.0,0.0);
  pose.translation() = zeroVec;
  referencePose.translation() = zeroVec;

  //We transform the 'pose' with the inverse of the referencePose transform:
  Eigen::Affine3d revertedPose;
  revertedPose = referencePose.inverse() * pose;
  //This 'revertedPose' transform now represents a rotation of the origin frame.
  //The reference frame is now the origin frame:
  //referencePose = referencePose.inverse() * referencePose; //Should be identity; this step is actually useless

  //Calculate Z-axis of pose- and referenceframe:
  Eigen::Vector3d axisZ(0.0,0.0,1.0);
  Eigen::Vector3d poseZ;
  poseZ = revertedPose * axisZ;

  //To calculate the projection of poseZ in the X-Z-plane of the reference frame we set its Y-component to zero:
  //We can do this because the referencePose transform is now an identity transform.
  Eigen::Vector3d projectionZ(poseZ[0], 0.0, poseZ[2]);


  //Now calculate the rotation angles using dot products:
  double xRotation, yRotation;
  yRotation = acos(axisZ.dot(projectionZ) / projectionZ.norm());
  xRotation = acos(poseZ.dot(projectionZ) / (poseZ.norm() * projectionZ.norm()));

  cost = costFactorX * std::abs(xRotation) + costFactorY * std::abs(yRotation);
  return cost;
}

void CartTrajectoryPt::getCartesianPoses(const RobotModel &model, EigenSTL::vector_Affine3d &poses) const
{
  EigenSTL::vector_Affine3d all_poses;
  poses.clear();

  if (computeCartesianPoses(all_poses))
  {
    poses.reserve(all_poses.size());
    for (const auto &pose : all_poses)
    {
      if (model.isValid(pose))
      {
        poses.push_back(pose);
      }
    }
  }
  else
  {
    ROS_ERROR("Failed for find ANY cartesian poses");
  }

  if (poses.empty())
  {
    ROS_WARN("Failed for find VALID cartesian poses, returning");
  }
  else
  {
    ROS_DEBUG_STREAM("Get cartesian poses, sampled: " << all_poses.size() << ", with " << poses.size()
                                                      << " valid(returned) poses");
  }
}

bool CartTrajectoryPt::getClosestJointPose(const std::vector<double> &seed_state, const RobotModel &model,
                                           std::vector<double> &joint_pose) const
{
  Eigen::Affine3d nominal_pose, candidate_pose;

  if (!model.getFK(seed_state, candidate_pose))
  {
    ROS_ERROR_STREAM("FK failed for seed pose for closest joint pose");
    return false;
  }

  // getting pose values
  Eigen::Vector3d t = candidate_pose.translation();
  Eigen::Vector3d rpy = candidate_pose.rotation().eulerAngles(0, 1, 2);

  std::vector<std::tuple<double, double, double> > vals = {
    std::make_tuple(t(0), wobj_pt_.position_tolerance.x_lower, wobj_pt_.position_tolerance.x_upper),
    std::make_tuple(t(1), wobj_pt_.position_tolerance.y_lower, wobj_pt_.position_tolerance.y_upper),
    std::make_tuple(t(2), wobj_pt_.position_tolerance.z_lower, wobj_pt_.position_tolerance.z_upper),
    std::make_tuple(rpy(0), wobj_pt_.orientation_tolerance.x_lower, wobj_pt_.orientation_tolerance.x_upper),
    std::make_tuple(rpy(1), wobj_pt_.orientation_tolerance.y_lower, wobj_pt_.orientation_tolerance.y_upper),
    std::make_tuple(rpy(2), wobj_pt_.orientation_tolerance.z_lower, wobj_pt_.orientation_tolerance.z_upper)
  };

  std::vector<double> closest_pose_vals = { t(0), t(1), t(2), rpy(0), rpy(1), rpy(2) };
  bool solve_ik = false;
  for (int i = 0; i < vals.size(); i++)
  {
    auto &lower = std::get<1>(vals[i]);
    auto &upper = std::get<2>(vals[i]);
    auto &v = std::get<0>(vals[i]);

    if (std::abs(upper - lower) > EQUALITY_TOLERANCE)
    {
      auto bounds = std::make_pair(lower, upper);
      if (std::minmax({ lower, v, upper }) != bounds)
      {
        solve_ik = true;
        closest_pose_vals[i] = v < lower ? lower : upper;
        ROS_DEBUG("Cartesian nominal [%i] exceeded bounds: [val: %f, lower: %f, upper: %f]", i, v, lower, upper);
      }
    }
    else
    {
      if (std::abs(v - lower) > EQUALITY_TOLERANCE)
      {
        solve_ik = true;
        ROS_DEBUG("Cartesian nominals [%i] differ: [val: %f, lower: %f, upper: %f]", i, v, lower, upper);
        closest_pose_vals[i] = lower;
      }
    }
  }

  if (solve_ik)
  {
    Eigen::Affine3d closest_pose = descartes_core::utils::toFrame(
        closest_pose_vals[0], closest_pose_vals[1], closest_pose_vals[2], closest_pose_vals[3], closest_pose_vals[4],
        closest_pose_vals[5], descartes_core::utils::EulerConventions::XYZ);
    if (!model.getIK(closest_pose, seed_state, joint_pose))
    {
      ROS_WARN_STREAM("Ik failed on closest pose");

      std::vector<std::vector<double> > joint_poses;
      getJointPoses(model, joint_poses);
      if (joint_poses.size() > 0)
      {
        ROS_WARN_STREAM("Closest cartesian pose not found, returning closest to seed joint pose");

        double sd = std::numeric_limits<double>::max();
        double d;
        for (auto j : joint_poses)
        {
          d = distance(seed_state, j);
          if (sd > d)
          {
            sd = d;
            joint_pose = j;
          }
        }
      }
      else
      {
        ROS_ERROR_STREAM("getClosestJointPose failed, no valid joint poses for this point");
        return false;
      }

      return true;
    }
  }
  else
  {
    joint_pose.assign(seed_state.begin(), seed_state.end());
  }

  return true;
}

bool CartTrajectoryPt::getNominalJointPose(const std::vector<double> &seed_state, const RobotModel &model,
                                           std::vector<double> &joint_pose) const
{
  Eigen::Affine3d robot_pose = wobj_base_.frame * wobj_pt_.frame * tool_pt_.frame_inv * tool_base_.frame_inv;
  return model.getIK(robot_pose, seed_state, joint_pose);
}

void CartTrajectoryPt::getJointPoses(const RobotModel &model, std::vector<std::vector<double> > &joint_poses) const
{
  joint_poses.clear();

  EigenSTL::vector_Affine3d poses;
  if (computeCartesianPoses(poses))
  {
    poses.reserve(poses.size());
    for (const auto &pose : poses)
    {
      std::vector<std::vector<double> > local_joint_poses;
      if (model.getAllIK(pose, local_joint_poses))
      {
        joint_poses.insert(joint_poses.end(), local_joint_poses.begin(), local_joint_poses.end());
      }
    }
  }
  else
  {
    ROS_ERROR("Failed for find ANY cartesian poses");
  }

  if (joint_poses.empty())
  {
    ROS_WARN("Failed for find ANY joint poses, returning");
  }
  else
  {
    ROS_DEBUG_STREAM("Get joint poses, sampled: " << poses.size() << ", with " << joint_poses.size()
                                                  << " valid(returned) poses");
  }
}

void CartTrajectoryPt::getJointPoses(const descartes_core::RobotModel &model, std::vector<std::vector<double> > &joint_poses, std::vector<double> &costs) const
{
  joint_poses.clear();

  EigenSTL::vector_Affine3d poses;
  if (computeCartesianPoses(poses))
  {
    poses.reserve(poses.size());
    costs.reserve(poses.size());
    Eigen::Affine3d referencePose = wobj_base_.frame * wobj_pt_.frame * tool_pt_.frame_inv * tool_base_.frame_inv;
    for (const auto &pose : poses)
    {
      std::vector<std::vector<double> > local_joint_poses;
      if (model.getAllIK(pose, local_joint_poses))
      {
        joint_poses.insert(joint_poses.end(), local_joint_poses.begin(), local_joint_poses.end());
        //For every joint solution found we need to add the cost once:
        int counter = 0;
        while(counter < local_joint_poses.size())
        {
          costs.push_back(computeWeldingCost(referencePose, pose));
          ++counter;
        }

      }
    }
  }
  else
  {
    ROS_ERROR("Failed for find ANY cartesian poses");
  }

  if (joint_poses.empty())
  {
    ROS_WARN("Failed for find ANY joint poses, returning");
  }
  else
  {
    ROS_DEBUG_STREAM("Get joint poses, sampled: " << poses.size() << ", with " << joint_poses.size()
                                                  << " valid(returned) poses");
  }
}

bool CartTrajectoryPt::isValid(const RobotModel &model) const
{
  Eigen::Affine3d robot_pose = wobj_base_.frame * wobj_pt_.frame * tool_pt_.frame_inv * tool_base_.frame_inv;
  return model.isValid(robot_pose);
}

bool CartTrajectoryPt::setDiscretization(const std::vector<double> &discretization)
{
  NOT_IMPLEMENTED_ERR(false);
}

} /* namespace descartes_trajectory */
