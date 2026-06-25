#ifndef _PERCEPTION_UTILS_H_
#define _PERCEPTION_UTILS_H_

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <iostream>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace c2_expl {
class PerceptionUtils {
public:
  PerceptionUtils(ros::NodeHandle& nh);
  ~PerceptionUtils() {
  }
  // Set position and yaw
  void setPose(const Vector3d& pos, const double& yaw);

  // Get info of current pose
  void getFOV(vector<Vector3d>& list1, vector<Vector3d>& list2);
  bool insideFOV(const Vector3d& point);
  void getFOVBoundingBox(Vector3d& bmin, Vector3d& bmax);

private:
  // Data
  // Current camera pos and yaw
  Vector3d pos_;
  double yaw_, pitch_;
  // Camera plane's normals in world frame
  vector<Vector3d> normals_;

  /* Params */
  // lidar parameters
  double lidar_pitch_;
  double lidar_max_dist_, lidar_vis_dist_, lidar_left_angle_, lidar_right_angle_, lidar_top_angle_,
      lidar_bottom_angle_;

  double max_dist_, vis_dist_, left_angle_, right_angle_, top_angle_, bottom_angle_;
  // Normal vectors of camera FOV planes in camera frame
  Vector3d n_top_, n_bottom_, n_left_, n_right_;
  // Transform between camera and body
  Eigen::Matrix4d T_cb_, T_bc_;
  // FOV vertices in body frame
  vector<Vector3d> cam_vertices1_, cam_vertices2_;
};

}  // namespace c2_expl
#endif