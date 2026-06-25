#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
// #include <swarm_msgs/swarm_drone_basecoor.h>

#include <array>
#include <memory>
#include <random>
#include <string>
#include <vector>

using std::default_random_engine;
using std::normal_distribution;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace c2_expl {
class SDFMap;

class MapROS {
public:
  MapROS();
  ~MapROS();
  void setMap(SDFMap* map);
  void init();

private:
  void depthPoseCallback(
      const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
  void cloudOdomCallback(
      const sensor_msgs::PointCloud2ConstPtr& msg, const nav_msgs::OdometryConstPtr& odom);
  void updateESDFCallback(const ros::TimerEvent& /*event*/);
  void visCallback(const ros::TimerEvent& /*event*/);
  //   void basecoorCallback(const swarm_msgs::swarm_drone_basecoorConstPtr& msg);

  void publishMapAll();
  void publishMapLocal();
  void publishKnownLocal();
  void publishESDF();
  void publishUpdateRange();
  void publishUnknown();
  void publishFree();
  void publishDepth();
  void publishVirtualGround(const Eigen::Vector3i& min_cut, const Eigen::Vector3i& max_cut,
      const std::vector<char>& ground_mask, int size_y);
  bool loadBoundaryCsv(const std::string& path);
  void buildBoundaryMask();
  bool isInBoundaryIndex(int x, int y) const;
  void publishBoundaryMarker();

  void processDepthImage();

  SDFMap* map_;
  // may use ExactTime?
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
      geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
      nav_msgs::Odometry>
      SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

  ros::NodeHandle node_;
  //   shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerCloudPose sync_cloud_odom_;
  //   SynchronizerImagePose sync_image_pose_;
  //   SynchronizerCloudPose sync_cloud_pose_;

  ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, unknown_pub_,
      update_range_pub_, depth_pub_, map_free_pub_, map_known_local_pub_, virtual_ground_pub_,
      boundary_marker_pub_;
  ros::Timer esdf_timer_, vis_timer_;
  ros::Subscriber basecoor_sub_;

  // params, depth projection
  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  string frame_id_;
  // msg publication
  double esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_;
  bool use_local_only_;
  bool virtual_ground_enable_;
  double virtual_ground_z_;
  int virtual_ground_stride_;
  bool boundary_enable_;
  string boundary_csv_;
  double boundary_offset_x_, boundary_offset_y_;
  bool boundary_marker_enable_;
  double boundary_marker_z_, boundary_marker_width_;
  double boundary_marker_r_, boundary_marker_g_, boundary_marker_b_, boundary_marker_a_;

  // data
  // flags of map state
  bool local_updated_, esdf_need_update_;
  // input
  Eigen::Vector3d odom_pos_;
  unique_ptr<cv::Mat> depth_image_;
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;

  normal_distribution<double> rand_noise_;
  default_random_engine eng_;
  std::vector<std::array<double, 2>> boundary_poly_;
  std::vector<char> boundary_mask_;
  int boundary_mask_size_y_;
  bool boundary_ready_;
  double boundary_min_x_, boundary_max_x_, boundary_min_y_, boundary_max_y_;

  ros::Time map_start_time_;

  friend SDFMap;
};
}  // namespace c2_expl

#endif
