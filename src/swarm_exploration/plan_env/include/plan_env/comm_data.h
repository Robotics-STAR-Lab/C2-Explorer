#ifndef COMM_DATA_H
#define COMM_DATA_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <string>

using Eigen::Vector3d;
using std::string;
using std::vector;

namespace c2_expl {
enum class LinkType { NO_CONNECTION, CONNECTED };

struct CommunicationData {
  int ego_id_;
  Eigen::Vector3d ego_position_;

  bool have_odom_;
  ros::Time last_odom_time_;

  vector<string> linktype_str_;
};

struct NeighborInfo {
  int neighbor_id_;
  Eigen::Vector3d neighbor_position_;

  ros::Time last_update_time_;
  LinkType connection_type_;
};

struct HeartbeatMsg {
  int drone_id_;

  Eigen::Vector3d position_;
  ros::Time heartbeat_timestamp_;
};

struct CommunicationParams {
  double connection_threshold_;

  double heartbeat_interval_;
  double neighbor_timeout_;
};

}  // namespace c2_expl
#endif
