#ifndef COMMUNICATION_GRAPH_H
#define COMMUNICATION_GRAPH_H

#include <ros/ros.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <plan_env/comm_data.h>
#include <plan_env/Heartbeat.h>
#include <plan_env/edt_environment.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::vector;

namespace c2_expl {
class EDTEnvironment;
class SDFMap;
struct CommunicationParams;

class CommunicationGraph {
public:
  CommunicationGraph(const EDTEnvironment::Ptr& edt_env, ros::NodeHandle& nh);
  ~CommunicationGraph();

  void getNeighborIds(vector<int>& neighbor_ids);

private:
  void updateDynamicGraph(const HeartbeatMsg& hb_msg);
  void updateAllLinks();
  LinkType classifyLinkType(double distance);
  void cleanupStaleNeighbors();

  // New helper functions
  void addNewNeighbor(int drone_id, const Eigen::Vector3d& position, LinkType link_type);
  void updateExistingNeighbor(NeighborInfo& neighbor, const Eigen::Vector3d& position,
      LinkType new_link_type, int drone_id);
  LinkType evaluateLinkType(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);

  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  // visualization
  void visualizeCommunicationGraph();

  // swarm
  void heartbeatTimerCallback(const ros::TimerEvent& e);
  void heartbeatCallback(const plan_env::HeartbeatConstPtr& msg);

  // data
  int drone_id_;
  std::map<int, NeighborInfo> neighbors_;  // key: drone_id

  shared_ptr<SDFMap> sdf_map_;
  shared_ptr<EDTEnvironment> edt_env_;
  shared_ptr<CommunicationData> cd_;
  shared_ptr<CommunicationParams> cp_;

  ros::Subscriber odom_sub_, heartbeat_sub_;
  ros::Publisher heartbeat_pub_, graph_pub_;
  ros::Timer heartbeat_timer_;

  double calculateDistance(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2) {
    return (pos1 - pos2).norm();
  }
};

}  // namespace c2_expl
#endif
