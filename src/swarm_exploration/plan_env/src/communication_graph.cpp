#include <plan_env/communication_graph.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>

#include <cmath>

namespace c2_expl {

// Constants for visualization colors
const std_msgs::ColorRGBA CONNECTED_COLOR = []() {
  std_msgs::ColorRGBA c;
  c.r = 0.0;
  c.g = 1.0;
  c.b = 0.0;
  c.a = 1.0;
  return c;
}();

CommunicationGraph::CommunicationGraph(const EDTEnvironment::Ptr& edt_env, ros::NodeHandle& nh) {
  this->edt_env_ = edt_env;
  sdf_map_ = edt_env_->sdf_map_;

  cp_.reset(new CommunicationParams);
  cd_.reset(new CommunicationData);

  // Load parameters
  nh.param("communication/connection_threshold", cp_->connection_threshold_, 500.0);
  nh.param("communication/heartbeat_interval", cp_->heartbeat_interval_, -1.0);
  nh.param("communication/neighbor_timeout", cp_->neighbor_timeout_, -1.0);
  nh.param("exploration/drone_id", drone_id_, 1);

  // Initialize communication data
  cd_->ego_id_ = drone_id_;
  cd_->have_odom_ = false;
  cd_->linktype_str_ = { "NO_CONNECTION", "CONNECTED" };

  // ROS subscribers and publishers
  odom_sub_ = nh.subscribe("/odom_world", 1, &CommunicationGraph::odometryCallback, this);
  heartbeat_timer_ = nh.createTimer(
      ros::Duration(cp_->heartbeat_interval_), &CommunicationGraph::heartbeatTimerCallback, this);
  heartbeat_pub_ = nh.advertise<plan_env::Heartbeat>("/communication/heartbeat_send", 10);
  heartbeat_sub_ = nh.subscribe(
      "/communication/heartbeat_recv", 100, &CommunicationGraph::heartbeatCallback, this);
  graph_pub_ = nh.advertise<visualization_msgs::Marker>("/communication/graph_vis", 1);
}

CommunicationGraph::~CommunicationGraph() {
}

void CommunicationGraph::updateDynamicGraph(const HeartbeatMsg& hb_msg) {
  LinkType current_link_type = evaluateLinkType(cd_->ego_position_, hb_msg.position_);
  auto neighbor_it = neighbors_.find(hb_msg.drone_id_);

  if (neighbor_it == neighbors_.end()) {
    // Add new neighbor if connected
    if (current_link_type != LinkType::NO_CONNECTION) {
      addNewNeighbor(hb_msg.drone_id_, hb_msg.position_, current_link_type);
    }
  } else {
    // Update existing neighbor
    updateExistingNeighbor(
        neighbor_it->second, hb_msg.position_, current_link_type, hb_msg.drone_id_);
  }
}

void CommunicationGraph::addNewNeighbor(
    int drone_id, const Eigen::Vector3d& position, LinkType link_type) {
  ROS_INFO("[CommGraph] Drone %d detected new neighbor %d", drone_id_, drone_id);

  NeighborInfo new_neighbor;
  new_neighbor.neighbor_id_ = drone_id;
  new_neighbor.neighbor_position_ = position;
  new_neighbor.last_update_time_ = ros::Time::now();
  new_neighbor.connection_type_ = link_type;

  neighbors_[drone_id] = new_neighbor;
}

void CommunicationGraph::updateExistingNeighbor(
    NeighborInfo& neighbor, const Eigen::Vector3d& position, LinkType new_link_type, int drone_id) {
  LinkType previous_link_type = neighbor.connection_type_;

  // Update only if not consecutively NO_CONNECTION
  if (new_link_type != LinkType::NO_CONNECTION || previous_link_type != LinkType::NO_CONNECTION) {
    neighbor.last_update_time_ = ros::Time::now();
  }

  neighbor.neighbor_position_ = position;
  neighbor.connection_type_ = new_link_type;

  // Log link type changes
  if (previous_link_type != new_link_type) {
    ROS_INFO("[CommGraph] Drone %d neighbor %d link type changed: %s --> %s", drone_id_, drone_id,
        cd_->linktype_str_[static_cast<int>(previous_link_type)].c_str(),
        cd_->linktype_str_[static_cast<int>(new_link_type)].c_str());
  }
}

LinkType CommunicationGraph::evaluateLinkType(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2) {
  double dist = calculateDistance(pos1, pos2);
  return classifyLinkType(dist);
}

LinkType CommunicationGraph::classifyLinkType(double distance) {
  if (distance <= cp_->connection_threshold_) {
    return LinkType::CONNECTED;
  }
  return LinkType::NO_CONNECTION;
}

void CommunicationGraph::cleanupStaleNeighbors() {
  ros::Time curr_time = ros::Time::now();

  // delete neighbors that have timed out
  for (auto it = neighbors_.begin(); it != neighbors_.end();) {
    if ((curr_time - it->second.last_update_time_).toSec() > cp_->neighbor_timeout_) {
      ROS_WARN("[CommGraph] neighbor %d timed out and has been removed from the list.", it->first);
      it = neighbors_.erase(it);
    } else {
      ++it;
    }
  }
}

void CommunicationGraph::updateAllLinks() {
  // Reclassify all existing links based on current positions
  for (auto& neighbor_pair : neighbors_) {
    NeighborInfo& neighbor = neighbor_pair.second;
    neighbor.connection_type_ = evaluateLinkType(cd_->ego_position_, neighbor.neighbor_position_);
    neighbor.last_update_time_ = ros::Time::now();
  }
}

void CommunicationGraph::getNeighborIds(vector<int>& neighbor_ids) {
  neighbor_ids.clear();
  for (const auto& neighbor_pair : neighbors_) {
    if (neighbor_pair.second.connection_type_ != LinkType::NO_CONNECTION) {
      neighbor_ids.push_back(neighbor_pair.first);
    }
  }
  neighbor_ids.push_back(cd_->ego_id_);  // Always include self
}

void CommunicationGraph::visualizeCommunicationGraph() {
  if (!cd_->have_odom_) return;

  visualization_msgs::Marker link_marker;
  link_marker.header.frame_id = "world";
  link_marker.header.stamp = ros::Time::now();
  link_marker.ns = "communication_links";
  link_marker.id = drone_id_;
  link_marker.type = visualization_msgs::Marker::LINE_LIST;
  link_marker.action = visualization_msgs::Marker::ADD;
  link_marker.scale.x = 0.08;
  link_marker.pose.orientation.w = 1.0;  // Identity quaternion

  for (const auto& neighbor_pair : neighbors_) {
    const NeighborInfo& neighbor = neighbor_pair.second;
    if (neighbor.connection_type_ == LinkType::NO_CONNECTION) continue;

    // Add line points
    geometry_msgs::Point start_point, end_point;
    start_point.x = cd_->ego_position_(0);
    start_point.y = cd_->ego_position_(1);
    start_point.z = cd_->ego_position_(2);
    end_point.x = neighbor.neighbor_position_(0);
    end_point.y = neighbor.neighbor_position_(1);
    end_point.z = neighbor.neighbor_position_(2);

    link_marker.points.push_back(start_point);
    link_marker.points.push_back(end_point);

    link_marker.colors.push_back(CONNECTED_COLOR);
    link_marker.colors.push_back(CONNECTED_COLOR);
  }

  graph_pub_.publish(link_marker);
}

void CommunicationGraph::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  cd_->ego_position_(0) = msg->pose.pose.position.x;
  cd_->ego_position_(1) = msg->pose.pose.position.y;
  cd_->ego_position_(2) = msg->pose.pose.position.z;

  if (!cd_->have_odom_) {
    cd_->have_odom_ = true;
    cd_->last_odom_time_ = ros::Time::now();
  }
}

// periodic heartbeat to broadcast own state
void CommunicationGraph::heartbeatTimerCallback(const ros::TimerEvent& e) {
  // periodically clean up stale neighbors
  this->cleanupStaleNeighbors();

  // update all links
  this->updateAllLinks();

  plan_env::Heartbeat hb_msg;
  hb_msg.drone_id = cd_->ego_id_;
  hb_msg.pos[0] = cd_->ego_position_(0);
  hb_msg.pos[1] = cd_->ego_position_(1);
  hb_msg.pos[2] = cd_->ego_position_(2);
  hb_msg.timestamp = ros::Time::now().toSec();

  // publish heartbeat
  heartbeat_pub_.publish(hb_msg);

  this->visualizeCommunicationGraph();
}

// heartbeat messages from other drones
void CommunicationGraph::heartbeatCallback(const plan_env::HeartbeatConstPtr& msg) {
  // ignore self heartbeat
  if (msg->drone_id == cd_->ego_id_) {
    return;
  }

  // ROS_WARN("[CommGraph] Heartbeat received from drone %d", msg->drone_id);

  HeartbeatMsg hb_msg;
  hb_msg.drone_id_ = msg->drone_id;
  hb_msg.position_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  hb_msg.heartbeat_timestamp_ = ros::Time(msg->timestamp);

  // process the received heartbeat
  this->updateDynamicGraph(hb_msg);
}

}  // namespace c2_expl
