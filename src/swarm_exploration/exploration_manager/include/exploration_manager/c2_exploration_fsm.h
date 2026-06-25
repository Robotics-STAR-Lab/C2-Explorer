#ifndef _C2_EXPLORATION_FSM_H_
#define _C2_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <exploration_manager/DroneState.h>
#include <exploration_manager/MeetingOpt.h>
#include <exploration_manager/MeetingOptResponse.h>
#include <exploration_manager/expl_data.h>
#include <bspline/Bspline.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <functional>
#include <unordered_set>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace c2_expl {
class FastPlannerManager;
class C2ExplorationManager;
class PlanningVisualization;
class CommunicationGraph;
class HGrid;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };

class C2ExplorationFSM {

public:
  C2ExplorationFSM();
  ~C2ExplorationFSM();

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  static constexpr int8_t kMeetingOptProposal = 0;
  static constexpr int8_t kMeetingOptCommit = 1;
  static constexpr int8_t kMeetingOptCancel = 2;
  static constexpr int8_t kMeetingOptFinalize = 3;
  static constexpr int8_t kMeetingOptProposalResponse = 0;
  static constexpr int8_t kMeetingOptCommitAck = 1;
  static constexpr int32_t kMeetingOptAccept = 1;
  static constexpr int32_t kMeetingOptReject = 2;

  /* workflow functions */
  int getId();
  int callExplorationPlanner();
  void visualize(int content);
  void drawFrontierVisualization();
  void drawPlanningVisualization();
  void clearVisMarker();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void safetyCallback(const ros::TimerEvent& e);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  /* Swarm and MeetingOpt callbacks */
  void droneStateTimerCallback(const ros::TimerEvent& e);
  void droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg);
  void optTimerCallback(const ros::TimerEvent& e);
  void optMsgCallback(const exploration_manager::MeetingOptConstPtr& msg);
  void optResMsgCallback(const exploration_manager::MeetingOptResponseConstPtr& msg);

  void swarmTrajCallback(const bspline::BsplineConstPtr& msg);
  void swarmTrajTimerCallback(const ros::TimerEvent& e);

  /* helper functions */
  static int resolveCenterGridId(
      const shared_ptr<HGrid>& hgrid, const DroneState& drone_state, const int center_idx);
  static std::string idsToStr(const vector<int>& ids);
  static void centersToGridIds(
      const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& centers, vector<int>& grid_ids,
      bool dedup);
  static void centersToCenterIds(
      const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& centers, vector<int>& center_ids,
      bool dedup);
  static void centersToCenterGridIds(const shared_ptr<HGrid>& hgrid,
      const vector<Vector3d>& centers, vector<int>& center_grid_ids);
  static void centersToCenterHulls(const shared_ptr<HGrid>& hgrid,
      const vector<Vector3d>& centers, vector<vector<Vector3d>>& center_hulls);
  static void preserveCenterHulls(const shared_ptr<HGrid>& hgrid,
      const vector<Vector3d>& old_centers, const vector<int>& old_center_ids,
      const vector<int>& old_center_grid_ids, const vector<vector<Vector3d>>& old_center_hulls,
      const vector<Vector3d>& new_centers, const vector<int>& new_center_ids,
      const vector<int>& new_center_grid_ids, vector<vector<Vector3d>>& new_center_hulls);
  static void packHullsToMsg(const vector<vector<Vector3d>>& hulls, vector<int32_t>& offsets,
      vector<geometry_msgs::Point>& points);
  static void unpackHullsFromMsg(const vector<int32_t>& offsets,
      const vector<geometry_msgs::Point>& points, vector<vector<Vector3d>>& hulls);
  static void uniqueIds(const vector<int>& in, vector<int>& out);
  static void applyAllocationToState(const shared_ptr<HGrid>& hgrid, const vector<int>& ids,
      const vector<Vector3d>& centers, const vector<vector<Vector3d>>& center_hulls,
      DroneState& state);
  static void msgPointsToCenters(
      const vector<geometry_msgs::Point>& points, vector<Vector3d>& centers);
  static void centersToMsgPoints(
      const vector<Vector3d>& centers, vector<geometry_msgs::Point>& points);
  static void fillMeetingOptPayload(const vector<int>& ego_ids, const vector<int>& other_ids,
      const vector<Vector3d>& ego_centers, const vector<Vector3d>& other_centers,
      const vector<vector<Vector3d>>& ego_hulls, const vector<vector<Vector3d>>& other_hulls,
      exploration_manager::MeetingOpt& opt);
  static void clearOptPendingLock(ExplorationData& ed);
  static bool hasOptPendingFromHost(const ExplorationData& ed, const int host_id,
      const double stamp);
  static bool hasAppliedUnconfirmedTx(const ExplorationData& ed, const int host_id,
      const double stamp);
  static void clearAppliedUnconfirmedTx(ExplorationData& ed);
  static void rollbackAppliedUnconfirmedTx(ExplorationData& ed, const int self_id);
  static void clearHostOptSession(ExplorationData& ed);
  static void collectStateTaskGridIds(
      const DroneState& st, const shared_ptr<HGrid>& hgrid, vector<int>& out);
  static void clearPeerRejectBackoff(ExplorationData& ed, const int peer_id);
  static double registerPeerRejectBackoff(
      ExplorationData& ed, const shared_ptr<FSMParam>& fp, const int peer_id, const double now);
  static bool containsId(const vector<int>& ids, const int id);
  static bool pushUniqueId(vector<int>& ids, const int id);
  static void forEachRemoteParticipant(const vector<int>& participant_ids, const int self_id,
      const std::function<void(size_t, int)>& fn);
  static bool findParticipantIndex(
      const vector<int>& participant_ids, const int drone_id, size_t& idx);
  static bool isCenterSetResolvableOnLocalHGrid(const shared_ptr<HGrid>& hgrid,
      const vector<geometry_msgs::Point>& points, const double tol, int& invalid_count);
  static void optDebugWarn(const shared_ptr<FSMParam>& fp, const std::string& msg);
  static void optDebugTrace(const shared_ptr<FSMParam>& fp, const std::string& msg);
  static void publishRepeated(
      ros::Publisher& pub, const exploration_manager::MeetingOpt& msg, const int repeat_send_num);
  static void publishRepeated(ros::Publisher& pub,
      const exploration_manager::MeetingOptResponse& msg, const int repeat_send_num);
  static int getMeetingOptRepeatNum(const shared_ptr<FSMParam>& fp, const int8_t phase);
  static int getMeetingOptResRepeatNum(const shared_ptr<FSMParam>& fp, const int8_t phase);
  static void sendMeetingOptAdaptive(ros::Publisher& opt_pub, const shared_ptr<FSMParam>& fp,
      const int from_id, const int to_id, const double stamp, const int8_t phase);
  static void sendMeetingOptAdaptive(ros::Publisher& opt_pub, const shared_ptr<FSMParam>& fp,
      const int from_id, const int to_id, const double stamp, const int8_t phase,
      const vector<int>& ego_ids, const vector<int>& other_ids, const vector<Vector3d>& ego_centers,
      const vector<Vector3d>& other_centers, const vector<vector<Vector3d>>& ego_hulls,
      const vector<vector<Vector3d>>& other_hulls);
  static void sendMeetingOptResponseAdaptive(ros::Publisher& opt_res_pub,
      const shared_ptr<FSMParam>& fp, const int from_id, const int to_id, const double stamp,
      const int8_t phase, const int32_t status);
  static void handleHostWaitSession(ExplorationData& ed, const shared_ptr<FSMParam>& fp,
      ros::Publisher& opt_pub, const int self_id, const double now, const bool wait_commit_ack);

  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<C2ExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;
  shared_ptr<CommunicationGraph> comm_graph_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, bspline_pub_;

  // Swarm state
  ros::Publisher drone_state_pub_, opt_pub_, opt_res_pub_, swarm_traj_pub_;
  ros::Subscriber drone_state_sub_, opt_sub_, opt_res_sub_, swarm_traj_sub_;
  ros::Timer drone_state_timer_, opt_timer_, swarm_traj_timer_;
};

}  // namespace c2_expl

#endif
