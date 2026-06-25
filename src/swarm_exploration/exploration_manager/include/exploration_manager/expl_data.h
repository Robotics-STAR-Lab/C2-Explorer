#ifndef EXPL_DATA_H
#define EXPL_DATA_H

#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <utility>
#include <ros/ros.h>
#include <bspline/Bspline.h>
#include <plan_env/comm_data.h>
#include <active_perception/uniform_grid.h>

using Eigen::Vector3d;
using std::map;
using std::pair;
using std::string;
using std::vector;

namespace c2_expl {

// enum GridState { EXPLORED, EXPLORING, EXPLORABLE };

struct Viewpoint;

struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_, is_frontier_updated_;
  vector<string> state_str_;

  Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  vector<Vector3d> start_poss;
  bspline::Bspline newest_traj_;

  // Swarm collision avoidance
  bool avoid_collision_, go_back_;
  ros::Time fsm_init_time_;
  ros::Time last_check_frontier_time_;
  ros::Time expl_start_time_;

  Eigen::Vector3d start_pos_;
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_;  // second

  // Swarm
  double attempt_interval_;   // Min interval of opt attempt
  double meeting_opt_interval_;  // Min interval of successful meeting opt
  double state_fresh_timeout_;  // Max age of received peer state (local receive time based)
  double opt_wait_timeout_;     // Timeout waiting for opt responses
  double opt_retry_interval_;   // Interval for proposal retransmission to unacked peers
  int opt_retry_max_;           // Max retransmission rounds while waiting response
  double opt_reject_backoff_;     // Base cooldown after proposal reject from a peer
  double opt_reject_backoff_max_; // Max cooldown cap for repeated rejects
  int repeat_send_num_;
  bool debug_opt_output_ = false;  // Verbose MeetingOpt communication debug output

  // Visualization
  double color_r_;
  double color_g_;
  double color_b_;
  double color_a_;
};

struct DroneState {
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  double yaw_;
  double stamp_;                // Stamp of pos,vel,yaw
  double recv_stamp_;           // Local receive time of this state
  double recent_attempt_time_;  // Stamp of latest opt attempt with any drone

  vector<int> grid_ids_;         // Id and state of grid tour
  vector<int> center_ids_;       // Optional non-hull center keys; -1 for hull-backed tasks
  vector<Vector3d> center_positions_;  // Assigned unknown centers
  vector<vector<Vector3d>> center_hulls_;  // Convex hull for each assigned center
  vector<int> center_grid_ids_;  // Grid id for each assigned center (same order as center_positions_)
  double recent_interact_time_;  // Stamp of latest opt with this drone
};

struct ExplorationData {
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<Vector3d> views1_, views2_;
  vector<double> yaws_;
  vector<Vector3d> frontier_tour_;
  vector<vector<Vector3d>> other_tours_;

  vector<Viewpoint> viewpoints_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_;  // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_, kino_path_;
  Vector3d next_pos_;
  double next_yaw_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;

  // Swarm, other drones' state
  vector<DroneState> swarm_state_;
  vector<int> neighbor_ids_;  // Communicable neighbor list.
  vector<double> meeting_opt_stamps_, meeting_opt_res_stamps_, meeting_opt_commit_res_stamps_;
  double meeting_opt_stamp_;
  vector<int> opt_participant_ids_;
  vector<vector<int>> opt_alloc_ids_;
  vector<vector<Vector3d>> opt_alloc_centers_;
  vector<vector<vector<Vector3d>>> opt_alloc_center_hulls_;
  vector<int> opt_wait_ids_;
  vector<int> opt_ack_ids_;
  vector<int> opt_commit_ack_ids_;
  vector<double> opt_peer_backoff_until_;  // Host-side per-peer cooldown deadline
  vector<int> opt_peer_reject_streak_;     // Host-side consecutive reject counter
  int opt_preferred_gate_host_id_;         // Preferred host currently observed as interval-gated
  double opt_preferred_gate_since_;        // Local time when preferred-host gate observation started
  int opt_pending_host_id_;
  double opt_pending_stamp_;
  double opt_pending_recv_time_;
  bool opt_commit_apply_pending_;
  int opt_commit_apply_host_id_;
  double opt_commit_apply_stamp_;
  double opt_commit_apply_time_;
  bool opt_commit_backup_valid_;
  DroneState opt_commit_backup_self_state_;
  DroneState opt_commit_backup_host_state_;
  double opt_wait_start_time_;
  double opt_last_send_time_;
  int opt_retry_count_;
  double opt_commit_wait_start_time_;
  double opt_commit_last_send_time_;
  int opt_commit_retry_count_;
  bool reallocated_, wait_opt_response_, wait_opt_commit_ack_;

  // Coverage planning
  vector<Vector3d> region_tour_;
  vector<Vector3d> grid_tour_, grid_tour2_;
  // int prev_first_id_;
  vector<int> last_grid_ids_;

  int plan_num_;

  bool init_;
};

struct ExplorationParam {
  // params
  bool refine_local_;
  int refined_num_;
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  string tsp_dir_;   // resource dir of tsp solver
  string mtsp_dir_;  // resource dir of tsp solver
  double relax_time_;
  int init_plan_num_;
  int prob_type_;

  // Center allocation anti-oscillation
  double center_switch_penalty_;
  double center_stay_bonus_;
  
  // Swarm
  int drone_num_;
  int drone_id_;
};

}  // namespace c2_expl

#endif  // EXPL_DATA_H
