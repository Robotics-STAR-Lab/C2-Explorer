#include <plan_manage/planner_manager.h>

#include <traj_utils/planning_visualization.h>

#include <exploration_manager/c2_exploration_manager.h>
#include <exploration_manager/c2_exploration_fsm.h>
#include <exploration_manager/expl_data.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>

#include <active_perception/perception_utils.h>
#include <active_perception/hgrid.h>

#include <plan_env/communication_graph.h>
#include <active_perception/uniform_grid.h>
// #include <lkh_tsp_solver/lkh_interface.h>
// #include <lkh_mtsp_solver/lkh3_interface.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using Eigen::Vector4d;

namespace c2_expl {
C2ExplorationFSM::C2ExplorationFSM() {
}

C2ExplorationFSM::~C2ExplorationFSM() {
}

void C2ExplorationFSM::init(ros::NodeHandle& nh) {
  // Parameters
  fp_.reset(new FSMParam);
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("fsm/attempt_interval", fp_->attempt_interval_, 0.2);
  nh.param("fsm/meeting_opt_interval", fp_->meeting_opt_interval_, 1.0);
  nh.param("fsm/state_fresh_timeout", fp_->state_fresh_timeout_, 1.0);
  nh.param("fsm/opt_wait_timeout", fp_->opt_wait_timeout_, 2.0);
  nh.param("fsm/opt_retry_interval", fp_->opt_retry_interval_, 0.3);
  nh.param("fsm/opt_retry_max", fp_->opt_retry_max_, 4);
  nh.param("fsm/opt_reject_backoff", fp_->opt_reject_backoff_, 0.6);
  nh.param("fsm/opt_reject_backoff_max", fp_->opt_reject_backoff_max_, 2.5);
  nh.param("fsm/repeat_send_num", fp_->repeat_send_num_, 10);
  nh.param("fsm/debug_opt_output", fp_->debug_opt_output_, false);

  nh.param("color_r", fp_->color_r_, 1.0);
  nh.param("color_g", fp_->color_g_, 0.0);
  nh.param("color_b", fp_->color_b_, 0.0);
  nh.param("color_a", fp_->color_a_, 1.0);

  // Planning modules
  expl_manager_.reset(new C2ExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  comm_graph_ = expl_manager_->comm_graph_;

  if (fp_->opt_reject_backoff_ < 0.0) fp_->opt_reject_backoff_ = 0.0;
  if (fp_->opt_reject_backoff_max_ < 0.0) fp_->opt_reject_backoff_max_ = 0.0;

  planner_manager_ = expl_manager_->planner_manager_;

  // FSM state
  fd_.reset(new FSMData);
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH",
    "IDL"
    "E" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;

  fd_->avoid_collision_ = false;
  fd_->go_back_ = false;
  fd_->expl_start_time_ = ros::Time(0.0);

  // ROS interfaces
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &C2ExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &C2ExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &C2ExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/move_base_simple/goal", 1, &C2ExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &C2ExplorationFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);

  // Swarm, timer, pub and sub
  drone_state_timer_ =
      nh.createTimer(ros::Duration(0.05), &C2ExplorationFSM::droneStateTimerCallback, this);
  drone_state_pub_ =
      nh.advertise<exploration_manager::DroneState>("/swarm_expl/drone_state_send", 10);
  drone_state_sub_ = nh.subscribe(
      "/swarm_expl/drone_state_recv", 10, &C2ExplorationFSM::droneStateMsgCallback, this);

  opt_timer_ = nh.createTimer(ros::Duration(0.5), &C2ExplorationFSM::optTimerCallback, this);
  opt_pub_ = nh.advertise<exploration_manager::MeetingOpt>("/swarm_expl/meeting_opt_send", 100);
  opt_sub_ = nh.subscribe("/swarm_expl/meeting_opt_recv", 100, &C2ExplorationFSM::optMsgCallback,
      this, ros::TransportHints().tcpNoDelay());

  opt_res_pub_ =
      nh.advertise<exploration_manager::MeetingOptResponse>("/swarm_expl/meeting_opt_res_send", 10);
  opt_res_sub_ = nh.subscribe("/swarm_expl/meeting_opt_res_recv", 100,
      &C2ExplorationFSM::optResMsgCallback, this, ros::TransportHints().tcpNoDelay());

  swarm_traj_pub_ = nh.advertise<bspline::Bspline>("/planning/swarm_traj_send", 100);
  swarm_traj_sub_ =
      nh.subscribe("/planning/swarm_traj_recv", 100, &C2ExplorationFSM::swarmTrajCallback, this);
  swarm_traj_timer_ =
      nh.createTimer(ros::Duration(0.1), &C2ExplorationFSM::swarmTrajTimerCallback, this);
}

int C2ExplorationFSM::getId() {
  return expl_manager_->ep_->drone_id_;
}

void C2ExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  // ROS_INFO_STREAM_THROTTLE(1.0,
  //     "\033[34m[FSM]\033[34m: Drone " << getId() << " state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom");
        return;
      }
      if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0) {
        ROS_WARN_THROTTLE(1.0, "wait for init");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      // ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case IDLE: {
      double check_interval = (ros::Time::now() - fd_->last_check_frontier_time_).toSec();
      if (check_interval > 5.0) {
        ROS_WARN("Drone %d has finished exploration", getId());
        transitState(FINISH, "FSM");
      }
      break;
    }

    case PLAN_TRAJ: {
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
        // fd_->start_pt_ = fd_->odom_pos_;
        // fd_->start_vel_ = fd_->odom_vel_;
        // fd_->start_vel_.setZero();
        // fd_->start_acc_.setZero();
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        // Get yaw angle and derivatives.
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }
      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());

      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
      } else if (res == FAIL) {  // Keep trying to replan
        fd_->static_state_ = true;
        ROS_WARN("Plan fail");
      } else if (res == NO_GRID) {
        // No grid available, exploration is finished
        fd_->static_state_ = true;
        fd_->last_check_frontier_time_ = ros::Time::now();
        ROS_WARN("No grid");
        transitState(IDLE, "FSM");
        visualize(1);
        // clearVisMarker();
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        if (fd_->expl_start_time_.isZero()) {
          fd_->expl_start_time_ = ros::Time::now();
        }
        bspline_pub_.publish(fd_->newest_traj_);
        fd_->static_state_ = false;

        // fd_->newest_traj_.drone_id = planner_manager_->swarm_traj_data_.drone_id_;
        // Broadcast trajectory to other drones for collision avoidance.
        fd_->newest_traj_.drone_id = expl_manager_->ep_->drone_id_;
        swarm_traj_pub_.publish(fd_->newest_traj_);

        thread vis_thread(&C2ExplorationFSM::visualize, this, 2);
        vis_thread.detach();

        transitState(EXEC_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      auto tn = ros::Time::now();
      // Check whether replan is needed
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (tn - info->start_time_).toSec();

      if (!fd_->go_back_) {
        bool need_replan = false;
        if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
          ROS_INFO("Replan: cluster covered=====================================");
          need_replan = true;
        } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan if traj is almost fully executed
          ROS_INFO("Replan: traj fully executed=================================");
          need_replan = true;
        } else if (t_cur > fp_->replan_thresh3_) {
          // Replan after some time
          ROS_INFO("Replan: periodic call=======================================");
          need_replan = true;
        }

        if (need_replan) {
          // New frontiers found.
          if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
            // Update frontier and plan new motion
            thread vis_thread(&C2ExplorationFSM::visualize, this, 1);
            vis_thread.detach();
            transitState(PLAN_TRAJ, "FSM");
          } else {
            // No frontier detected, go to IDLE and wait
            fd_->last_check_frontier_time_ = ros::Time::now();
            transitState(IDLE, "FSM");
            ROS_WARN("Idle since no frontier is detected");
            fd_->static_state_ = true;
            replan_pub_.publish(std_msgs::Empty());
            // clearVisMarker();
            visualize(1);
          }
        }
      } else {
        // Check if reach goal
        auto pos = info->position_traj_.evaluateDeBoorT(t_cur);
        // Consider the goal reached within 1 m.
        if ((pos - expl_manager_->ed_->next_pos_).norm() < 1.0) {
          replan_pub_.publish(std_msgs::Empty());
          clearVisMarker();
          transitState(FINISH, "FSM");
          return;
        }
        // Goal not reached, replan.
        if (t_cur > fp_->replan_thresh3_ || info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan for going back
          replan_pub_.publish(std_msgs::Empty());
          transitState(PLAN_TRAJ, "FSM");
          thread vis_thread(&C2ExplorationFSM::visualize, this, 1);
          vis_thread.detach();
        }
      }

      break;
    }
  }
}

int C2ExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
  ros::Time t1 = ros::Time::now();

  int res;
  // Simple trajectory replanning: only plan from the current state to the next target.
  if (fd_->avoid_collision_ || fd_->go_back_) {  // Only replan trajectory
    res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
        fd_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
    fd_->avoid_collision_ = false;
  } else {  // Do full planning normally
    ROS_INFO("\033[1;32mDrone %d =========================\033[0m", expl_manager_->ep_->drone_id_);
    res = expl_manager_->planExploreMotion(
        fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  }

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

    fd_->newest_traj_ = bspline;
  }
  ros::Time t2 = ros::Time::now();
  ROS_INFO("Exploration planning time: %.3f s", (t2 - t1).toSec());
  return res;
}

void C2ExplorationFSM::visualize(int content) {
  if (content == 1) {
    drawFrontierVisualization();
  } else if (content == 2) {
    drawPlanningVisualization();
  }
}

void C2ExplorationFSM::drawFrontierVisualization() {
  auto ed_ptr = expl_manager_->ed_;

  // Draw frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
        visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4), "frontier", i,
        PlanningVisualization::PUBLISHER::FRONTIER);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i,
        PlanningVisualization::PUBLISHER::FRONTIER);
  }
  last_ftr_num = ed_ptr->frontiers_.size();

  // Draw top viewpoints
  visualization_->drawSpheres(ed_ptr->points_, 0.2, PlanningVisualization::Color::DeepGreen(),
      "points", 0, PlanningVisualization::PUBLISHER::VIEWPOINT);
  visualization_->drawLines(ed_ptr->views1_, ed_ptr->views2_, 0.02,
      PlanningVisualization::Color::Black(), "view_fov", 0,
      PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw next goal
  if (ed_ptr->next_goal_.allFinite() && std::isfinite(ed_ptr->next_yaw_)) {
    Eigen::Quaterniond next_q;
    next_q = Eigen::AngleAxisd(ed_ptr->next_yaw_, Eigen::Vector3d::UnitZ());
    visualization_->drawPose(ed_ptr->next_goal_, next_q, "next_goal", 0);
  }
}

void C2ExplorationFSM::drawPlanningVisualization() {
  constexpr double kExploredGridVisZ = 0.24;
  constexpr double kGridOutlineVisZ = 0.40;
  constexpr double kOtherHullVisZ = 0.58;
  constexpr double kEgoHullVisZ = 0.74;
  constexpr double kOtherCenterVisZ = 0.86;
  constexpr double kEgoCenterVisZ = 0.96;

  const bool finished = (state_ == FINISH || state_ == IDLE);
  const double color_a = finished ? 0.15 : fp_->color_a_;
  const Vector4d drone_color(fp_->color_r_, fp_->color_g_, fp_->color_b_, color_a);

  auto& states = expl_manager_->ed_->swarm_state_;
  auto& ego_state = states[getId() - 1];
  std::unordered_set<int> ego_hull_grid_ids;

  // Draw assigned center convex hulls. Ego tasks stay highlighted, while other drones are muted
  // context rendered above the grid outlines.
  for (size_t di = 0; di < states.size(); ++di) {
    vector<Vector3d> hull_l1, hull_l2;
    const auto& drone_state = states[di];
    const bool is_ego_drone = (static_cast<int>(di) + 1 == getId());
    const double hull_z = is_ego_drone ? kEgoHullVisZ : kOtherHullVisZ;
    const double center_z = is_ego_drone ? kEgoCenterVisZ : kOtherCenterVisZ;
    for (int ci = 0; ci < static_cast<int>(drone_state.center_positions_.size()); ++ci) {
      vector<Vector3d> hull;
      if (ci < static_cast<int>(drone_state.center_hulls_.size())) {
        hull = drone_state.center_hulls_[ci];
      }
      if (hull.empty()) {
        expl_manager_->hgrid_->getCenterHullByPos(drone_state.center_positions_[ci], hull, 1.0);
      }
      if (hull.size() < 2) continue;
      for (size_t i = 0; i < hull.size(); ++i) {
        Vector3d p1 = hull[i];
        Vector3d p2 = hull[(i + 1) % hull.size()];
        p1.z() = hull_z;
        p2.z() = hull_z;
        hull_l1.push_back(p1);
        hull_l2.push_back(p2);
      }
      if (is_ego_drone) {
        const int gid = resolveCenterGridId(expl_manager_->hgrid_, drone_state, ci);
        if (gid >= 0) ego_hull_grid_ids.insert(gid);
      }
    }

    const Vector4d hull_color =
        is_ego_drone ? drone_color : Vector4d(0.28, 0.33, 0.40, finished ? 0.05 : 0.22);
    vector<Vector3d> center_positions = drone_state.center_positions_;
    for (auto& point : center_positions) point.z() = center_z;
    visualization_->drawSpheres(center_positions, is_ego_drone ? 0.24 : 0.18, hull_color,
        "center_hull_centers", static_cast<int>(di) + 1, PlanningVisualization::PUBLISHER::HGRID);
    visualization_->drawLines(hull_l1, hull_l2, is_ego_drone ? 0.10 : 0.06, hull_color,
        "center_hulls", static_cast<int>(di) + 1, PlanningVisualization::PUBLISHER::HGRID);
  }

  // Draw assigned grid outlines that are not already represented by ego hull tasks.
  vector<int> ego_grid_outline_ids;
  ego_grid_outline_ids.reserve(ego_state.grid_ids_.size());
  for (const int gid : ego_state.grid_ids_) {
    if (ego_hull_grid_ids.find(gid) == ego_hull_grid_ids.end()) {
      ego_grid_outline_ids.push_back(gid);
    }
  }

  vector<Vector3d> pts1, pts2;
  expl_manager_->hgrid_->getGridMarker(ego_grid_outline_ids, pts1, pts2);
  for (auto& point : pts1) point.z() = kGridOutlineVisZ;
  for (auto& point : pts2) point.z() = kGridOutlineVisZ;
  visualization_->drawLines(pts1, pts2, 0.07, drone_color, "partition", 1,
      PlanningVisualization::PUBLISHER::HGRID);

  // Draw explored grid outlines as semi-transparent history.
  vector<int> explored_ids;
  expl_manager_->hgrid_->getExploredGrids(explored_ids);
  vector<Vector3d> exp_pts1, exp_pts2;
  expl_manager_->hgrid_->getGridMarker(explored_ids, exp_pts1, exp_pts2);
  for (auto& point : exp_pts1) point.z() = kExploredGridVisZ;
  for (auto& point : exp_pts2) point.z() = kExploredGridVisZ;
  visualization_->drawLines(exp_pts1, exp_pts2, 0.05, Vector4d(0.35, 0.35, 0.35, 0.10),
      "partition_explored", 2, PlanningVisualization::PUBLISHER::HGRID);

  // Draw current grid tour and B-spline trajectory.
  auto info = &planner_manager_->local_data_;
  auto grid_tour = expl_manager_->ed_->region_tour_;
  visualization_->drawLines(
      grid_tour, 0.1, drone_color, "grid_tour", 0, PlanningVisualization::PUBLISHER::HGRID);
  visualization_->drawBspline(
      info->position_traj_, 0.1, drone_color, false, 0.15, Vector4d(1, 1, 0, 1));

  // Clear stale next-viewpoint markers previously published on the hgrid topic.
  visualization_->drawSpheres(
      {}, 0.35, Vector4d(0, 0, 0, 1), "grid_tour", 1, PlanningVisualization::PUBLISHER::HGRID);
  visualization_->drawLines({}, {}, 0.08, Vector4d(0, 0, 0, 1), "grid_tour", 2,
      PlanningVisualization::PUBLISHER::HGRID);

  // Draw hgrid connectivity graph.
  vector<Vector3d> cg_unknown_s, cg_unknown_e, cg_free_s, cg_free_e, cg_portal_s, cg_portal_e,
      cg_active_unknown_nodes, cg_active_free_nodes, cg_inactive_unknown_nodes,
      cg_inactive_free_nodes, cg_edge_cost_pos;
  vector<double> cg_edge_costs;
  expl_manager_->hgrid_->getConnectivityGraphVisualizationData(1, cg_unknown_s, cg_unknown_e,
      cg_free_s, cg_free_e, cg_portal_s, cg_portal_e, cg_active_unknown_nodes,
      cg_active_free_nodes, cg_inactive_unknown_nodes, cg_inactive_free_nodes, cg_edge_cost_pos,
      cg_edge_costs);
  visualization_->drawConnectivityGraph(cg_unknown_s, cg_unknown_e, cg_free_s, cg_free_e,
      cg_portal_s, cg_portal_e, cg_active_unknown_nodes, cg_active_free_nodes,
      cg_inactive_unknown_nodes, cg_inactive_free_nodes, cg_edge_cost_pos, cg_edge_costs,
      "hgrid_connectivity_graph_d" + std::to_string(getId()),
      PlanningVisualization::PUBLISHER::CONNECTIVITY_GRAPH);
}

void C2ExplorationFSM::clearVisMarker() {
  for (int i = 0; i < 10; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i,
        PlanningVisualization::PUBLISHER::FRONTIER);
  }
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "frontier_tour", 0,
      PlanningVisualization::PUBLISHER::VIEWPOINT);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0,
      PlanningVisualization::PUBLISHER::VIEWPOINT);
  visualization_->drawSpheres(
      {}, 0.35, Vector4d(0, 0, 0, 1), "grid_tour", 1, PlanningVisualization::PUBLISHER::HGRID);
  visualization_->drawLines(
      {}, {}, 0.08, Vector4d(0, 0, 0, 1), "grid_tour", 2, PlanningVisualization::PUBLISHER::HGRID);
  visualization_->drawConnectivityGraph({}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {},
      "hgrid_connectivity_graph_d" + std::to_string(getId()),
      PlanningVisualization::PUBLISHER::CONNECTIVITY_GRAPH);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void C2ExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  if (state_ == WAIT_TRIGGER) {
    auto ed = expl_manager_->ed_;

    expl_manager_->updateFrontierStruct(fd_->odom_pos_);

    // cout << "odom: " << fd_->odom_pos_.transpose() << endl;
    vector<int> tmp_id1;
    vector<vector<int>> tmp_id2;
    // Optimize the global path with single-drone ATSP.
    bool status = expl_manager_->findGlobalTourOfGrid(
        { fd_->odom_pos_ }, { fd_->odom_vel_ }, tmp_id1, tmp_id2, true);
    // bool status = false;

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4), "frontier", i,
          PlanningVisualization::PUBLISHER::FRONTIER);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i,
          PlanningVisualization::PUBLISHER::FRONTIER);
    }
    if (status)
      visualize(2);
    else
      visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0,
          PlanningVisualization::PUBLISHER::VIEWPOINT);
  }
}

void C2ExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

  // // Debug traj planner
  // Eigen::Vector3d pos;
  // pos << msg->pose.position.x, msg->pose.position.y, 1;
  // expl_manager_->ed_->next_pos_ = pos;

  // Eigen::Vector3d dir = pos - fd_->odom_pos_;
  // expl_manager_->ed_->next_yaw_ = atan2(dir[1], dir[0]);
  // fd_->go_back_ = true;
  // transitState(PLAN_TRAJ, "triggerCallback");
  // return;

  if (state_ != WAIT_TRIGGER) return;

  fd_->expl_start_time_ = ros::Time(0.0);
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  fd_->start_pos_ = fd_->odom_pos_;
  ROS_WARN_STREAM("Start expl pos: " << fd_->start_pos_.transpose());

  if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
    transitState(PLAN_TRAJ, "triggerCallback");
  } else
    transitState(FINISH, "triggerCallback");
}

// Avoid collisions.
void C2ExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void C2ExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = ros::Time::now();
  }
}

// Broadcast own state periodically
void C2ExplorationFSM::droneStateTimerCallback(const ros::TimerEvent& e) {

  exploration_manager::DroneState msg;
  msg.drone_id = getId();

  auto& state = expl_manager_->ed_->swarm_state_[msg.drone_id - 1];

  if (fd_->static_state_) {
    state.pos_ = fd_->odom_pos_;
    state.vel_ = fd_->odom_vel_;
    state.yaw_ = fd_->odom_yaw_;
  } else {
    LocalTrajData* info = &planner_manager_->local_data_;
    double t_r = (ros::Time::now() - info->start_time_).toSec();
    state.pos_ = info->position_traj_.evaluateDeBoorT(t_r);
    state.vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
    state.yaw_ = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
  }
  state.stamp_ = ros::Time::now().toSec();
  state.recv_stamp_ = state.stamp_;
  msg.pos = { float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2]) };
  msg.vel = { float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2]) };
  msg.yaw = state.yaw_;
  for (auto& id : state.grid_ids_) {
    msg.grid_ids.push_back(id);
  }
  for (const auto& c : state.center_positions_) {
    geometry_msgs::Point p;
    p.x = c[0];
    p.y = c[1];
    p.z = c[2];
    msg.center_points.push_back(p);
  }
  for (const int cid : state.center_ids_) {
    msg.center_ids.push_back(cid);
  }
  msg.recent_attempt_time = state.recent_attempt_time_;
  msg.stamp = state.stamp_;

  drone_state_pub_.publish(msg);
}

// Receive other drones' states.
void C2ExplorationFSM::droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg) {
  // Update other drones' states
  if (msg->drone_id == getId()) return;

  auto& drone_state = expl_manager_->ed_->swarm_state_[msg->drone_id - 1];
  if (drone_state.stamp_ + 1e-4 >= msg->stamp) return;  // Avoid unordered msg

  const vector<Vector3d> old_center_positions = drone_state.center_positions_;
  const vector<int> old_center_ids = drone_state.center_ids_;
  const vector<int> old_center_grid_ids = drone_state.center_grid_ids_;
  const vector<vector<Vector3d>> old_center_hulls = drone_state.center_hulls_;

  drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
  drone_state.yaw_ = msg->yaw;
  drone_state.grid_ids_.clear();
  for (int i = 0; i < msg->grid_ids.size(); ++i) {
    drone_state.grid_ids_.push_back(msg->grid_ids[i]);
  }
  drone_state.center_positions_.clear();
  for (const auto& p : msg->center_points) {
    drone_state.center_positions_.emplace_back(p.x, p.y, p.z);
  }
  centersToCenterGridIds(
      expl_manager_->hgrid_, drone_state.center_positions_, drone_state.center_grid_ids_);
  drone_state.center_ids_.clear();
  for (const auto& cid : msg->center_ids) {
    drone_state.center_ids_.push_back(cid);
  }
  if (drone_state.center_ids_.empty() && !drone_state.center_positions_.empty()) {
    centersToCenterIds(
        expl_manager_->hgrid_, drone_state.center_positions_, drone_state.center_ids_, true);
  }
  preserveCenterHulls(expl_manager_->hgrid_, old_center_positions, old_center_ids,
      old_center_grid_ids, old_center_hulls, drone_state.center_positions_, drone_state.center_ids_,
      drone_state.center_grid_ids_, drone_state.center_hulls_);

  drone_state.stamp_ = msg->stamp;
  drone_state.recv_stamp_ = ros::Time::now().toSec();
  drone_state.recent_attempt_time_ = msg->recent_attempt_time;
}

// Start allocation optimization.
void C2ExplorationFSM::optTimerCallback(const ros::TimerEvent& e) {
  if (state_ == INIT) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: state=INIT");
    return;
  }
  auto ed = expl_manager_->ed_;
  if (state_ == FINISH) {
    if (ed->wait_opt_commit_ack_ || ed->wait_opt_response_) {
      clearHostOptSession(*ed);
    }
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: state=FINISH");
    return;
  }

  comm_graph_->getNeighborIds(expl_manager_->ed_->neighbor_ids_);

  // Select nearby drone not interacting with recently
  auto& states = expl_manager_->ed_->swarm_state_;
  auto& ego_state = states[getId() - 1];
  auto& neighbor = expl_manager_->ed_->neighbor_ids_;
  auto tn = ros::Time::now().toSec();
  optDebugWarn(fp_, "[Meeting Opt Debug] optTimer enter: drone_id=" + std::to_string(getId()) +
                        " state=" + std::to_string(int(state_)) +
                        " neighbors=" + idsToStr(neighbor) +
                        " wait_opt_response=" + std::to_string(int(ed->wait_opt_response_)));
  if (ed->wait_opt_commit_ack_) {
    handleHostWaitSession(*ed, fp_, opt_pub_, getId(), tn, true);
    return;
  }

  if (ed->wait_opt_response_) {
    handleHostWaitSession(*ed, fp_, opt_pub_, getId(), tn, false);
    return;
  }

  // In terminal/idle states, skip meeting-opt probing when neither self nor neighbors
  // hold any assigned grids/centers.
  if (state_ == IDLE || state_ == FINISH) {
    auto hasAssignedWork = [](const DroneState& s) {
      return !s.grid_ids_.empty() || !s.center_positions_.empty() || !s.center_ids_.empty();
    };
    const bool self_has_work = hasAssignedWork(ego_state);
    bool neighbor_has_work = false;
    for (const int id : neighbor) {
      if (id <= 0 || id > static_cast<int>(states.size()) || id == getId()) continue;
      if (hasAssignedWork(states[id - 1])) {
        neighbor_has_work = true;
        break;
      }
    }
    if (!self_has_work && !neighbor_has_work) {
      optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: idle/finish no assigned work");
      return;
    }
  }

  // Avoid frequent attempt
  if (tn - ego_state.recent_attempt_time_ < fp_->attempt_interval_) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: attempt interval gate, dt=" +
                          std::to_string(tn - ego_state.recent_attempt_time_) + " < " +
                          std::to_string(fp_->attempt_interval_));
    return;
  }
  if (tn - ego_state.recent_interact_time_ < fp_->meeting_opt_interval_) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: interact interval gate, dt=" +
                          std::to_string(tn - ego_state.recent_interact_time_) + " < " +
                          std::to_string(fp_->meeting_opt_interval_));
    return;
  }

  // Choose host from a globally consistent leader set (neighbor view + self),
  // independent of participant freshness filtering.
  vector<int> leader_ids = neighbor;
  pushUniqueId(leader_ids, getId());
  std::sort(leader_ids.begin(), leader_ids.end());
  leader_ids.erase(std::unique(leader_ids.begin(), leader_ids.end()), leader_ids.end());
  if (leader_ids.size() < 2) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: drone_id=" + std::to_string(getId()) +
                          " leader_ids<2 final_leader_ids=" + idsToStr(leader_ids));
    return;
  }
  optDebugWarn(fp_, "[Meeting Opt Debug] optTimer leader_ids decided: drone_id=" +
                        std::to_string(getId()) + " final_leader_ids=" + idsToStr(leader_ids));
  const int preferred_host_id = leader_ids.front();
  // Select host with deterministic id order.
  // Primary failover condition: preferred host stale.
  // Secondary failover condition: preferred host interval-gated for too long.
  const double failover_timeout =
      std::max(3.0 * fp_->state_fresh_timeout_, fp_->meeting_opt_interval_);
  const double gate_failover_timeout =
      std::max(std::max(2.0 * fp_->meeting_opt_interval_, 4.0 * fp_->attempt_interval_), 0.5);
  auto hostFreshEnough = [&](const int id) {
    if (id == getId()) return true;
    if (fp_->state_fresh_timeout_ <= 1e-6) return true;
    return (tn - states[id - 1].recv_stamp_) <= failover_timeout;
  };
  auto hostCanStartNow = [&](const int id) {
    const auto& s = states[id - 1];
    return (tn - s.recent_attempt_time_ >= fp_->attempt_interval_) &&
           (tn - s.recent_interact_time_ >= fp_->meeting_opt_interval_);
  };
  int host_id = preferred_host_id;
  if (!hostFreshEnough(preferred_host_id)) {
    ed->opt_preferred_gate_host_id_ = -1;
    ed->opt_preferred_gate_since_ = 0.0;
    host_id = -1;
    for (const int id : leader_ids) {
      if (hostFreshEnough(id) && hostCanStartNow(id)) {
        host_id = id;
        break;
      }
    }
    if (host_id < 0) {
      optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: no eligible host candidate");
      return;
    }
    if (host_id != preferred_host_id) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Host failover from d" << preferred_host_id
                                                                           << " to d" << host_id
                                                                           << " due to stale state");
      }
    }
  } else if (!hostCanStartNow(preferred_host_id)) {
    if (ed->opt_preferred_gate_host_id_ != preferred_host_id) {
      ed->opt_preferred_gate_host_id_ = preferred_host_id;
      ed->opt_preferred_gate_since_ = tn;
    }
    const double gate_duration = std::max(0.0, tn - ed->opt_preferred_gate_since_);
    if (gate_duration + 1e-6 < gate_failover_timeout) {
      optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: preferred host gated, keep host=d" +
                            std::to_string(preferred_host_id) +
                            " gated_for=" + std::to_string(gate_duration) +
                            " < failover=" + std::to_string(gate_failover_timeout));
      return;
    }

    host_id = -1;
    for (const int id : leader_ids) {
      if (id == preferred_host_id) continue;
      if (hostFreshEnough(id) && hostCanStartNow(id)) {
        host_id = id;
        break;
      }
    }
    if (host_id < 0) {
      optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: preferred host gated for "
                        "too long but no alternative host");
      return;
    }
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Host soft-failover from d"
                                        << preferred_host_id << " to d" << host_id
                                        << " due to sustained gate dt=" << gate_duration);
    }
  } else {
    ed->opt_preferred_gate_host_id_ = -1;
    ed->opt_preferred_gate_since_ = 0.0;
  }
  if (getId() != host_id) {
    optDebugWarn(fp_,
        "[Meeting Opt Debug] optTimer return: not host, selected host=" + std::to_string(host_id));
    return;
  }

  vector<int> drone_ids;
  drone_ids.reserve(neighbor.size());
  for (auto id : neighbor) {
    if (id == getId()) {
      drone_ids.push_back(id);
      continue;
    }
    if (fp_->state_fresh_timeout_ > 1e-6 &&
        tn - states[id - 1].recv_stamp_ > fp_->state_fresh_timeout_) {
      optDebugWarn(fp_, "[Meeting Opt Debug] skip participant " + std::to_string(id) +
                            ": stale recv dt=" + std::to_string(tn - states[id - 1].recv_stamp_));
      continue;
    }
    if (tn - states[id - 1].recent_interact_time_ < fp_->meeting_opt_interval_) {
      optDebugWarn(fp_,
          "[Meeting Opt Debug] skip participant " + std::to_string(id) +
              ": recent interact dt=" + std::to_string(tn - states[id - 1].recent_interact_time_));
      continue;
    }
    if (id - 1 < static_cast<int>(ed->opt_peer_backoff_until_.size())) {
      const double backoff_until = ed->opt_peer_backoff_until_[id - 1];
      if (backoff_until > tn + 1e-6) {
        const int streak = (id - 1 < static_cast<int>(ed->opt_peer_reject_streak_.size())) ?
                               ed->opt_peer_reject_streak_[id - 1] :
                               0;
        optDebugWarn(fp_, "[Meeting Opt Debug] skip participant " + std::to_string(id) +
                              ": reject-backoff remain=" + std::to_string(backoff_until - tn) +
                              " streak=" + std::to_string(streak));
        continue;
      }
      if (id - 1 < static_cast<int>(ed->opt_peer_reject_streak_.size()) &&
          ed->opt_peer_reject_streak_[id - 1] > 0) {
        clearPeerRejectBackoff(*ed, id);
      }
    }
    drone_ids.push_back(id);
  }
  pushUniqueId(drone_ids, getId());

  if (drone_ids.size() < 2) {
    optDebugWarn(fp_,
        "[Meeting Opt Debug] optTimer return: participant_ids<2, ids=" + idsToStr(drone_ids));
    return;
  }

  // Do global optimization with all participant drones
  vector<int> opt_ids;
  unordered_map<int, char> opt_ids_map;
  vector<int> state_task_ids;
  for (auto id : drone_ids) {
    collectStateTaskGridIds(states[id - 1], expl_manager_->hgrid_, state_task_ids);
    for (const auto gid : state_task_ids) {
      opt_ids_map[gid] = 1;
    }
  }
  for (auto pair : opt_ids_map) opt_ids.push_back(pair.first);

  if (fp_ && fp_->debug_opt_output_) {
    ROS_WARN_STREAM("[MeetingOpt] host=" << getId() << " participants=" << drone_ids.size()
                                      << " opt_grids=" << opt_ids.size());
  }
  if (opt_ids.empty()) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: opt_ids empty");
    return;
  }

  vector<Eigen::Vector3d> positions;
  vector<Eigen::Vector3d> velocities;
  vector<DroneState> participant_states;
  vector<vector<int>> first_ids_list, second_ids_list;
  positions.reserve(drone_ids.size());
  velocities.reserve(drone_ids.size());
  participant_states.reserve(drone_ids.size());
  first_ids_list.resize(drone_ids.size());
  second_ids_list.resize(drone_ids.size());

  for (size_t i = 0; i < drone_ids.size(); ++i) {
    const auto& state = states[drone_ids[i] - 1];
    positions.push_back(state.pos_);
    velocities.push_back(Eigen::Vector3d(0, 0, 0));
    participant_states.push_back(state);
    if (state_ != WAIT_TRIGGER) {
      vector<int> ids_only;
      collectStateTaskGridIds(state, expl_manager_->hgrid_, ids_only);
      expl_manager_->hgrid_->getConsistentGrid(
          ids_only, ids_only, first_ids_list[i], second_ids_list[i]);
    }
  }

  // Non-participant non-hull centers are protected by connectivity graph node id.
  // Split hull tasks do not have stable node-id identity, so they are protected by hull geometry.
  unordered_set<int> participant_set(drone_ids.begin(), drone_ids.end());
  unordered_set<int> blocked_node_id_set;
  vector<int> blocked_center_node_ids;
  vector<vector<Vector3d>> blocked_center_hulls;
  for (int did = 1; did <= static_cast<int>(states.size()); ++did) {
    if (participant_set.find(did) != participant_set.end()) continue;
    const auto& st = states[did - 1];
    for (int i = 0; i < static_cast<int>(st.center_ids_.size()); ++i) {
      const bool has_hull =
          (i < static_cast<int>(st.center_hulls_.size()) && !st.center_hulls_[i].empty());
      if (has_hull) continue;
      const int cid = st.center_ids_[i];
      if (cid < 0) continue;
      if (blocked_node_id_set.insert(cid).second) {
        blocked_center_node_ids.push_back(cid);
      }
    }
    for (const auto& hull : st.center_hulls_) {
      if (!hull.empty()) blocked_center_hulls.push_back(hull);
    }
  }

  const vector<DroneState>* ownership_states =
      (state_ == WAIT_TRIGGER) ? nullptr : &participant_states;
  if (state_ == WAIT_TRIGGER) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_THROTTLE(
          1.0, "[MeetingOpt] Skip participant-task ownership filter during WAIT_TRIGGER");
    }
  }
  C2ExplorationManager::AllocationRequest allocation_request;
  allocation_request.drone_positions = positions;
  allocation_request.drone_velocities = velocities;
  allocation_request.drone_ids = drone_ids;
  allocation_request.grid_ids = opt_ids;
  allocation_request.drone_states = ownership_states;
  allocation_request.blocked_center_node_ids = blocked_center_node_ids;
  allocation_request.blocked_center_hulls = blocked_center_hulls;

  C2ExplorationManager::AllocationResult allocation_result;
  expl_manager_->allocateTasks(allocation_request, allocation_result);
  vector<vector<Vector3d>>& alloc_centers = allocation_result.centers;
  vector<vector<vector<Vector3d>>>& alloc_center_hulls = allocation_result.center_hulls;
  if (alloc_centers.size() != drone_ids.size()) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: alloc_centers size mismatch, alloc=" +
                          std::to_string(alloc_centers.size()) +
                          " participants=" + std::to_string(drone_ids.size()));
    return;
  }
  if (alloc_center_hulls.size() != drone_ids.size()) {
    alloc_center_hulls.resize(drone_ids.size());
  }
  for (size_t i = 0; i < drone_ids.size(); ++i) {
    if (alloc_center_hulls[i].size() < alloc_centers[i].size()) {
      vector<vector<Vector3d>> recovered_hulls;
      centersToCenterHulls(expl_manager_->hgrid_, alloc_centers[i], recovered_hulls);
      alloc_center_hulls[i].resize(alloc_centers[i].size());
      for (int j = 0; j < static_cast<int>(alloc_centers[i].size()); ++j) {
        if (!alloc_center_hulls[i][j].empty()) continue;
        if (j < static_cast<int>(recovered_hulls.size())) {
          alloc_center_hulls[i][j] = recovered_hulls[j];
        }
      }
    }
  }

  vector<vector<int>> alloc_ids;
  alloc_ids.resize(drone_ids.size());
  for (size_t i = 0; i < drone_ids.size(); ++i) {
    centersToGridIds(expl_manager_->hgrid_, alloc_centers[i], alloc_ids[i], true);
  }

  vector<int> filtered_drone_ids;
  vector<vector<Vector3d>> filtered_alloc_centers;
  vector<vector<vector<Vector3d>>> filtered_alloc_center_hulls;
  vector<vector<int>> filtered_alloc_ids;
  vector<int> pruned_drone_ids;
  filtered_drone_ids.reserve(drone_ids.size());
  filtered_alloc_centers.reserve(drone_ids.size());
  filtered_alloc_center_hulls.reserve(drone_ids.size());
  filtered_alloc_ids.reserve(drone_ids.size());

  for (size_t i = 0; i < drone_ids.size(); ++i) {
    const int pid = drone_ids[i];
    const auto& st = states[pid - 1];
    const bool has_new_alloc = !alloc_ids[i].empty() || !alloc_centers[i].empty();
    const bool has_existing_alloc = !st.grid_ids_.empty() || !st.center_positions_.empty();
    const bool keep = (pid == getId()) || has_new_alloc || has_existing_alloc;
    if (!keep) {
      pruned_drone_ids.push_back(pid);
      continue;
    }
    filtered_drone_ids.push_back(pid);
    filtered_alloc_centers.push_back(alloc_centers[i]);
    filtered_alloc_center_hulls.push_back(alloc_center_hulls[i]);
    filtered_alloc_ids.push_back(alloc_ids[i]);
  }

  if (!pruned_drone_ids.empty() && fp_ && fp_->debug_opt_output_) {
    ROS_WARN_STREAM("[MeetingOpt] pruned empty participants count=" << pruned_drone_ids.size());
  }
  drone_ids.swap(filtered_drone_ids);
  alloc_centers.swap(filtered_alloc_centers);
  alloc_center_hulls.swap(filtered_alloc_center_hulls);
  alloc_ids.swap(filtered_alloc_ids);

  if (drone_ids.size() < 2) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: filtered participant_ids<2, ids=" +
                          idsToStr(drone_ids));
    return;
  }

  for (size_t i = 0; i < drone_ids.size(); ++i) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM("[MeetingOpt] Alloc to drone " << drone_ids[i]
                                                  << " centers=" << alloc_centers[i].size()
                                                  << " grids=" << alloc_ids[i].size());
    }
  }

  ed->opt_participant_ids_ = drone_ids;
  ed->opt_alloc_centers_ = alloc_centers;
  ed->opt_alloc_center_hulls_ = alloc_center_hulls;
  ed->opt_alloc_ids_ = alloc_ids;
  ed->opt_wait_ids_.clear();
  ed->opt_ack_ids_.clear();
  ed->opt_commit_ack_ids_.clear();
  forEachRemoteParticipant(
      drone_ids, getId(), [&](size_t /*idx*/, int id) { ed->opt_wait_ids_.push_back(id); });
  ed->meeting_opt_stamp_ = tn;
  ed->opt_wait_start_time_ = tn;
  ed->opt_last_send_time_ = tn;
  ed->opt_retry_count_ = 0;
  ed->wait_opt_response_ = true;
  ed->wait_opt_commit_ack_ = false;
  ed->opt_commit_wait_start_time_ = 0.0;
  ed->opt_commit_last_send_time_ = 0.0;
  ed->opt_commit_retry_count_ = 0;
  ego_state.recent_attempt_time_ = tn;

  if (fp_ && fp_->debug_opt_output_) {
    ROS_WARN_STREAM("[Meeting Opt] Host " << getId() << " sending proposal to "
                                          << ed->opt_wait_ids_.size() << " peers");
  }

  size_t host_idx = 0;
  if (!findParticipantIndex(drone_ids, getId(), host_idx)) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM(
          "[Meeting Opt] Host " << getId() << " cannot find self in participant_ids, abort");
    }
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: host not found in participant_ids");
    return;
  }
  forEachRemoteParticipant(drone_ids, getId(), [&](size_t i, int to_id) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM("[Meeting Opt] Host " << getId() << " send proposal to " << to_id
                                            << " ego=" << alloc_ids[host_idx].size()
                                            << " peer=" << alloc_ids[i].size());
    }
    sendMeetingOptAdaptive(opt_pub_, fp_, getId(), to_id, tn, kMeetingOptProposal,
        alloc_ids[host_idx], alloc_ids[i], alloc_centers[host_idx], alloc_centers[i],
        alloc_center_hulls[host_idx], alloc_center_hulls[i]);
  });
}

// Handle pairwise optimization requests from other drones.
void C2ExplorationFSM::optMsgCallback(const exploration_manager::MeetingOptConstPtr& msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) {
    optDebugTrace(fp_, "[Meeting Opt Debug] optMsg return: not for me/self msg from=" +
                           std::to_string(msg->from_drone_id) +
                           " to=" + std::to_string(msg->to_drone_id));
    return;
  }
  if (state_ == FINISH) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: state=FINISH");
    return;
  }

  const bool is_commit = (msg->phase == kMeetingOptCommit);
  const bool is_cancel = (msg->phase == kMeetingOptCancel);
  const bool is_finalize = (msg->phase == kMeetingOptFinalize);
  const bool is_proposal = (msg->phase == kMeetingOptProposal);
  if (!is_proposal && !is_commit && !is_cancel && !is_finalize) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Drone "
                                        << getId() << " drop opt from " << msg->from_drone_id
                                        << ": unknown phase=" << static_cast<int>(msg->phase));
    }
    return;
  }
  const int from_idx = msg->from_drone_id - 1;
  constexpr double kStampEps = 1e-4;
  auto ed = expl_manager_->ed_;
  auto& state1 = ed->swarm_state_[msg->from_drone_id - 1];  // Requesting drone.
  auto& state2 = ed->swarm_state_[getId() - 1];             // Local drone.

  if (is_proposal) {
    // Last proposal stamp processed from the peer.
    const double last_stamp = ed->meeting_opt_stamps_[from_idx];
    if (msg->stamp < last_stamp - kStampEps) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Drone " << getId() << " drop opt from "
                                                             << msg->from_drone_id
                                                             << ": old stamp=" << msg->stamp);
      }
      optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: proposal too old, stamp=" +
                            std::to_string(msg->stamp) + " last=" + std::to_string(last_stamp));
      return;
    }
    if (msg->stamp > last_stamp + kStampEps) {
      ed->meeting_opt_stamps_[from_idx] = msg->stamp;
      optDebugWarn(fp_, "[Meeting Opt Debug] proposal new stamp update: from=" +
                            std::to_string(msg->from_drone_id) +
                            " stamp=" + std::to_string(msg->stamp));
    } else {
      optDebugWarn(fp_, "[Meeting Opt Debug] proposal same-stamp retransmission: from=" +
                            std::to_string(msg->from_drone_id) +
                            " stamp=" + std::to_string(msg->stamp));
    }
  }

  if (fp_ && fp_->debug_opt_output_) {
    ROS_WARN_STREAM("[Meeting Opt] Drone "
                    << getId() << " recv opt from " << msg->from_drone_id << " stamp=" << msg->stamp
                    << " phase=" << static_cast<int>(msg->phase) << " ego_ids=" << msg->ego_ids.size()
                    << " other_ids=" << msg->other_ids.size());
  }

  if (is_finalize) {
    if (hasAppliedUnconfirmedTx(*ed, msg->from_drone_id, msg->stamp)) {
      clearAppliedUnconfirmedTx(*ed);
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM("[Meeting Opt] Drone " << getId() << " finalize tx from "
                                               << msg->from_drone_id << " stamp=" << msg->stamp);
      }
    }
    return;
  }

  if (is_proposal) {
    const double now = ros::Time::now().toSec();
    if (ed->wait_opt_response_ || ed->wait_opt_commit_ack_) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM("[Meeting Opt] Drone "
                        << getId() << " reject from " << msg->from_drone_id
                        << ": local host session busy wait_proposal=" << ed->wait_opt_response_
                        << " wait_commit=" << ed->wait_opt_commit_ack_);
      }
      sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
          msg->stamp, kMeetingOptProposalResponse, kMeetingOptReject);
      return;
    }
    if (ed->opt_pending_host_id_ > 0 && fp_->opt_wait_timeout_ > 1e-6 &&
        now - ed->opt_pending_recv_time_ > fp_->opt_wait_timeout_) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Drone "
                                          << getId()
                                          << " clear stale pending host=" << ed->opt_pending_host_id_
                                          << " dt=" << (now - ed->opt_pending_recv_time_));
      }
      clearOptPendingLock(*ed);
    }

    if (ed->opt_commit_apply_pending_ && fp_->opt_wait_timeout_ > 1e-6 &&
        now - ed->opt_commit_apply_time_ > 2.0 * fp_->opt_wait_timeout_) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Drone "
                                          << getId() << " clear stale unconfirmed commit tx: host="
                                          << ed->opt_commit_apply_host_id_
                                          << " stamp=" << ed->opt_commit_apply_stamp_);
      }
      clearAppliedUnconfirmedTx(*ed);
    }

    // If the same host starts a newer negotiation round, old unconfirmed commit is obsolete.
    if (ed->opt_commit_apply_pending_ && msg->from_drone_id == ed->opt_commit_apply_host_id_ &&
        msg->stamp > ed->opt_commit_apply_stamp_ + kStampEps) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM("[Meeting Opt] Drone "
                        << getId() << " clear obsolete unconfirmed commit tx from host "
                        << ed->opt_commit_apply_host_id_ << ": old_stamp="
                        << ed->opt_commit_apply_stamp_ << " new_stamp=" << msg->stamp);
      }
      clearAppliedUnconfirmedTx(*ed);
    }

    if (ed->opt_commit_apply_pending_) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM("[Meeting Opt] Drone "
                        << getId() << " reject proposal from " << msg->from_drone_id
                        << ": unconfirmed commit tx host=" << ed->opt_commit_apply_host_id_
                        << " stamp=" << ed->opt_commit_apply_stamp_);
      }
      sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
          msg->stamp, kMeetingOptProposalResponse, kMeetingOptReject);
      return;
    }

    // Idempotent retransmission of already accepted proposal from the same host.
    if (hasOptPendingFromHost(*ed, msg->from_drone_id, msg->stamp)) {
      sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
          msg->stamp, kMeetingOptProposalResponse, kMeetingOptAccept);
      optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: proposal retransmission acked");
      return;
    }

    // Single pending-host lock on receiver: reject other hosts until current pending session ends.
    if (ed->opt_pending_host_id_ > 0 && ed->opt_pending_host_id_ != msg->from_drone_id) {
      const double cooldown = registerPeerRejectBackoff(*ed, fp_, msg->from_drone_id, now);
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM("[Meeting Opt] Drone " << getId() << " reject from " << msg->from_drone_id
                                               << ": pending host=" << ed->opt_pending_host_id_
                                               << " stamp=" << ed->opt_pending_stamp_
                                               << " cooldown=" << cooldown);
      }
      sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
          msg->stamp, kMeetingOptProposalResponse, kMeetingOptReject);
      return;
    }

    // This drone just initiated a pairing attempt too.
    const double local_attempt_gap = ros::Time::now().toSec() - state2.recent_attempt_time_;
    if (local_attempt_gap < fp_->attempt_interval_) {
      // Just made another meeting opt attempt, should reject this attempt to avoid frequent changes
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN("[Meeting Opt] Drone %d reject from %d: recent attempt %.3f interval %.3f", getId(),
            msg->from_drone_id, local_attempt_gap, fp_->attempt_interval_);
      }
      sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
          msg->stamp, kMeetingOptProposalResponse, kMeetingOptReject);
      return;
    } else {
      int invalid_ego_centers = 0;
      int invalid_other_centers = 0;
      const bool centers_resolvable = isCenterSetResolvableOnLocalHGrid(expl_manager_->hgrid_,
                                          msg->ego_centers, 1.5, invalid_ego_centers) &&
                                      isCenterSetResolvableOnLocalHGrid(expl_manager_->hgrid_,
                                          msg->other_centers, 1.5, invalid_other_centers);
      if (!centers_resolvable) {
        if (fp_ && fp_->debug_opt_output_) {
          ROS_WARN("[Meeting Opt] Drone %d reject from %d: unresolved centers ego=%d other=%d",
              getId(), msg->from_drone_id, invalid_ego_centers, invalid_other_centers);
        }
        sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
            msg->stamp, kMeetingOptProposalResponse, kMeetingOptReject);
        optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: reject proposal due to unresolved "
                          "centers, response sent");
        return;
      }
    }

    // No opt attempt recently, and the grid info between drones are consistent, accept proposal.
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN("[Meeting Opt] Drone %d accept proposal from %d", getId(), msg->from_drone_id);
    }
    ed->opt_pending_host_id_ = msg->from_drone_id;
    ed->opt_pending_stamp_ = msg->stamp;
    ed->opt_pending_recv_time_ = now;
    sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
        msg->stamp, kMeetingOptProposalResponse, kMeetingOptAccept);
    optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: proposal accepted");
    return;
  }

  if (is_cancel) {
    const bool has_pending = hasOptPendingFromHost(*ed, msg->from_drone_id, msg->stamp);
    const bool has_unconfirmed = hasAppliedUnconfirmedTx(*ed, msg->from_drone_id, msg->stamp);
    if (!has_pending && !has_unconfirmed) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_DEBUG_STREAM_THROTTLE(1.0, "[Meeting Opt] Drone "
                                           << getId() << " cancel without pending from "
                                           << msg->from_drone_id << " stamp=" << msg->stamp);
      }
      optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: cancel without pending, from=" +
                            std::to_string(msg->from_drone_id));
      return;
    }
    if (has_pending) {
      clearOptPendingLock(*ed);
    }
    if (has_unconfirmed) {
      rollbackAppliedUnconfirmedTx(*ed, getId());
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM("[Meeting Opt] Drone " << getId() << " rollback tx from "
                                               << msg->from_drone_id << " stamp=" << msg->stamp);
      }
      // Only back off when an already-applied commit was rolled back.
      state2.recent_attempt_time_ = ros::Time::now().toSec();
    }
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM("[Meeting Opt] Drone " << getId() << " cancel pending from "
                                             << msg->from_drone_id << " stamp=" << msg->stamp);
    }
    state1.recent_interact_time_ = msg->stamp;
    optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: cancel handled and pending cleared");
    return;
  }

  // Commit phase
  if (hasAppliedUnconfirmedTx(*ed, msg->from_drone_id, msg->stamp)) {
    sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
        msg->stamp, kMeetingOptCommitAck, kMeetingOptAccept);
    return;
  }
  if (!hasOptPendingFromHost(*ed, msg->from_drone_id, msg->stamp)) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_DEBUG_STREAM_THROTTLE(1.0, "[Meeting Opt] Drone "
                                         << getId() << " commit without pending from "
                                         << msg->from_drone_id << " stamp=" << msg->stamp);
    }
    sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
        msg->stamp, kMeetingOptCommitAck, kMeetingOptReject);
    optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: commit without pending, from=" +
                          std::to_string(msg->from_drone_id));
    return;
  }
  vector<int> msg_ego_ids(msg->ego_ids.begin(), msg->ego_ids.end());
  vector<int> msg_other_ids(msg->other_ids.begin(), msg->other_ids.end());
  vector<Vector3d> msg_ego_centers, msg_other_centers;
  msgPointsToCenters(msg->ego_centers, msg_ego_centers);
  msgPointsToCenters(msg->other_centers, msg_other_centers);
  vector<vector<Vector3d>> msg_ego_hulls, msg_other_hulls;
  unpackHullsFromMsg(msg->ego_hull_offsets, msg->ego_hull_points, msg_ego_hulls);
  unpackHullsFromMsg(msg->other_hull_offsets, msg->other_hull_points, msg_other_hulls);
  if (msg_ego_hulls.size() < msg_ego_centers.size()) msg_ego_hulls.resize(msg_ego_centers.size());
  if (msg_other_hulls.size() < msg_other_centers.size()) {
    msg_other_hulls.resize(msg_other_centers.size());
  }

  const DroneState host_state_before = state1;
  const DroneState self_state_before = state2;
  applyAllocationToState(
      expl_manager_->hgrid_, msg_ego_ids, msg_ego_centers, msg_ego_hulls, state1);
  applyAllocationToState(
      expl_manager_->hgrid_, msg_other_ids, msg_other_centers, msg_other_hulls, state2);

  if (fp_ && fp_->debug_opt_output_) {
    ROS_WARN_STREAM("[Meeting Opt] Drone "
                    << getId() << " commit apply: state1(drone " << msg->from_drone_id << ") grids="
                    << state1.grid_ids_.size() << " centers=" << state1.center_ids_.size()
                    << " state2(drone " << getId() << ") grids=" << state2.grid_ids_.size()
                    << " centers=" << state2.center_ids_.size());
  }

  state1.recent_interact_time_ = msg->stamp;
  state2.recent_attempt_time_ = ros::Time::now().toSec();
  ed->reallocated_ = true;
  ed->opt_commit_backup_host_state_ = host_state_before;
  ed->opt_commit_backup_self_state_ = self_state_before;
  ed->opt_commit_backup_valid_ = true;
  ed->opt_commit_apply_pending_ = true;
  ed->opt_commit_apply_host_id_ = msg->from_drone_id;
  ed->opt_commit_apply_stamp_ = msg->stamp;
  ed->opt_commit_apply_time_ = ros::Time::now().toSec();

  clearOptPendingLock(*ed);
  sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
      msg->stamp, kMeetingOptCommitAck, kMeetingOptAccept);

  if (state_ == IDLE && !state2.grid_ids_.empty()) {
    transitState(PLAN_TRAJ, "optMsgCallback");
    ROS_WARN("Restart after opt!");
  }
}

//
void C2ExplorationFSM::optResMsgCallback(
    const exploration_manager::MeetingOptResponseConstPtr& msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) {
    optDebugTrace(fp_, "[Meeting Opt Debug] optRes return: not for me/self msg from=" +
                           std::to_string(msg->from_drone_id) +
                           " to=" + std::to_string(msg->to_drone_id));
    return;
  }
  if (state_ == FINISH) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optRes return: state=FINISH");
    return;
  }

  auto ed = expl_manager_->ed_;
  const bool is_proposal_res = (msg->phase == kMeetingOptProposalResponse);
  const bool is_commit_ack = (msg->phase == kMeetingOptCommitAck);
  if (!is_proposal_res && !is_commit_ack) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Host "
                                        << getId() << " drop opt_res from " << msg->from_drone_id
                                        << ": unknown phase=" << static_cast<int>(msg->phase));
    }
    return;
  }

  if (fabs(ed->meeting_opt_stamp_ - msg->stamp) > 1e-5) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM_THROTTLE(1.0,
          "[Meeting Opt] Host " << getId() << " drop opt_res from " << msg->from_drone_id
                                << ": wait_proposal=" << ed->wait_opt_response_
                                << " wait_commit=" << ed->wait_opt_commit_ack_
                                << " expect_stamp=" << ed->meeting_opt_stamp_ << " got=" << msg->stamp);
    }
    optDebugWarn(fp_, "[Meeting Opt Debug] optRes return: stamp/session mismatch, wait=" +
                          std::to_string(int(ed->wait_opt_response_)) + "/" +
                          std::to_string(int(ed->wait_opt_commit_ack_)) +
                          " expect=" + std::to_string(ed->meeting_opt_stamp_) +
                          " got=" + std::to_string(msg->stamp));
    return;
  }

  vector<double>& phase_stamps =
      is_commit_ack ? ed->meeting_opt_commit_res_stamps_ : ed->meeting_opt_res_stamps_;
  if (msg->stamp <= phase_stamps[msg->from_drone_id - 1] + 1e-4) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Host "
                                        << getId() << " drop opt_res from " << msg->from_drone_id
                                        << ": old stamp=" << msg->stamp
                                        << " phase=" << static_cast<int>(msg->phase));
    }
    optDebugTrace(fp_,
        "[Meeting Opt Debug] optRes return: old stamp from=" + std::to_string(msg->from_drone_id) +
            " stamp=" + std::to_string(msg->stamp));
    return;
  }
  phase_stamps[msg->from_drone_id - 1] = msg->stamp;

  if (is_proposal_res) {
    if (!ed->wait_opt_response_ || ed->wait_opt_commit_ack_) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Host " << getId() << " drop proposal-res from "
                                                            << msg->from_drone_id
                                                            << ": no proposal wait session");
      }
      return;
    }

    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM("[Meeting Opt] Host "
                      << getId() << " recv proposal-res from " << msg->from_drone_id
                      << " status=" << msg->status << " stamp=" << msg->stamp
                      << " wait=" << ed->opt_wait_ids_.size() << " ack=" << ed->opt_ack_ids_.size());
    }

    if (msg->status != kMeetingOptAccept) {
      const double now = ros::Time::now().toSec();
      const double cooldown = registerPeerRejectBackoff(*ed, fp_, msg->from_drone_id, now);
      if (msg->from_drone_id > 0 &&
          msg->from_drone_id <= static_cast<int>(ed->swarm_state_.size())) {
        ed->swarm_state_[msg->from_drone_id - 1].recent_interact_time_ = now;
      }
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM("[Meeting Opt] Host " << getId() << " cooldown peer " << msg->from_drone_id
                                              << " after reject: hold=" << cooldown << "s");
        ROS_WARN("[Meeting Opt] Abort: drone %d rejected proposal", msg->from_drone_id);
      }
      forEachRemoteParticipant(ed->opt_participant_ids_, getId(), [&](size_t /*idx*/, int to_id) {
        if (fp_ && fp_->debug_opt_output_) {
          ROS_WARN_STREAM("[Meeting Opt] Host " << getId() << " cancel to " << to_id
                                                << " phase=2 stamp=" << ed->meeting_opt_stamp_);
        }
        sendMeetingOptAdaptive(
            opt_pub_, fp_, getId(), to_id, ed->meeting_opt_stamp_, kMeetingOptCancel);
      });
      clearHostOptSession(*ed);
      return;
    }

    clearPeerRejectBackoff(*ed, msg->from_drone_id);

    pushUniqueId(ed->opt_ack_ids_, msg->from_drone_id);
    if (ed->opt_ack_ids_.size() < ed->opt_wait_ids_.size()) return;

    size_t host_idx = 0;
    if (!findParticipantIndex(ed->opt_participant_ids_, getId(), host_idx)) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "[Meeting Opt] Host " << getId() << " clear invalid proposal wait session");
      }
      clearHostOptSession(*ed);
      return;
    }

    forEachRemoteParticipant(ed->opt_participant_ids_, getId(), [&](size_t i, int to_id) {
      if (fp_ && fp_->debug_opt_output_) {
        ROS_WARN_STREAM("[Meeting Opt] Host " << getId() << " send commit to " << to_id
                                              << " ego=" << ed->opt_alloc_ids_[host_idx].size()
                                              << " peer=" << ed->opt_alloc_ids_[i].size());
      }
      sendMeetingOptAdaptive(opt_pub_, fp_, getId(), to_id, ed->meeting_opt_stamp_,
          kMeetingOptCommit, ed->opt_alloc_ids_[host_idx], ed->opt_alloc_ids_[i],
          ed->opt_alloc_centers_[host_idx], ed->opt_alloc_centers_[i],
          ed->opt_alloc_center_hulls_[host_idx], ed->opt_alloc_center_hulls_[i]);
    });

    const double now = ros::Time::now().toSec();
    ed->wait_opt_response_ = false;
    ed->wait_opt_commit_ack_ = true;
    ed->opt_commit_ack_ids_.clear();
    ed->opt_commit_wait_start_time_ = now;
    ed->opt_commit_last_send_time_ = now;
    ed->opt_commit_retry_count_ = 0;
    return;
  }

  // Commit ACK phase
  if (!ed->wait_opt_commit_ack_ || ed->wait_opt_response_) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Host " << getId() << " drop commit-ack from "
                                                          << msg->from_drone_id
                                                          << ": no commit wait session");
    }
    return;
  }

  if (fp_ && fp_->debug_opt_output_) {
    ROS_WARN_STREAM("[Meeting Opt] Host " << getId() << " recv commit-ack from " << msg->from_drone_id
                                          << " status=" << msg->status << " stamp=" << msg->stamp
                                          << " wait=" << ed->opt_wait_ids_.size()
                                          << " ack=" << ed->opt_commit_ack_ids_.size());
  }

  if (msg->status != kMeetingOptAccept) {
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN("[Meeting Opt] Abort: drone %d reject/invalid commit ack", msg->from_drone_id);
    }
    forEachRemoteParticipant(ed->opt_participant_ids_, getId(), [&](size_t /*idx*/, int to_id) {
      sendMeetingOptAdaptive(opt_pub_, fp_, getId(), to_id, ed->meeting_opt_stamp_,
          kMeetingOptCancel);
    });
    clearHostOptSession(*ed);
    return;
  }

  pushUniqueId(ed->opt_commit_ack_ids_, msg->from_drone_id);
  if (ed->opt_commit_ack_ids_.size() < ed->opt_wait_ids_.size()) return;

  // All commit ACKs received, apply allocation locally.
  for (size_t i = 0; i < ed->opt_participant_ids_.size(); ++i) {
    auto& state = ed->swarm_state_[ed->opt_participant_ids_[i] - 1];
    applyAllocationToState(expl_manager_->hgrid_, ed->opt_alloc_ids_[i], ed->opt_alloc_centers_[i],
        ed->opt_alloc_center_hulls_[i], state);
    state.recent_interact_time_ = msg->stamp;
    if (fp_ && fp_->debug_opt_output_) {
      ROS_WARN_STREAM("[Meeting Opt] Commit drone " << ed->opt_participant_ids_[i]
                                                    << " grids=" << state.grid_ids_.size()
                                                    << " centers=" << state.center_ids_.size());
    }
  }
  ed->reallocated_ = true;

  size_t host_idx = 0;
  if (findParticipantIndex(ed->opt_participant_ids_, getId(), host_idx)) {
    forEachRemoteParticipant(ed->opt_participant_ids_, getId(), [&](size_t i, int to_id) {
      sendMeetingOptAdaptive(opt_pub_, fp_, getId(), to_id, ed->meeting_opt_stamp_,
          kMeetingOptFinalize, ed->opt_alloc_ids_[host_idx], ed->opt_alloc_ids_[i],
          ed->opt_alloc_centers_[host_idx], ed->opt_alloc_centers_[i],
          ed->opt_alloc_center_hulls_[host_idx], ed->opt_alloc_center_hulls_[i]);
    });
  }
  clearHostOptSession(*ed);

  auto& state1 = ed->swarm_state_[getId() - 1];
  if (state_ == IDLE && !state1.grid_ids_.empty()) {
    transitState(PLAN_TRAJ, "optResMsgCallback");
    ROS_WARN("Restart after opt!");
  }
}

void C2ExplorationFSM::swarmTrajCallback(const bspline::BsplineConstPtr& msg) {
  // Get newest trajs from other drones, for inter-drone collision avoidance
  auto& sdat = planner_manager_->swarm_traj_data_;

  // Ignore self trajectory
  if (msg->drone_id == sdat.drone_id_) return;

  // Ignore outdated trajectory
  if (sdat.receive_flags_[msg->drone_id - 1] == true &&
      msg->start_time.toSec() <= sdat.swarm_trajs_[msg->drone_id - 1].start_time_ + 1e-3)
    return;

  // Convert the msg to B-spline
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  // // Transform of drone's basecoor, optional step (skip if use swarm_pilot)
  // Eigen::Vector4d tf;
  // planner_manager_->edt_environment_->sdf_map_->getBaseCoor(msg->drone_id, tf);
  // double yaw = tf[3];
  // Eigen::Matrix3d rot;
  // rot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  // Eigen::Vector3d trans = tf.head<3>();
  // for (int i = 0; i < pos_pts.rows(); ++i) {
  //   Eigen::Vector3d tmp = pos_pts.row(i);
  //   tmp = rot * tmp + trans;
  //   pos_pts.row(i) = tmp;
  // }

  sdat.swarm_trajs_[msg->drone_id - 1].setUniformBspline(pos_pts, msg->order, 0.1);
  sdat.swarm_trajs_[msg->drone_id - 1].setKnot(knots);
  sdat.swarm_trajs_[msg->drone_id - 1].start_time_ = msg->start_time.toSec();
  sdat.receive_flags_[msg->drone_id - 1] = true;

  if (state_ == EXEC_TRAJ) {
    // Check collision with received trajectory
    if (!planner_manager_->checkSwarmCollision(msg->drone_id)) {
      ROS_ERROR("Drone %d collide with drone %d.", sdat.drone_id_, msg->drone_id);
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "swarmTrajCallback");
    }
  }
}

void C2ExplorationFSM::swarmTrajTimerCallback(const ros::TimerEvent& e) {
  // Broadcast newest traj of this drone to others
  if (state_ == EXEC_TRAJ) {
    swarm_traj_pub_.publish(fd_->newest_traj_);

  } else if (state_ == WAIT_TRIGGER) {
    // Publish a virtual traj at current pose, to avoid collision
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = ros::Time::now();
    bspline.traj_id = planner_manager_->local_data_.traj_id_;

    Eigen::MatrixXd pos_pts(4, 3);
    for (int i = 0; i < 4; ++i) pos_pts.row(i) = fd_->odom_pos_.transpose();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    NonUniformBspline tmp(pos_pts, planner_manager_->pp_.bspline_degree_, 1.0);
    Eigen::VectorXd knots = tmp.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    bspline.drone_id = expl_manager_->ep_->drone_id_;
    swarm_traj_pub_.publish(bspline);
  }
}

void C2ExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  ROS_INFO_STREAM("[" + pos_call + "]: Drone "
                  << getId()
                  << " from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
}

int C2ExplorationFSM::resolveCenterGridId(
    const shared_ptr<HGrid>& hgrid, const DroneState& drone_state, const int center_idx) {
  if (center_idx >= 0 && center_idx < static_cast<int>(drone_state.center_grid_ids_.size()) &&
      drone_state.center_grid_ids_[center_idx] >= 0) {
    return drone_state.center_grid_ids_[center_idx];
  }
  if (center_idx >= 0 && center_idx < static_cast<int>(drone_state.center_positions_.size())) {
    int gid = -1;
    if (hgrid->getGridIdByCenterPos(drone_state.center_positions_[center_idx], gid, 1.0)) {
      return gid;
    }
  }
  return -1;
}

std::string C2ExplorationFSM::idsToStr(const vector<int>& ids) {
  std::ostringstream ss;
  ss << "[";
  for (size_t i = 0; i < ids.size(); ++i) {
    if (i) ss << " ";
    ss << static_cast<int>(ids[i]);
  }
  ss << "]";
  return ss.str();
}

void C2ExplorationFSM::centersToGridIds(const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& centers,
    vector<int>& grid_ids, bool dedup) {
  grid_ids.clear();
  if (!hgrid) return;
  unordered_set<int> seen;
  for (const auto& c : centers) {
    int gid = -1;
    if (!hgrid->getGridIdByCenterPos(c, gid)) continue;
    if (!dedup || seen.insert(gid).second) {
      grid_ids.push_back(gid);
    }
  }
}

void C2ExplorationFSM::centersToCenterIds(const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& centers,
    vector<int>& center_ids, bool dedup) {
  (void)dedup;
  center_ids.clear();
  center_ids.reserve(centers.size());
  if (!hgrid) {
    center_ids.resize(centers.size(), -1);
    return;
  }
  const auto& ids = hgrid->getLastCostMatrixCenterIds();
  for (const auto& c : centers) {
    int idx = -1;
    int cid = -1;
    if (hgrid->findCenterIndexByPos(c, idx, 1.0) && idx >= 0 &&
        idx < static_cast<int>(ids.size())) {
      cid = ids[idx];
    }
    center_ids.push_back(cid);
  }
}

void C2ExplorationFSM::centersToCenterGridIds(
    const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& centers, vector<int>& center_grid_ids) {
  center_grid_ids.clear();
  center_grid_ids.reserve(centers.size());
  if (!hgrid) {
    center_grid_ids.resize(centers.size(), -1);
    return;
  }
  for (const auto& c : centers) {
    int gid = -1;
    if (hgrid->getGridIdByCenterPos(c, gid, 1.0)) {
      center_grid_ids.push_back(gid);
    } else {
      center_grid_ids.push_back(-1);
    }
  }
}

void C2ExplorationFSM::centersToCenterHulls(const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& centers,
    vector<vector<Vector3d>>& center_hulls) {
  center_hulls.clear();
  if (!hgrid) return;
  const auto& center_hulls_all = hgrid->getLastCostMatrixCenterHulls();

  center_hulls.reserve(centers.size());
  for (const auto& c : centers) {
    vector<Vector3d> hull;
    int idx = -1;
    // 1.0m tolerance keeps matching robust after short map updates.
    if (hgrid->findCenterIndexByPos(c, idx, 1.0) && idx >= 0 &&
        idx < static_cast<int>(center_hulls_all.size())) {
      hull = center_hulls_all[idx];
    }
    center_hulls.push_back(hull);
  }
}

void C2ExplorationFSM::preserveCenterHulls(const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& old_centers,
    const vector<int>& old_center_ids, const vector<int>& old_center_grid_ids,
    const vector<vector<Vector3d>>& old_center_hulls, const vector<Vector3d>& new_centers,
    const vector<int>& new_center_ids, const vector<int>& new_center_grid_ids,
    vector<vector<Vector3d>>& new_center_hulls) {
  (void)old_center_ids;
  (void)new_center_ids;
  new_center_hulls.clear();
  new_center_hulls.resize(new_centers.size());

  unordered_set<int> used_old;
  for (int i = 0; i < static_cast<int>(new_centers.size()); ++i) {
    const int gid = (i < static_cast<int>(new_center_grid_ids.size())) ? new_center_grid_ids[i] : -1;
    int best_idx = -1;
    double best_d2 = 1.0;
    for (int j = 0; j < static_cast<int>(old_centers.size()); ++j) {
      if (used_old.find(j) != used_old.end()) continue;
      if (j >= static_cast<int>(old_center_hulls.size()) || old_center_hulls[j].empty()) continue;
      const int old_gid =
          (j < static_cast<int>(old_center_grid_ids.size())) ? old_center_grid_ids[j] : -1;
      if (gid >= 0 && old_gid >= 0 && gid != old_gid) continue;
      const double d2 = (new_centers[i] - old_centers[j]).squaredNorm();
      if (d2 < best_d2) {
        best_d2 = d2;
        best_idx = j;
      }
    }
    if (best_idx >= 0) {
      used_old.insert(best_idx);
      new_center_hulls[i] = old_center_hulls[best_idx];
      continue;
    }

    if (hgrid) {
      vector<Vector3d> hull;
      if (hgrid->getCenterHullByPos(new_centers[i], hull, 1.0) && !hull.empty()) {
        new_center_hulls[i] = hull;
      }
    }
  }
}

void C2ExplorationFSM::packHullsToMsg(const vector<vector<Vector3d>>& hulls, vector<int32_t>& offsets,
    vector<geometry_msgs::Point>& points) {
  offsets.clear();
  points.clear();
  offsets.reserve(hulls.size() + 1);
  offsets.push_back(0);
  for (const auto& hull : hulls) {
    for (const auto& v : hull) {
      geometry_msgs::Point p;
      p.x = v.x();
      p.y = v.y();
      p.z = v.z();
      points.push_back(p);
    }
    offsets.push_back(static_cast<int32_t>(points.size()));
  }
}

void C2ExplorationFSM::unpackHullsFromMsg(const vector<int32_t>& offsets, const vector<geometry_msgs::Point>& points,
    vector<vector<Vector3d>>& hulls) {
  hulls.clear();
  if (offsets.empty()) return;

  int prev = 0;
  for (size_t i = 1; i < offsets.size(); ++i) {
    int cur = offsets[i];
    if (cur < prev) {
      hulls.clear();
      return;
    }
    if (cur > static_cast<int>(points.size())) {
      hulls.clear();
      return;
    }
    vector<Vector3d> hull;
    hull.reserve(cur - prev);
    for (int j = prev; j < cur; ++j) {
      hull.emplace_back(points[j].x, points[j].y, points[j].z);
    }
    hulls.push_back(hull);
    prev = cur;
  }
}

void C2ExplorationFSM::uniqueIds(const vector<int>& in, vector<int>& out) {
  out.clear();
  unordered_set<int> seen;
  for (const int id : in) {
    if (id < 0) continue;
    if (seen.insert(id).second) out.push_back(id);
  }
}

void C2ExplorationFSM::applyAllocationToState(const shared_ptr<HGrid>& hgrid, const vector<int>& ids,
    const vector<Vector3d>& centers, const vector<vector<Vector3d>>& center_hulls,
    DroneState& state) {
  state.center_positions_ = centers;
  state.center_hulls_ = center_hulls;
  if (state.center_hulls_.size() < state.center_positions_.size()) {
    state.center_hulls_.resize(state.center_positions_.size());
  }
  centersToCenterGridIds(hgrid, state.center_positions_, state.center_grid_ids_);
  // Keep solver-provided grid ids as authoritative to avoid remap drift after movement.
  uniqueIds(ids, state.grid_ids_);
  state.center_ids_.clear();
  state.center_ids_.reserve(state.center_positions_.size());
  const auto* last_center_ids = hgrid ? &hgrid->getLastCostMatrixCenterIds() : nullptr;
  for (int i = 0; i < static_cast<int>(state.center_positions_.size()); ++i) {
    int cid = -1;
    if (hgrid && last_center_ids != nullptr) {
      int idx = -1;
      if (hgrid->findCenterIndexByPos(state.center_positions_[i], idx, 1.0) &&
          idx >= 0 && idx < static_cast<int>(last_center_ids->size())) {
        cid = (*last_center_ids)[idx];
      }
    }
    state.center_ids_.push_back(cid);
  }
}

void C2ExplorationFSM::msgPointsToCenters(const vector<geometry_msgs::Point>& points, vector<Vector3d>& centers) {
  centers.clear();
  centers.reserve(points.size());
  for (const auto& p : points) {
    centers.emplace_back(p.x, p.y, p.z);
  }
}

void C2ExplorationFSM::centersToMsgPoints(const vector<Vector3d>& centers, vector<geometry_msgs::Point>& points) {
  points.clear();
  points.reserve(centers.size());
  for (const auto& c : centers) {
    geometry_msgs::Point p;
    p.x = c.x();
    p.y = c.y();
    p.z = c.z();
    points.push_back(p);
  }
}

void C2ExplorationFSM::fillMeetingOptPayload(const vector<int>& ego_ids, const vector<int>& other_ids,
    const vector<Vector3d>& ego_centers, const vector<Vector3d>& other_centers,
    const vector<vector<Vector3d>>& ego_hulls, const vector<vector<Vector3d>>& other_hulls,
    exploration_manager::MeetingOpt& opt) {
  opt.ego_ids.clear();
  opt.other_ids.clear();
  for (const int id : ego_ids) opt.ego_ids.push_back(id);
  for (const int id : other_ids) opt.other_ids.push_back(id);
  centersToMsgPoints(ego_centers, opt.ego_centers);
  centersToMsgPoints(other_centers, opt.other_centers);
  packHullsToMsg(ego_hulls, opt.ego_hull_offsets, opt.ego_hull_points);
  packHullsToMsg(other_hulls, opt.other_hull_offsets, opt.other_hull_points);
}

void C2ExplorationFSM::clearOptPendingLock(ExplorationData& ed) {
  ed.opt_pending_host_id_ = -1;
  ed.opt_pending_stamp_ = 0.0;
  ed.opt_pending_recv_time_ = 0.0;
}

bool C2ExplorationFSM::hasOptPendingFromHost(const ExplorationData& ed, const int host_id, const double stamp) {
  if (host_id <= 0 || ed.opt_pending_host_id_ != host_id) return false;
  return std::fabs(ed.opt_pending_stamp_ - stamp) <= 1e-4;
}

bool C2ExplorationFSM::hasAppliedUnconfirmedTx(const ExplorationData& ed, const int host_id, const double stamp) {
  if (!ed.opt_commit_apply_pending_) return false;
  if (host_id <= 0 || ed.opt_commit_apply_host_id_ != host_id) return false;
  return std::fabs(ed.opt_commit_apply_stamp_ - stamp) <= 1e-4;
}

void C2ExplorationFSM::clearAppliedUnconfirmedTx(ExplorationData& ed) {
  ed.opt_commit_apply_pending_ = false;
  ed.opt_commit_apply_host_id_ = -1;
  ed.opt_commit_apply_stamp_ = 0.0;
  ed.opt_commit_apply_time_ = 0.0;
  ed.opt_commit_backup_valid_ = false;
}

void C2ExplorationFSM::rollbackAppliedUnconfirmedTx(ExplorationData& ed, const int self_id) {
  if (!ed.opt_commit_apply_pending_ || !ed.opt_commit_backup_valid_) {
    clearAppliedUnconfirmedTx(ed);
    return;
  }
  const int host_idx = ed.opt_commit_apply_host_id_ - 1;
  const int self_idx = self_id - 1;
  if (host_idx >= 0 && host_idx < static_cast<int>(ed.swarm_state_.size())) {
    ed.swarm_state_[host_idx] = ed.opt_commit_backup_host_state_;
  }
  if (self_idx >= 0 && self_idx < static_cast<int>(ed.swarm_state_.size())) {
    ed.swarm_state_[self_idx] = ed.opt_commit_backup_self_state_;
  }
  ed.reallocated_ = true;
  clearAppliedUnconfirmedTx(ed);
}

void C2ExplorationFSM::clearHostOptSession(ExplorationData& ed) {
  ed.wait_opt_response_ = false;
  ed.wait_opt_commit_ack_ = false;
  ed.opt_wait_start_time_ = 0.0;
  ed.opt_last_send_time_ = 0.0;
  ed.opt_retry_count_ = 0;
  ed.opt_commit_wait_start_time_ = 0.0;
  ed.opt_commit_last_send_time_ = 0.0;
  ed.opt_commit_retry_count_ = 0;
  ed.opt_wait_ids_.clear();
  ed.opt_ack_ids_.clear();
  ed.opt_commit_ack_ids_.clear();
  ed.opt_alloc_ids_.clear();
  ed.opt_alloc_centers_.clear();
  ed.opt_alloc_center_hulls_.clear();
  ed.opt_participant_ids_.clear();
}

void C2ExplorationFSM::collectStateTaskGridIds(
    const DroneState& st, const shared_ptr<HGrid>& hgrid, vector<int>& out) {
  out.clear();
  out.reserve(st.grid_ids_.size() + st.center_grid_ids_.size() + st.center_positions_.size());
  unordered_set<int> seen;
  auto add_gid = [&](const int gid) {
    if (gid < 0) return;
    if (seen.insert(gid).second) out.push_back(gid);
  };
  for (const int gid : st.grid_ids_) add_gid(gid);
  for (const int gid : st.center_grid_ids_) add_gid(gid);
  if (!hgrid) return;
  for (const auto& c : st.center_positions_) {
    int gid = -1;
    if (hgrid->getGridIdByCenterPos(c, gid, 1.0)) {
      add_gid(gid);
    }
  }
}

void C2ExplorationFSM::clearPeerRejectBackoff(ExplorationData& ed, const int peer_id) {
  if (peer_id <= 0) return;
  const int idx = peer_id - 1;
  if (idx < 0 || idx >= static_cast<int>(ed.opt_peer_backoff_until_.size())) return;
  ed.opt_peer_backoff_until_[idx] = 0.0;
  if (idx < static_cast<int>(ed.opt_peer_reject_streak_.size())) {
    ed.opt_peer_reject_streak_[idx] = 0;
  }
}

double C2ExplorationFSM::registerPeerRejectBackoff(
    ExplorationData& ed, const shared_ptr<FSMParam>& fp, const int peer_id, const double now) {
  if (!fp || fp->opt_reject_backoff_ <= 1e-6 || peer_id <= 0) {
    return 0.0;
  }
  const int idx = peer_id - 1;
  if (idx < 0 || idx >= static_cast<int>(ed.opt_peer_backoff_until_.size())) return 0.0;

  int streak = 0;
  if (idx < static_cast<int>(ed.opt_peer_reject_streak_.size())) {
    streak = ed.opt_peer_reject_streak_[idx];
  }
  if (ed.opt_peer_backoff_until_[idx] + 1e-6 < now) {
    streak = 0;
  }
  streak = std::min(streak + 1, 8);
  if (idx < static_cast<int>(ed.opt_peer_reject_streak_.size())) {
    ed.opt_peer_reject_streak_[idx] = streak;
  }

  double duration = fp->opt_reject_backoff_ * std::pow(1.5, std::max(0, streak - 1));
  if (fp->opt_reject_backoff_max_ > 1e-6) {
    duration = std::min(duration, fp->opt_reject_backoff_max_);
  }
  ed.opt_peer_backoff_until_[idx] = std::max(ed.opt_peer_backoff_until_[idx], now + duration);
  return std::max(0.0, ed.opt_peer_backoff_until_[idx] - now);
}

bool C2ExplorationFSM::containsId(const vector<int>& ids, const int id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

bool C2ExplorationFSM::pushUniqueId(vector<int>& ids, const int id) {
  if (containsId(ids, id)) return false;
  ids.push_back(id);
  return true;
}

void C2ExplorationFSM::forEachRemoteParticipant(const vector<int>& participant_ids,
    const int self_id, const std::function<void(size_t, int)>& fn) {
  for (size_t idx = 0; idx < participant_ids.size(); ++idx) {
    const int peer_id = participant_ids[idx];
    if (peer_id == self_id) continue;
    fn(idx, peer_id);
  }
}

bool C2ExplorationFSM::findParticipantIndex(const vector<int>& participant_ids, const int drone_id, size_t& idx) {
  auto it = std::find(participant_ids.begin(), participant_ids.end(), drone_id);
  if (it == participant_ids.end()) return false;
  idx = static_cast<size_t>(it - participant_ids.begin());
  return true;
}

bool C2ExplorationFSM::isCenterSetResolvableOnLocalHGrid(const shared_ptr<HGrid>& hgrid,
    const vector<geometry_msgs::Point>& points, const double tol, int& invalid_count) {
  invalid_count = 0;
  if (!hgrid) return false;
  for (const auto& p : points) {
    int gid = -1;
    if (!hgrid->getGridIdByCenterPos(Vector3d(p.x, p.y, p.z), gid, tol)) {
      ++invalid_count;
    }
  }
  return invalid_count == 0;
}

void C2ExplorationFSM::optDebugWarn(const shared_ptr<FSMParam>& fp, const std::string& msg) {
  if (!fp || !fp->debug_opt_output_) return;
  ROS_WARN_STREAM(msg);
}

void C2ExplorationFSM::optDebugTrace(const shared_ptr<FSMParam>& fp, const std::string& msg) {
  if (!fp || !fp->debug_opt_output_) return;
  ROS_DEBUG_STREAM_THROTTLE(0.2, msg);
}

void C2ExplorationFSM::publishRepeated(
    ros::Publisher& pub, const exploration_manager::MeetingOpt& msg, const int repeat_send_num) {
  for (int i = 0; i < repeat_send_num; ++i) pub.publish(msg);
}

void C2ExplorationFSM::publishRepeated(ros::Publisher& pub,
    const exploration_manager::MeetingOptResponse& msg, const int repeat_send_num) {
  for (int i = 0; i < repeat_send_num; ++i) pub.publish(msg);
}

int C2ExplorationFSM::getMeetingOptRepeatNum(const shared_ptr<FSMParam>& fp, const int8_t phase) {
  const int base_repeat = fp ? std::max(1, fp->repeat_send_num_) : 1;
  if (!fp) return base_repeat;
  if (phase == kMeetingOptProposal) return std::max(base_repeat, 2);
  if (phase == kMeetingOptCommit || phase == kMeetingOptCancel ||
      phase == kMeetingOptFinalize) {
    return std::max(base_repeat, 3);
  }
  return base_repeat;
}

int C2ExplorationFSM::getMeetingOptResRepeatNum(const shared_ptr<FSMParam>& fp, const int8_t phase) {
  const int base_repeat = fp ? std::max(1, fp->repeat_send_num_) : 1;
  if (!fp) return base_repeat;
  if (phase == kMeetingOptProposalResponse) return std::max(base_repeat, 2);
  if (phase == kMeetingOptCommitAck) return std::max(base_repeat, 3);
  return base_repeat;
}

void C2ExplorationFSM::sendMeetingOptAdaptive(ros::Publisher& opt_pub, const shared_ptr<FSMParam>& fp, const int from_id,
    const int to_id, const double stamp, const int8_t phase) {
  exploration_manager::MeetingOpt opt;
  opt.from_drone_id = from_id;
  opt.to_drone_id = to_id;
  opt.stamp = stamp;
  opt.phase = phase;
  publishRepeated(opt_pub, opt, getMeetingOptRepeatNum(fp, phase));
}

void C2ExplorationFSM::sendMeetingOptAdaptive(ros::Publisher& opt_pub, const shared_ptr<FSMParam>& fp, const int from_id,
    const int to_id, const double stamp, const int8_t phase, const vector<int>& ego_ids,
    const vector<int>& other_ids, const vector<Vector3d>& ego_centers,
    const vector<Vector3d>& other_centers, const vector<vector<Vector3d>>& ego_hulls,
    const vector<vector<Vector3d>>& other_hulls) {
  exploration_manager::MeetingOpt opt;
  opt.from_drone_id = from_id;
  opt.to_drone_id = to_id;
  opt.stamp = stamp;
  opt.phase = phase;
  fillMeetingOptPayload(ego_ids, other_ids, ego_centers, other_centers, ego_hulls, other_hulls, opt);
  publishRepeated(opt_pub, opt, getMeetingOptRepeatNum(fp, phase));
}

void C2ExplorationFSM::sendMeetingOptResponseAdaptive(ros::Publisher& opt_res_pub, const shared_ptr<FSMParam>& fp,
    const int from_id, const int to_id, const double stamp, const int8_t phase,
    const int32_t status) {
  exploration_manager::MeetingOptResponse response;
  response.from_drone_id = from_id;
  response.to_drone_id = to_id;
  response.phase = phase;
  response.status = status;
  response.stamp = stamp;
  publishRepeated(opt_res_pub, response, getMeetingOptResRepeatNum(fp, phase));
}

void C2ExplorationFSM::handleHostWaitSession(ExplorationData& ed, const shared_ptr<FSMParam>& fp,
    ros::Publisher& opt_pub, const int self_id, const double now, const bool wait_commit_ack) {
  size_t host_idx = 0;
  if (!findParticipantIndex(ed.opt_participant_ids_, self_id, host_idx)) {
    if (fp && fp->debug_opt_output_) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[Meeting Opt] Host " << self_id
                                                          << (wait_commit_ack ? " clear invalid "
                                                                                "commit-ack "
                                                                                "waiting session" :
                                                                                " clear invalid "
                                                                                "waiting session"));
    }
    clearHostOptSession(ed);
    return;
  }

  double* wait_start_time =
      wait_commit_ack ? &ed.opt_commit_wait_start_time_ : &ed.opt_wait_start_time_;
  double* last_send_time =
      wait_commit_ack ? &ed.opt_commit_last_send_time_ : &ed.opt_last_send_time_;
  int* retry_count = wait_commit_ack ? &ed.opt_commit_retry_count_ : &ed.opt_retry_count_;
  const vector<int>* ack_ids = wait_commit_ack ? &ed.opt_commit_ack_ids_ : &ed.opt_ack_ids_;
  const int8_t phase = wait_commit_ack ? kMeetingOptCommit : kMeetingOptProposal;

  if (fp->opt_wait_timeout_ > 1e-6 && now - *wait_start_time > fp->opt_wait_timeout_) {
    if (fp && fp->debug_opt_output_) {
      if (wait_commit_ack) {
        ROS_WARN_STREAM("[Meeting Opt] Host "
                        << self_id << " commit-ack wait timeout, abort stamp=" << ed.meeting_opt_stamp_
                        << " need=" << ed.opt_wait_ids_.size()
                        << " ack=" << ed.opt_commit_ack_ids_.size());
      } else {
        ROS_WARN_STREAM("[Meeting Opt] Host "
                        << self_id << " opt wait timeout, cancel stamp=" << ed.meeting_opt_stamp_
                        << " wait=" << ed.opt_wait_ids_.size() << " ack=" << ed.opt_ack_ids_.size());
      }
    }
    forEachRemoteParticipant(ed.opt_participant_ids_, self_id, [&](size_t /*idx*/, int to_id) {
      sendMeetingOptAdaptive(opt_pub, fp, self_id, to_id, ed.meeting_opt_stamp_,
          kMeetingOptCancel);
    });
    clearHostOptSession(ed);
    return;
  }

  const bool can_retry = fp->opt_retry_max_ > 0 && *retry_count < fp->opt_retry_max_;
  if (can_retry && (now - *last_send_time >= fp->opt_retry_interval_)) {
    int resent = 0;
    for (const int to_id : ed.opt_wait_ids_) {
      if (containsId(*ack_ids, to_id)) {
        if (!wait_commit_ack) {
          optDebugWarn(fp, "[Meeting Opt Debug] wait session skip retry to " +
                               std::to_string(to_id) + ": already acked");
        }
        continue;
      }
      size_t peer_idx = 0;
      if (!findParticipantIndex(ed.opt_participant_ids_, to_id, peer_idx)) {
        if (!wait_commit_ack) {
          optDebugWarn(fp, "[Meeting Opt Debug] wait session skip retry to " +
                               std::to_string(to_id) + ": participant index missing");
        }
        continue;
      }
      sendMeetingOptAdaptive(opt_pub, fp, self_id, to_id, ed.meeting_opt_stamp_, phase,
          ed.opt_alloc_ids_[host_idx], ed.opt_alloc_ids_[peer_idx], ed.opt_alloc_centers_[host_idx],
          ed.opt_alloc_centers_[peer_idx], ed.opt_alloc_center_hulls_[host_idx],
          ed.opt_alloc_center_hulls_[peer_idx]);
      ++resent;
    }
    if (resent > 0) {
      *last_send_time = now;
      ++(*retry_count);
      if (fp && fp->debug_opt_output_) {
        ROS_WARN_STREAM("[Meeting Opt] Host "
                        << self_id << " retry " << (wait_commit_ack ? "commit" : "proposal")
                        << " round " << *retry_count << "/" << fp->opt_retry_max_
                        << " resent=" << resent);
      }
    }
  } else if (!wait_commit_ack) {
    optDebugWarn(fp,
        "[Meeting Opt Debug] wait session no retry: can_retry=" + std::to_string(int(can_retry)) +
            " dt=" + std::to_string(now - *last_send_time) +
            " retry_interval=" + std::to_string(fp->opt_retry_interval_));
  }

  optDebugWarn(fp, wait_commit_ack ? "[Meeting Opt Debug] optTimer return: waiting commit acks" :
                                     "[Meeting Opt Debug] optTimer return: waiting opt responses");
}
}  // namespace c2_expl
