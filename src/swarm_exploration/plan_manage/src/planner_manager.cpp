// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

namespace c2_expl {
// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {
}

FastPlannerManager::~FastPlannerManager() {
  std::cout << "des manager" << std::endl;
}

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/accept_vel", pp_.accept_vel_, pp_.max_vel_ + 0.5);
  nh.param("manager/accept_acc", pp_.accept_acc_, pp_.max_acc_ + 0.5);
  nh.param("manager/max_yawdot", pp_.max_yawdot_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
  nh.param("manager/bspline_degree", pp_.bspline_degree_, 3);
  nh.param("manager/min_time", pp_.min_time_, false);
  nh.param("manager/relax_time1", pp_.relax_time1_, 0.5);
  nh.param("manager/relax_time2", pp_.relax_time2_, 0.5);

  bool use_geometric_path, use_kinodynamic_path, use_optimization, use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_optimization", use_optimization, false);
  nh.param("manager/use_active_perception", use_active_perception, false);

  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    path_finder_.reset(new Astar);
    // path_finder_->setParam(nh);
    // path_finder_->setEnvironment(edt_environment_);
    // path_finder_->init();
    path_finder_->init(nh, edt_environment_);
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_active_perception) {
    frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
    // heading_planner_.reset(new HeadingPlanner(nh));
    // heading_planner_->setMap(sdf_map_);
    // plan_data_.view_cons_.idx_ = -1;
  }
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance) {
  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoorT(t_now);
  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoorT(t_now + fut_t);
    // double dist = edt_environment_->sdf_map_->getDistance(fut_pt);
    if (sdf_map_->getInflateOccupancy(fut_pt) == 1) {
      distance = radius;
      // std::cout << "collision at: " << fut_pt.transpose() << ", dist: " << dist << std::endl;
      std::cout << "collision at: " << fut_pt.transpose() << std::endl;
      return false;
    }
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(const Eigen::Vector3d& start_pt,
    const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
    const Eigen::Vector3d& end_pt, const Eigen::Vector3d& end_vel, const double& time_lb) {
  // std::cout << "[Kino replan]: start: " << start_pt.transpose() << ", " << start_vel.transpose()
  // << ", "
  //           << start_acc.transpose() << ", goal:" << end_pt.transpose() << ", " <<
  //           end_vel.transpose()
  //           << endl;

  if ((start_pt - end_pt).norm() < 1e-2) {
    cout << "Close goal" << endl;
    return false;
  }

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // Kinodynamic path searching

  auto t1 = ros::Time::now();

  kino_path_finder_->reset();
  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);
  if (status == KinodynamicAstar::NO_PATH) {
    ROS_INFO("search 1 fail");
    // Retry
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);
    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[Kino replan]: Can't find path." << endl;
      return false;
    }
  }
  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  double t_search = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Parameterize path to B-spline
  double ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  // std::cout << "point set:" << std::endl;
  // for (auto pt : point_set) std::cout << pt.transpose() << std::endl;
  // std::cout << "derivative:" << std::endl;
  // for (auto dr : start_end_derivatives) std::cout << dr.transpose() << std::endl;

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      ts, point_set, start_end_derivatives, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline init(ctrl_pts, pp_.bspline_degree_, ts);

  // B-spline-based optimization
  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_function |= BsplineOptimizer::MINTIME;
  vector<Eigen::Vector3d> start, end;
  init.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  // Swarm collision avoidance term
  cost_function |= BsplineOptimizer::SWARM;
  vector<NonUniformBspline> trajs;
  swarm_traj_data_.getValidTrajs(trajs);
  bspline_optimizers_[0]->setSwarmTrajs(trajs);

  bspline_optimizers_[0]->optimize(ctrl_pts, ts, cost_function, 1, 1);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, ts);

  vector<Eigen::Vector3d> start2, end2;
  local_data_.position_traj_.getBoundaryStates(2, 0, start2, end2);
  // std::cout << "State error: (" << (start2[0] - start[0]).norm() << ", "
  //           << (start2[1] - start[1]).norm() << ", " << (start2[2] - start[2]).norm() << ")"
  //           << std::endl;

  double t_opt = (ros::Time::now() - t1).toSec();
  // ROS_INFO("Kino t: %lf, opt: %lf", t_search, t_opt);

  updateTrajInfo();
  return true;
}

void FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3d>& tour,
    const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc, const double& time_lb) {
  if (tour.empty()) ROS_ERROR("Empty path to traj planner");

  // Generate traj through waypoints-based method
  const int pt_num = tour.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = tour[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd times(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
    times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

  PolynomialTraj init_traj;
  PolynomialTraj::waypointsTraj(pos, cur_vel, zero, cur_acc, zero, times, init_traj);

  // B-spline-based optimization
  vector<Vector3d> points, boundary_deri;
  double duration = init_traj.getTotalTime();
  int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist;
  seg_num = max(8, seg_num);
  double dt = duration / double(seg_num);

  // std::cout << "duration: " << duration << ", seg_num: " << seg_num << ", dt: " << dt <<
  // std::endl;

  for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
    points.push_back(init_traj.evaluate(ts, 0));
  boundary_deri.push_back(init_traj.evaluate(0.0, 1));
  boundary_deri.push_back(init_traj.evaluate(duration, 1));
  boundary_deri.push_back(init_traj.evaluate(0.0, 2));
  boundary_deri.push_back(init_traj.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      dt, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);

  int cost_func = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_func |= BsplineOptimizer::MINTIME;

  vector<Vector3d> start, end;
  tmp_traj.getBoundaryStates(2, 0, start, end);
  start[0] = tour[0];
  start[1] = cur_vel;
  start[2] = cur_acc;
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  // Swarm collision avoidance term
  cost_func |= BsplineOptimizer::SWARM;
  vector<NonUniformBspline> trajs;
  swarm_traj_data_.getValidTrajs(trajs);
  bspline_optimizers_[0]->setSwarmTrajs(trajs);

  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  updateTrajInfo();
}

// !SECTION

// SECTION global planning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  // Generate global reference trajectory
  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  points.insert(points.begin(), start_pos);

  // Insert intermediate points if two waypoints are too far
  vector<Eigen::Vector3d> inter_points;
  const double dist_thresh = 4.0;

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();
    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;
      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }
  inter_points.push_back(points.back());

  // At least 3 waypoints are required to solve the problem
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  int pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

  time(0) += pp_.max_vel_ / (2 * pp_.max_acc_);
  time(time.rows() - 1) += pp_.max_vel_ / (2 * pp_.max_acc_);

  PolynomialTraj gl_traj;
  PolynomialTraj::waypointsTraj(pos, zero, zero, zero, zero, time, gl_traj);

  auto time_now = ros::Time::now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // truncate a local trajectory

  double dt, duration;
  Eigen::MatrixXd ctrl_pts = paramLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, pp_.bspline_degree_, dt);

  std::cout << "ctrl pt: " << ctrl_pts.rows() << std::endl;

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_ = time_now;
  ROS_INFO("global trajectory generated.");

  updateTrajInfo();

  return true;
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();

  local_data_.traj_id_ += 1;
}

Eigen::MatrixXd FastPlannerManager::paramLocalTraj(double start_t, double& dt, double& duration) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;
  global_data_.getTrajInfoInSphere(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
      start_end_derivative, dt, duration);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      dt, point_set, start_end_derivative, pp_.bspline_degree_, ctrl_pts);

  return ctrl_pts;
}
// !SECTION

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  auto t1 = ros::Time::now();
  // calculate waypoints of heading

  auto& pos = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw = 0.3;
  int seg_num = ceil(duration / dt_yaw);
  dt_yaw = duration / seg_num;

  const double forward_t = 2.0;
  double last_yaw = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS |
                  BsplineOptimizer::START | BsplineOptimizer::END;

  vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw[0], 0, 0),
    Eigen::Vector3d(start_yaw[1], 0, 0), Eigen::Vector3d(start_yaw[2], 0, 0) };
  vector<Eigen::Vector3d> end = { Eigen::Vector3d(end_yaw[0], 0, 0),
    Eigen::Vector3d(end_yaw[1], 0, 0), Eigen::Vector3d(end_yaw[2], 0, 0) };
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, pp_.bspline_degree_, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_ = path_yaw;
  plan_data_.dt_yaw_ = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "yaw time: " << (ros::Time::now() - t1).toSec() << std::endl;
}

// only consider smoothness and boundary state
void FastPlannerManager::planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw,
    bool lookfwd, const double& relax_time) {
  const int seg_num = 12;
  double dt_yaw = local_data_.duration_ / double(seg_num);  // time of B-spline segment

  // double dt_yaw = 0.5;
  // int seg_num = ceil(local_data_.duration_ / dt_yaw);
  // seg_num = max(5, seg_num);  // Restrict the minimal num
  // dt_yaw = local_data_.duration_ / double(seg_num);

  Eigen::Vector3d start_yaw3d = start_yaw;
  // std::cout << "dt_yaw: " << dt_yaw << ", start yaw: " << start_yaw3d[0] << ", end: " << end_yaw
  //           << std::endl;

  while (start_yaw3d[0] < -M_PI) start_yaw3d[0] += 2 * M_PI;
  while (start_yaw3d[0] > M_PI) start_yaw3d[0] -= 2 * M_PI;
  double last_yaw = start_yaw3d[0];
  int last_idx = 0;

  // Yaw traj control points
  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  // Initial state
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

  // Add waypoint constraints if look forward is enabled
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;
  if (lookfwd) {
    const double forward_t = 2.0;
    const int relax_num1 = pp_.relax_time1_ / dt_yaw;
    const int relax_num2 = pp_.relax_time2_ / dt_yaw;
    for (int i = 1; i < seg_num - relax_num2; ++i) {
      double tc = i * dt_yaw;
      Eigen::Vector3d pc = local_data_.position_traj_.evaluateDeBoorT(tc);
      double tf = min(local_data_.duration_, tc + forward_t);
      Eigen::Vector3d pf = local_data_.position_traj_.evaluateDeBoorT(tf);
      Eigen::Vector3d pd = pf - pc;
      Eigen::Vector3d waypt;
      if (pd.norm() > 1e-6) {
        waypt(0) = atan2(pd(1), pd(0));
        waypt(1) = waypt(2) = 0.0;
        calcNextYaw(last_yaw, waypt(0));

        // Check if the waypoint satisfy yawdot constraint
        double dt1 = double(i - last_idx) * dt_yaw;
        double dt2 = double(seg_num + 1 - i) * dt_yaw;

        double diff1 = fabs(waypt(0) - last_yaw);
        double yd1 = min(diff1, 2 * M_PI - diff1) / dt1;

        double tmp2 = end_yaw;
        calcNextYaw(waypt(0), tmp2);
        double diff2 = fabs(waypt(0) - tmp2);
        double yd2 = min(diff2, 2 * M_PI - diff2) / dt2;

        // std::cout << "waypt: " << waypt[0] << ", yd1: " << yd1 << ", yd2: " << yd2 << std::endl;

        if (yd1 < pp_.max_yawdot_ && yd2 < pp_.max_yawdot_) {
          last_yaw = waypt(0);
          waypts.push_back(waypt);
          waypt_idx.push_back(i);
          // std::cout << "add waypt" << std::endl;
        } else {
          // std::cout << "ignore waypt" << std::endl;
        }
      }
    }
  }
  // Final state
  Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
  calcNextYaw(last_yaw, end_yaw3d(0));
  yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;

  // Debug rapid change of yaw
  if (fabs(start_yaw3d[0] - end_yaw3d[0]) >= M_PI) {
    // ROS_ERROR("Yaw change rapidly!");
    std::cout << "start yaw: " << start_yaw3d[0] << ", " << end_yaw3d[0] << std::endl;
  }

  // // Interpolate start and end value for smoothness
  // for (int i = 1; i < seg_num; ++i)
  // {
  //   double tc = i * dt_yaw;
  //   Eigen::Vector3d waypt = (1 - double(i) / seg_num) * start_yaw3d + double(i) / seg_num *
  //   end_yaw3d; std::cout << "i: " << i << ", wp: " << waypt[0] << ", "; calcNextYaw(last_yaw,
  //   waypt(0));
  // }
  // std::cout << "" << std::endl;

  auto t1 = ros::Time::now();

  // Call B-spline optimization solver
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START | BsplineOptimizer::END |
                  BsplineOptimizer::WAYPOINTS;
  vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw3d[0], 0, 0),
    Eigen::Vector3d(start_yaw3d[1], 0, 0), Eigen::Vector3d(start_yaw3d[2], 0, 0) };
  vector<Eigen::Vector3d> end = { Eigen::Vector3d(end_yaw3d[0], 0, 0), Eigen::Vector3d(0, 0, 0) };
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // std::cout << "2: " << (ros::Time::now() - t1).toSec() << std::endl;

  // Update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  plan_data_.dt_yaw_ = dt_yaw;

  // plan_data_.path_yaw_ = path;
  // plan_data_.dt_yaw_path_ = dt_yaw * subsp;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]
  double round_last = last_yaw;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

bool FastPlannerManager::checkSwarmCollision(const int& id) {
  // Check collision with another drone's trajectory
  if (swarm_traj_data_.receive_flags_[id - 1] == false) return true;

  auto& traj = swarm_traj_data_.swarm_trajs_[id - 1];

  double self_t = 0.02 + (ros::Time::now() - local_data_.start_time_).toSec();
  double other_t = 0.02 + ros::Time::now().toSec() - traj.start_time_;

  while (self_t < local_data_.duration_ && self_t > 0 && other_t < traj.duration_) {
    auto self_pos = local_data_.position_traj_.evaluateDeBoorT(self_t);
    auto other_pos = traj.evaluateDeBoorT(other_t);
    // if ((self_pos - other_pos).norm() < 0.6) return false;
    // if ((self_pos - other_pos).head<2>().norm() < 0.2) return false;
    if ((self_pos - other_pos).head<2>().norm() < 0.5) return false;

    self_t += 0.02;
    other_t += 0.02;
  }
  return true;
}

}  // namespace c2_expl
