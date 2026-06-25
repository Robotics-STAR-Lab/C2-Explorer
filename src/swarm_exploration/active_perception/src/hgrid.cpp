#include <active_perception/uniform_grid.h>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <cmath>
#include <limits>
#include <active_perception/hgrid.h>
#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>

namespace c2_expl {

HGrid::HGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh) {

  this->edt_ = edt;

  nh.param("partitioning/use_swarm_tf", use_swarm_tf_, false);
  nh.param("partitioning/grid_size", grid_size_, 5.0);
  nh.param("partitioning/w_distance", w_distance_, 1.0);  // Weight for distance cost
  nh.param("partitioning/w_global_vel_change", w_global_vel_change_, 1.0);
  nh.param("partitioning/w_global_dist_time", w_global_dist_time_, w_global_vel_change_);
  nh.param("partitioning/w_global_turn_time", w_global_turn_time_, w_global_vel_change_);
  nh.param("partitioning/w_global_z_change", w_global_z_change_, 1.0);
  nh.param("exploration/vm", global_vm_, 2.0);
  nh.param("exploration/am", global_am_, 2.0);
  nh.param("exploration/vz", global_vz_, global_vm_);
  nh.param("partitioning/alpha_consistency", alpha_consistency_,
      1.2);  // Alpha for grid adjacency penalty
  nh.param("partitioning/path_cost_adjacent_thresh", path_cost_adjacent_thresh_, 1.0);
  nh.param("partitioning/convex_split_hull_area_ratio", convex_split_hull_area_ratio_,
      kConvexSplitHullAreaRatio);
  nh.param("exploration/drone_id", drone_id_, -1);

  path_finder_.reset(new Astar);
  path_finder_->init(nh, edt);

  grid1_.reset(new UniformGrid(edt, nh, 1));

  // Swarm tf
  grid1_->use_swarm_tf_ = use_swarm_tf_;
  double yaw = 0.0;
  rot_sw_ << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  trans_sw_ << 0.0, 0.0, 0;
  grid1_->rot_sw_ = rot_sw_;
  grid1_->trans_sw_ = trans_sw_;

  grid1_->initGridData();

  updateBaseCoor();
}

HGrid::~HGrid() {
}

bool HGrid::updateBaseCoor() {
  rot_sw_ = Eigen::Matrix3d::Identity();
  trans_sw_ = Eigen::Vector3d::Zero();

  grid1_->rot_sw_ = rot_sw_;
  grid1_->trans_sw_ = trans_sw_;
  grid1_->updateBaseCoor();

  return true;
}

void HGrid::inputFrontiers(const vector<Eigen::Vector3d>& avgs) {
  // Single-layer mode: only level-1 grid is used.
  last_frontier_averages_ = avgs;
  grid1_->inputFrontiers(avgs);
}

void HGrid::updateGridData(const int& drone_id, vector<int>& grid_ids, bool reallocated,
    const vector<int>& last_grid_ids, vector<int>& first_ids, vector<int>& second_ids,
    const Eigen::Vector3d& pos) {
  drone_id_ = drone_id;
  // Single-layer mode (Falcon style): only level-1 ids are valid.
  vector<int> grid_ids_l1;
  grid_ids_l1.reserve(grid_ids.size());
  for (const int id : grid_ids) {
    if (id >= 0 && id < static_cast<int>(grid1_->grid_data_.size())) {
      grid_ids_l1.push_back(id);
    }
  }

  vector<int> parti_ids_l1, parti_ids_l1_all;
  grid1_->updateGridData(drone_id, grid_ids_l1, parti_ids_l1, parti_ids_l1_all, pos);
  grid_ids = grid_ids_l1;

  // Maintain consistency of next visited grid
  getConsistentGrid(last_grid_ids, grid_ids, first_ids, second_ids);
}

// Find the first two grids in the current assignment that remain consistent with the previous
// sequence, preserving path continuity.
void HGrid::getConsistentGrid(const vector<int>& last_ids, const vector<int>& cur_ids,
    vector<int>& first_ids, vector<int>& second_ids) {
  first_ids.clear();
  second_ids.clear();
  if (last_ids.empty() || cur_ids.empty()) return;

  auto normalize = [&](int id) -> int { return toCoarseId(id); };

  const int first_target = normalize(last_ids.front());
  if (first_target < 0) return;
  int second_target = -1;
  for (int i = 1; i < static_cast<int>(last_ids.size()); ++i) {
    const int cand = normalize(last_ids[i]);
    if (cand != first_target) {
      second_target = cand;
      break;
    }
  }

  for (const int id : cur_ids) {
    if (normalize(id) == first_target) {
      first_ids.push_back(id);
    }
  }

  if (second_target < 0) return;
  for (const int id : cur_ids) {
    if (normalize(id) == second_target) {
      second_ids.push_back(id);
    }
  }
}

double HGrid::computeGridAdjacencyConsistencyFactor(const double path_cost) const {
  if (!std::isfinite(path_cost) || path_cost <= 0.0) return 1.0;

  const double alpha = std::max(1e-3, std::abs(alpha_consistency_));
  const double gsize = std::max(1e-3, grid_size_);
  const double rho = path_cost / (alpha * gsize);

  if (rho <= 1.0 + 1e-6) return 1.0;

  const double growth = 1.0 + (rho - 1) * (rho - 1);
  return growth;
}

void HGrid::computePathExtraCostsFromPoints(const vector<Eigen::Vector3d>& path,
    const Eigen::Vector3d* start_vel, double& vel_change_cost, double& z_change_cost) const {
  vel_change_cost = 0.0;
  z_change_cost = 0.0;
  if (path.size() < 2) return;

  const double vm = std::max(1e-3, global_vm_);
  const double am = std::max(1e-3, global_am_);
  const double vz = std::max(1e-3, global_vz_);

  // Merge collinear segments, same pattern as Falcon PathCostEvaluator::computePathCost.
  vector<Eigen::Vector3d> path_merged;
  path_merged.reserve(path.size());
  path_merged.push_back(path.front());
  for (int k = 1; k < static_cast<int>(path.size()) - 1; ++k) {
    const Eigen::Vector3d seg1 = path[k] - path[k - 1];
    const Eigen::Vector3d seg2 = path[k + 1] - path[k];
    if (seg1.norm() < 1e-6 || seg2.norm() < 1e-6) continue;
    const Eigen::Vector3d dir1 = seg1.normalized();
    const Eigen::Vector3d dir2 = seg2.normalized();
    if (dir1.dot(dir2) < 1.0 - 1e-4) {
      path_merged.push_back(path[k]);
    }
  }
  path_merged.push_back(path.back());
  if (path_merged.size() < 2) return;

  // Initial velocity-change cost.
  if (start_vel != nullptr && start_vel->norm() > 1e-3) {
    const Eigen::Vector3d seg = path_merged[1] - path_merged[0];
    if (seg.norm() > 1e-6) {
      const Eigen::Vector3d dir = seg.normalized();
      const double vc = start_vel->dot(dir);
      double t = std::pow(vm - std::abs(vc), 2) / (2.0 * vm * am);
      if (vc < 0.0) t += 2.0 * std::abs(vc) / am;
      vel_change_cost += t;
    }
  }

  // Velocity-change cost at each turning point.
  for (int k = 1; k < static_cast<int>(path_merged.size()) - 1; ++k) {
    const Eigen::Vector3d seg1 = path_merged[k] - path_merged[k - 1];
    const Eigen::Vector3d seg2 = path_merged[k + 1] - path_merged[k];
    if (seg1.norm() < 1e-6 || seg2.norm() < 1e-6) continue;
    const Eigen::Vector3d dir1 = seg1.normalized();
    const Eigen::Vector3d dir2 = seg2.normalized();
    const double vc = (dir1 * vm).dot(dir2);
    double t = std::pow(vm - std::abs(vc), 2) / (2.0 * vm * am);
    if (vc < 0.0) t += 2.0 * std::abs(vc) / am;
    vel_change_cost += t;
  }

  // Z-change cost.
  for (int k = 0; k < static_cast<int>(path_merged.size()) - 1; ++k) {
    z_change_cost += std::abs(path_merged[k + 1].z() - path_merged[k].z()) / vz;
  }
}

void HGrid::computePathExtraCostsFromGraph(ConnectivityGraph* cg, const std::vector<int>& node_path,
    const Eigen::Vector3d* start_vel, double& vel_change_cost, double& z_change_cost) const {
  vel_change_cost = 0.0;
  z_change_cost = 0.0;
  if (!cg || node_path.size() < 2) return;

  vector<Eigen::Vector3d> path_pts;
  path_pts.reserve(node_path.size());
  for (const int id : node_path) {
    auto node = cg->getNode(id);
    if (!node) return;
    path_pts.push_back(node->pos_);
  }
  computePathExtraCostsFromPoints(path_pts, start_vel, vel_change_cost, z_change_cost);
}

double HGrid::computeDroneToCenterCost(const Eigen::Vector3d& drone_pos,
    const Eigen::Vector3d& drone_vel, const Eigen::Vector3d& center_pos, UniformGrid* center_ug,
    int center_node_id, ConnectivityGraph* drone_cg, int drone_cg_node_id, int center_idx,
    double hybrid_radius, bool for_global_tour) {
  (void)center_idx;

  const double dist = (drone_pos - center_pos).norm();
  double dist_cost = 1000.0;
  double vel_change_cost = 0.0;
  double z_change_cost = 0.0;

  if (dist < hybrid_radius) {
    path_finder_->reset();
    if (path_finder_->search(drone_pos, center_pos) == Astar::REACH_END) {
      auto path = path_finder_->getPath();
      dist_cost = path_finder_->pathLength(path);
      if (for_global_tour) {
        computePathExtraCostsFromPoints(path, &drone_vel, vel_change_cost, z_change_cost);
      }
    } else {
      dist_cost = 1000.0;
    }
  } else {
    dist_cost = 2.0 * dist;
  }

  const bool same_graph = (drone_cg != nullptr && center_ug != nullptr &&
                           drone_cg == center_ug->getConnectivityGraph().get());
  const bool can_use_cg = (same_graph && drone_cg_node_id != -1 && center_node_id >= 0);
  if ((dist >= hybrid_radius || dist_cost > 499.0) && can_use_cg) {
    std::vector<int> path;
    const double cg_dist_cost =
        drone_cg->searchConnectivityGraphBFS(drone_cg_node_id, center_node_id, path);
    if (cg_dist_cost < 1000.0) {
      dist_cost = cg_dist_cost;
      if (for_global_tour) {
        computePathExtraCostsFromGraph(
            drone_cg, path, &drone_vel, vel_change_cost, z_change_cost);
      }
    }
  }

  if (!for_global_tour) {
    dist_cost *= computeGridAdjacencyConsistencyFactor(dist_cost);
  }

  if (for_global_tour) {
    const double vm = std::max(1e-3, global_vm_);
    const double dist_time_cost = dist_cost / vm;
    const double turn_time_cost = vel_change_cost;
    return w_global_dist_time_ * dist_time_cost + w_global_turn_time_ * turn_time_cost +
           w_global_z_change_ * z_change_cost;
  }
  return w_distance_ * dist_cost;
}

double HGrid::computeCenterToCenterCost(const Eigen::Vector3d& center_i_pos, UniformGrid* center_i_ug,
    int center_i_node_id, int center_i_idx, const Eigen::Vector3d& center_j_pos,
    UniformGrid* center_j_ug, int center_j_node_id, int center_j_idx, double hybrid_radius,
    bool for_global_tour) {
  (void)center_i_idx;
  (void)center_j_idx;
  
  const double dist = (center_i_pos - center_j_pos).norm();
  double dist_cost = 1000.0;
  double vel_change_cost = 0.0;
  double z_change_cost = 0.0;

  if (dist < hybrid_radius) {
    path_finder_->reset();
    if (path_finder_->search(center_i_pos, center_j_pos) == Astar::REACH_END) {
      auto path = path_finder_->getPath();
      dist_cost = path_finder_->pathLength(path);
      if (for_global_tour) {
        computePathExtraCostsFromPoints(path, nullptr, vel_change_cost, z_change_cost);
      }
    } else {
      dist_cost = 1000.0;
    }
  }

  ConnectivityGraph::Ptr cg;
  const bool same_graph = (center_i_ug != nullptr && center_i_ug == center_j_ug);
  if (same_graph) {
    cg = center_i_ug->getConnectivityGraph();
  }
  const bool can_use_cg = (cg && center_i_node_id >= 0 && center_j_node_id >= 0);
  if ((dist >= hybrid_radius || dist_cost > 499.0) && can_use_cg) {
    std::vector<int> path;
    dist_cost = cg->searchConnectivityGraphBFS(center_i_node_id, center_j_node_id, path);
    if (for_global_tour && dist_cost < 1000.0) {
      computePathExtraCostsFromGraph(
          cg.get(), path, nullptr, vel_change_cost, z_change_cost);
    }
  } else if (dist >= hybrid_radius && !can_use_cg) {
    dist_cost = 2.0 * dist;
  } else if (dist < hybrid_radius && dist_cost > 499.0 && !can_use_cg) {
    dist_cost = dist;
  }

  if (!for_global_tour) {
    dist_cost *= computeGridAdjacencyConsistencyFactor(dist_cost);
  }

  if (for_global_tour) {
    const double vm = std::max(1e-3, global_vm_);
    const double dist_time_cost = dist_cost / vm;
    const double turn_time_cost = vel_change_cost;
    return w_global_dist_time_ * dist_time_cost + w_global_turn_time_ * turn_time_cost +
           w_global_z_change_ * z_change_cost;
  }
  return w_distance_ * dist_cost;
}

vector<Eigen::Vector3d> HGrid::buildConvexHull(vector<Pt2> pts, double z) const {
  vector<Eigen::Vector3d> hull;
  if (pts.empty()) return hull;

  std::sort(pts.begin(), pts.end(), [](const Pt2& a, const Pt2& b) {
    if (a.x == b.x) return a.y < b.y;
    return a.x < b.x;
  });

  vector<Pt2> unique_pts;
  unique_pts.reserve(pts.size());
  constexpr double kEps = 1e-3;
  for (const auto& p : pts) {
    if (!unique_pts.empty()) {
      const auto& q = unique_pts.back();
      if (std::abs(p.x - q.x) < kEps && std::abs(p.y - q.y) < kEps) continue;
    }
    unique_pts.push_back(p);
  }
  pts.swap(unique_pts);

  if (pts.size() == 1) {
    hull.emplace_back(pts[0].x, pts[0].y, z);
    return hull;
  }

  auto cross = [](const Pt2& a, const Pt2& b, const Pt2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  };

  vector<Pt2> lower, upper;
  for (const auto& p : pts) {
    while (lower.size() >= 2 && cross(lower[lower.size() - 2], lower.back(), p) <= 1e-8) {
      lower.pop_back();
    }
    lower.push_back(p);
  }
  for (int i = static_cast<int>(pts.size()) - 1; i >= 0; --i) {
    const auto& p = pts[i];
    while (upper.size() >= 2 && cross(upper[upper.size() - 2], upper.back(), p) <= 1e-8) {
      upper.pop_back();
    }
    upper.push_back(p);
  }

  vector<Pt2> poly;
  poly.reserve(lower.size() + upper.size());
  for (const auto& p : lower) poly.push_back(p);
  for (size_t i = 1; i + 1 < upper.size(); ++i) poly.push_back(upper[i]);

  hull.reserve(poly.size());
  for (const auto& p : poly) {
    hull.emplace_back(p.x, p.y, z);
  }
  return hull;
}

double HGrid::getHullArea2D(const vector<Eigen::Vector3d>& hull) const {
  if (hull.size() < 3) return 0.0;
  double area2 = 0.0;
  for (size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i];
    const auto& b = hull[(i + 1) % hull.size()];
    area2 += a.x() * b.y() - a.y() * b.x();
  }
  return 0.5 * std::abs(area2);
}

bool HGrid::pointInsideHull2D(
    const vector<Eigen::Vector3d>& hull, const Eigen::Vector3d& p, double tol) const {
  if (hull.empty()) return false;
  if (hull.size() == 1) {
    return (hull.front().head<2>() - p.head<2>()).norm() <= tol;
  }
  if (hull.size() == 2) {
    const auto a = hull[0].head<2>();
    const auto b = hull[1].head<2>();
    const auto ab = b - a;
    const double len = ab.norm();
    if (len < 1e-6) return (a - p.head<2>()).norm() <= tol;
    const double t = std::max(0.0, std::min(1.0, (p.head<2>() - a).dot(ab) / (len * len)));
    const auto proj = a + t * ab;
    return (proj - p.head<2>()).norm() <= tol;
  }

  int sign = 0;
  for (size_t i = 0; i < hull.size(); ++i) {
    const auto a = hull[i].head<2>();
    const auto b = hull[(i + 1) % hull.size()].head<2>();
    const auto e = b - a;
    const double el = e.norm();
    if (el < 1e-6) continue;
    const double cross = e.x() * (p.y() - a.y()) - e.y() * (p.x() - a.x());
    if (std::abs(cross) <= tol * el) continue;
    const int cur_sign = (cross > 0.0) ? 1 : -1;
    if (sign == 0) {
      sign = cur_sign;
    } else if (sign != cur_sign) {
      return false;
    }
  }
  return true;
}

bool HGrid::hullContainsFrontier(const vector<Eigen::Vector3d>& hull) const {
  for (const auto& avg : last_frontier_averages_) {
    if (pointInsideHull2D(hull, avg)) return true;
  }
  return false;
}

Eigen::Vector3d HGrid::getHullCentroid(const vector<Eigen::Vector3d>& hull) const {
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  if (hull.empty()) return c;
  for (const auto& p : hull) c += p;
  c /= static_cast<double>(hull.size());
  return c;
}

bool HGrid::splitByKMeans2(const vector<Pt2>& pts, vector<Pt2>& g0, vector<Pt2>& g1) const {
  g0.clear();
  g1.clear();
  if (pts.size() < 2) return false;

  int seed0 = 0;
  int seed1 = 0;
  double far = -1.0;
  for (int i = 1; i < static_cast<int>(pts.size()); ++i) {
    const double dx = pts[i].x - pts[seed0].x;
    const double dy = pts[i].y - pts[seed0].y;
    const double d2 = dx * dx + dy * dy;
    if (d2 > far) {
      far = d2;
      seed1 = i;
    }
  }
  if (seed1 == seed0) return false;

  Pt2 c0 = pts[seed0];
  Pt2 c1 = pts[seed1];
  vector<int> labels(pts.size(), -1);
  bool changed = true;
  for (int iter = 0; iter < 10 && changed; ++iter) {
    changed = false;
    double s0x = 0.0, s0y = 0.0, s1x = 0.0, s1y = 0.0;
    int n0 = 0, n1 = 0;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
      const double d0 =
          (pts[i].x - c0.x) * (pts[i].x - c0.x) + (pts[i].y - c0.y) * (pts[i].y - c0.y);
      const double d1 =
          (pts[i].x - c1.x) * (pts[i].x - c1.x) + (pts[i].y - c1.y) * (pts[i].y - c1.y);
      const int label = (d0 <= d1) ? 0 : 1;
      if (labels[i] != label) {
        labels[i] = label;
        changed = true;
      }
      if (label == 0) {
        s0x += pts[i].x;
        s0y += pts[i].y;
        ++n0;
      } else {
        s1x += pts[i].x;
        s1y += pts[i].y;
        ++n1;
      }
    }
    if (n0 == 0 || n1 == 0) return false;
    c0.x = s0x / n0;
    c0.y = s0y / n0;
    c1.x = s1x / n1;
    c1.y = s1y / n1;
  }

  for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
    if (labels[i] == 0) {
      g0.push_back(pts[i]);
    } else if (labels[i] == 1) {
      g1.push_back(pts[i]);
    }
  }
  return !g0.empty() && !g1.empty();
}

vector<HGrid::SplitHull> HGrid::collectUnknownCenterHulls(
    UniformGrid* ug, int local_id, int unknown_idx, const Eigen::Vector3d& center) const {
  vector<SplitHull> splits;
  if (!ug) return splits;
  if (local_id < 0 ||
      local_id >= static_cast<int>(ug->ccl_free_unknown_states_and_centers_idx_.size())) {
    return splits;
  }
  if (local_id < 0 || local_id >= static_cast<int>(ug->ccl_voxels_addr_.size())) {
    return splits;
  }

  const auto& center_map = ug->ccl_free_unknown_states_and_centers_idx_[local_id];
  int ccl_idx = -1;
  for (int i = 0; i < static_cast<int>(center_map.size()); ++i) {
    // 1 = unknown component.
    if (center_map[i].first == 1 && center_map[i].second == unknown_idx) {
      ccl_idx = i;
      break;
    }
  }
  if (ccl_idx < 0) return splits;
  if (ccl_idx >= static_cast<int>(ug->ccl_voxels_addr_[local_id].size())) return splits;

  const auto& voxel_addrs = ug->ccl_voxels_addr_[local_id][ccl_idx];
  if (voxel_addrs.empty()) return splits;

  vector<Pt2> xy_points;
  xy_points.reserve(voxel_addrs.size());
  for (const int addr : voxel_addrs) {
    Eigen::Vector3i idx;
    ug->edt_->sdf_map_->addressToIndex(addr, idx);
    Eigen::Vector3d pos;
    ug->edt_->sdf_map_->indexToPos(idx, pos);
    xy_points.push_back({ pos.x(), pos.y() });
  }

  const double res = ug->edt_->sdf_map_->getResolution();
  const int min_cluster_pts = std::max(kConvexSplitMinClusterPts,
      static_cast<int>(xy_points.size() * kConvexSplitMinClusterRatio));

  struct Node2D {
    vector<Pt2> pts;
    int depth = 0;
  };
  vector<Node2D> stack;
  stack.push_back({ xy_points, 0 });
  vector<vector<Pt2>> clusters;
  clusters.reserve(4);

  while (!stack.empty()) {
    Node2D cur = std::move(stack.back());
    stack.pop_back();

    if (cur.pts.empty()) continue;
    auto hull = buildConvexHull(cur.pts, center.z());
    const double hull_area = getHullArea2D(hull);
    const double occupancy_area = cur.pts.size() * res * res;
    const double ratio = hull_area / std::max(occupancy_area, res * res);

    bool can_split = true;
    if (cur.depth >= kConvexSplitMaxDepth) can_split = false;
    if (static_cast<int>(cur.pts.size()) < 2 * min_cluster_pts) can_split = false;
    if (ratio <= convex_split_hull_area_ratio_) can_split = false;
    if (static_cast<int>(clusters.size() + stack.size() + 1) >= kConvexSplitMaxParts) {
      can_split = false;
    }

    if (!can_split) {
      clusters.push_back(std::move(cur.pts));
      continue;
    }

    vector<Pt2> g0, g1;
    if (!splitByKMeans2(cur.pts, g0, g1) || static_cast<int>(g0.size()) < min_cluster_pts ||
        static_cast<int>(g1.size()) < min_cluster_pts) {
      clusters.push_back(std::move(cur.pts));
      continue;
    }

    stack.push_back({ std::move(g0), cur.depth + 1 });
    stack.push_back({ std::move(g1), cur.depth + 1 });
  }

  if (clusters.empty()) clusters.push_back(std::move(xy_points));

  for (const auto& cluster : clusters) {
    auto hull = buildConvexHull(cluster, center.z());
    if (hull.empty()) continue;

    Eigen::Vector3d c = center;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto& p : cluster) {
      const double dx = p.x - center.x();
      const double dy = p.y - center.y();
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        c.x() = p.x;
        c.y() = p.y;
      }
    }

    SplitHull sh;
    sh.center = c;
    sh.hull = std::move(hull);
    splits.push_back(std::move(sh));
  }

  std::sort(splits.begin(), splits.end(), [&](const SplitHull& a, const SplitHull& b) {
    return (a.center - center).squaredNorm() < (b.center - center).squaredNorm();
  });

  return splits;
}

void HGrid::appendCenter(vector<CenterMeta>& centers, UniformGrid* ug, int node_id,
    const Eigen::Vector3d& pos, int grid_id, int center_id, int center_type,
    const vector<Eigen::Vector3d>& hull) {
  CenterMeta meta;
  meta.ug = ug;
  meta.node_id = node_id;
  meta.pos = pos;
  centers.push_back(meta);
  last_center_grid_ids_.push_back(grid_id);
  last_center_positions_.push_back(pos);
  last_center_hulls_.push_back(hull);
  last_center_ids_.push_back(center_id);
  last_center_types_.push_back(center_type);
}

bool HGrid::appendHullFreeCenter(vector<CenterMeta>& centers, UniformGrid* ug, const int local_id,
    const int grid_id, const vector<Eigen::Vector3d>& hull) {
  if (!ug || hull.empty() || !hullContainsFrontier(hull)) return false;
  auto& grid = ug->grid_data_[local_id];
  const auto& free_active_idx = grid.centers_free_active_idx_;
  const auto& free_active_centers = grid.centers_free_active_;

  int best_k = -1;
  double best_d2 = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d ref = getHullCentroid(hull);
  for (int k = 0; k < static_cast<int>(free_active_centers.size()); ++k) {
    const Eigen::Vector3d& free_center = free_active_centers[k];
    if (!pointInsideHull2D(hull, free_center)) continue;
    const double d2 = (free_center - ref).squaredNorm();
    if (d2 < best_d2) {
      best_d2 = d2;
      best_k = k;
    }
  }

  Eigen::Vector3d free_pos = ref;
  int node_id = -1;
  if (best_k >= 0) {
    free_pos = free_active_centers[best_k];
    if (best_k < static_cast<int>(free_active_idx.size())) {
      const int free_idx = free_active_idx[best_k];
      if (free_idx >= 0 && free_idx < static_cast<int>(grid.centers_free_.size())) {
        node_id = ug->getConnectivityNodeId(local_id, free_idx);
      }
    }
  }

  appendCenter(centers, ug, node_id, free_pos, grid_id, -1, FREE_ACTIVE_CENTER, hull);
  return true;
}

void HGrid::collectCostMatrixCenters(const vector<int>& grid_ids, bool active_centers,
    const vector<HullTask>* hull_tasks, vector<CenterMeta>& centers) {
  centers.clear();
  centers.reserve(grid_ids.size() * 2);

  vector<int> normalized_ids;
  normalized_ids.reserve(grid_ids.size());
  for (const int gid : grid_ids) {
    const int coarse = toCoarseId(gid);
    if (coarse < 0 || coarse >= static_cast<int>(grid1_->grid_data_.size())) continue;
    if (std::find(normalized_ids.begin(), normalized_ids.end(), coarse) == normalized_ids.end()) {
      normalized_ids.push_back(coarse);
    }
  }

  for (const int grid_id : normalized_ids) {
    UniformGrid* ug = grid1_.get();
    const int local_id = grid_id;
    auto& grid = ug->grid_data_[local_id];
    bool appended = false;
    bool has_split_hulls = false;
    int added_unknown = 0;
    int added_free = 0;

    const auto& active_idx = grid.centers_unknown_active_idx_;
    if (!active_idx.empty()) {
      for (int idx : active_idx) {
        if (idx < 0 || idx >= static_cast<int>(grid.centers_unknown_.size())) continue;
        const Eigen::Vector3d& base_center = grid.centers_unknown_[idx];
        const int local_center_idx = grid.centers_free_.size() + idx;
        const int base_node_id = ug->getConnectivityNodeId(local_id, local_center_idx);

        auto split_hulls = collectUnknownCenterHulls(ug, local_id, idx, base_center);
        if (split_hulls.empty()) {
          appendCenter(centers, ug, base_node_id, base_center, grid_id, base_node_id,
              UNKNOWN_ACTIVE_CENTER, {});
          appended = true;
          ++added_unknown;
          continue;
        }

        has_split_hulls = true;
        for (int k = 0; k < static_cast<int>(split_hulls.size()); ++k) {
          const int node_id = (k == 0) ? base_node_id : -1;
          appendCenter(centers, ug, node_id, split_hulls[k].center, grid_id, -1,
              UNKNOWN_ACTIVE_CENTER, split_hulls[k].hull);
          appended = true;
          ++added_unknown;
        }
      }
    }

    if (hull_tasks) {
      for (const auto& task : *hull_tasks) {
        if (task.empty() || task.grid_id != grid_id) continue;
        bool already_selected = false;
        for (int idx = 0; idx < static_cast<int>(last_center_hulls_.size()); ++idx) {
          if (idx >= static_cast<int>(last_center_grid_ids_.size()) ||
              last_center_grid_ids_[idx] != grid_id)
            continue;
          if (last_center_hulls_[idx].empty()) continue;
          if (pointInsideHull2D(task.hull, last_center_positions_[idx])) {
            already_selected = true;
            break;
          }
        }
        if (already_selected) continue;
        if (appendHullFreeCenter(centers, ug, local_id, grid_id, task.hull)) {
          appended = true;
          has_split_hulls = true;
          ++added_free;
        }
      }
    }

    const auto& free_active_idx = grid.centers_free_active_idx_;
    const auto& free_active_centers = grid.centers_free_active_;

    if (!has_split_hulls && (active_centers || !appended) && !free_active_centers.empty()) {
      for (int k = 0; k < static_cast<int>(free_active_centers.size()); ++k) {
        const Eigen::Vector3d& free_center = free_active_centers[k];
        int node_id = -1;
        if (k < static_cast<int>(free_active_idx.size())) {
          const int free_idx = free_active_idx[k];
          if (free_idx >= 0 && free_idx < static_cast<int>(grid.centers_free_.size())) {
            node_id = ug->getConnectivityNodeId(local_id, free_idx);
          }
        }
        appendCenter(centers, ug, node_id, free_center, grid_id, node_id, FREE_ACTIVE_CENTER, {});
        appended = true;
        ++added_free;
      }
    }

    if (!appended) {
      ROS_WARN("[HGrid] Grid %d: total_unknown=%zu, active_unknown=%zu, total_free=%zu, "
               "active_free=%zu -> SKIPPED (no active unknown/free center)",
          grid_id, grid.centers_unknown_.size(), active_idx.size(), grid.centers_free_.size(),
          free_active_idx.size());
    } else {
      ROS_INFO_THROTTLE(5.0,
          "[HGrid] Grid %d: total_unknown=%zu, active_unknown=%zu, total_free=%zu, "
          "active_free=%zu -> added centers unknown=%d free=%d",
          grid_id, grid.centers_unknown_.size(), active_idx.size(), grid.centers_free_.size(),
          free_active_idx.size(), added_unknown, added_free);
    }
  }
}

void HGrid::getAllocationCostMatrix(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<int>& grid_ids, Eigen::MatrixXd& mat,
    const vector<HullTask>* hull_tasks) {
  buildCostMatrix(positions, velocities, grid_ids, mat, false, false, hull_tasks);
}

void HGrid::getTourCostMatrix(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<int>& grid_ids, Eigen::MatrixXd& mat,
    const vector<HullTask>* hull_tasks) {
  const bool active_centers = hull_tasks != nullptr && !hull_tasks->empty();
  buildCostMatrix(positions, velocities, grid_ids, mat, true, active_centers, hull_tasks);
}

void HGrid::buildCostMatrix(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<int>& grid_ids, Eigen::MatrixXd& mat,
    bool for_global_tour, bool active_centers, const vector<HullTask>* hull_tasks) {
  // Build per-active-center cost matrix (Falcon-style) while keeping the same depot/drone layout.
  const int drone_num = positions.size();
  if (velocities.size() != positions.size()) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[HGrid] velocities size (" << velocities.size()
                                                               << ") != positions size ("
                                                               << positions.size() << ")");
  }

  last_center_grid_ids_.clear();
  last_center_positions_.clear();
  last_center_hulls_.clear();
  last_center_ids_.clear();
  last_center_types_.clear();

  vector<CenterMeta> centers;
  collectCostMatrixCenters(grid_ids, active_centers, hull_tasks, centers);

  const int center_num = centers.size();
  const int dimen = 1 + drone_num + center_num;
  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // Virtual depot to drones
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = 0;
    mat(1 + i, 0) = 1000;
  }
  // Virtual depot to centers
  for (int i = 0; i < center_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;
    mat(1 + drone_num + i, 0) = 0;
  }
  // Costs between drones
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;
    }
  }

  const double hybrid_radius = 2.0 * grid_size_;

  auto compute_drone_to_anchor_dist =
      [&](const Eigen::Vector3d& pos, const Eigen::Vector3d& anchor) -> double {
    const double dist = (pos - anchor).norm();
    double dist_cost = 1000.0;
    if (dist < hybrid_radius) {
      path_finder_->reset();
      if (path_finder_->search(pos, anchor) == Astar::REACH_END) {
        auto path = path_finder_->getPath();
        dist_cost = path_finder_->pathLength(path);
      } else {
        dist_cost = 1000.0;
      }
    } else {
      dist_cost = 5.0 * dist;
    }
    return dist_cost;
  };

  std::vector<int> drone_cg_node_ids(drone_num, -1);
  std::vector<ConnectivityGraph::Ptr> drone_cg_ptrs(drone_num);
  ConnectivityGraph::Ptr base_cg;
  if (grid1_) {
    base_cg = grid1_->getConnectivityGraph();
  }
  if (base_cg) {
    std::vector<Eigen::Vector3d> cg_node_positions;
    std::vector<int> cg_node_ids;
    base_cg->getNodePositionsWithIDs(cg_node_positions, cg_node_ids);

    constexpr int kDroneCgAnchors = 6;
    constexpr int kDroneCgNodeBase = -1000000;
    for (int i = 0; i < drone_num; ++i) {
      if (cg_node_ids.empty()) break;
      std::vector<std::pair<double, int>> dist_idx;
      dist_idx.reserve(cg_node_ids.size());
      for (int k = 0; k < static_cast<int>(cg_node_ids.size()); ++k) {
        const double d2 = (positions[i] - cg_node_positions[k]).squaredNorm();
        dist_idx.emplace_back(d2, k);
      }
      std::sort(dist_idx.begin(), dist_idx.end(),
          [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
            return a.first < b.first;
          });

      const int drone_node_id = kDroneCgNodeBase - i;
      base_cg->removeNode(drone_node_id);
      ConnectivityNode::Ptr drone_node =
          std::make_shared<ConnectivityNode>(drone_node_id, positions[i], ConnectivityNode::TYPE::FREE);

      bool has_edge = false;
      const int anchor_num = std::min(static_cast<int>(dist_idx.size()), kDroneCgAnchors);
      for (int k = 0; k < anchor_num; ++k) {
        const int idx = dist_idx[k].second;
        const int anchor_id = cg_node_ids[idx];
        auto anchor_node = base_cg->getNode(anchor_id);
        if (!anchor_node) continue;
        const double edge_cost = compute_drone_to_anchor_dist(positions[i], anchor_node->pos_);
        if (edge_cost > 499.0) continue;
        drone_node->addNeighbor(anchor_id, edge_cost, ConnectivityEdge::TYPE::FREE);
        anchor_node->addNeighbor(drone_node_id, edge_cost, ConnectivityEdge::TYPE::FREE);
        has_edge = true;
      }

      if (!has_edge) continue;
      base_cg->addNode(drone_node);
      drone_cg_node_ids[i] = drone_node_id;
      drone_cg_ptrs[i] = base_cg;
    }
  }

  // Costs from drones to centers
  for (int i = 0; i < drone_num; ++i) {
    const Eigen::Vector3d drone_vel =
        (i < static_cast<int>(velocities.size())) ? velocities[i] : Eigen::Vector3d::Zero();
    for (int j = 0; j < center_num; ++j) {
      const double cost = computeDroneToCenterCost(positions[i], drone_vel, centers[j].pos,
          centers[j].ug, centers[j].node_id, drone_cg_ptrs[i].get(), drone_cg_node_ids[i], j,
          hybrid_radius, for_global_tour);
      mat(1 + i, 1 + drone_num + j) = cost;
      // Not allow center -> drone
      mat(1 + drone_num + j, 1 + i) = 1000;
    }
  }

  // Remove temporary drone nodes so center-center graph costs are unchanged.
  for (int i = 0; i < drone_num; ++i) {
    if (!drone_cg_ptrs[i] || drone_cg_node_ids[i] == -1) continue;
    drone_cg_ptrs[i]->removeNode(drone_cg_node_ids[i]);
  }

  // Costs between centers
  for (int i = 0; i < center_num; ++i) {
    for (int j = i + 1; j < center_num; ++j) {
      const double cost = computeCenterToCenterCost(centers[i].pos, centers[i].ug, centers[i].node_id,
          i, centers[j].pos, centers[j].ug, centers[j].node_id, j, hybrid_radius,
          for_global_tour);
      mat(1 + drone_num + i, 1 + drone_num + j) = cost;
      mat(1 + drone_num + j, 1 + drone_num + i) = cost;
    }
  }

  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 1000;
  }
}

bool HGrid::findCenterIndexByPos(const Eigen::Vector3d& pos, int& center_idx, double tol) const {
  center_idx = -1;
  if (last_center_positions_.empty()) return false;
  double best_dist = std::numeric_limits<double>::infinity();
  int best_idx = -1;
  for (int i = 0; i < static_cast<int>(last_center_positions_.size()); ++i) {
    double dist = (pos - last_center_positions_[i]).norm();
    if (dist < best_dist) {
      best_dist = dist;
      best_idx = i;
    }
  }
  if (best_idx >= 0 && best_dist <= tol) {
    center_idx = best_idx;
    return true;
  }
  return false;
}

bool HGrid::getGridIdByCenterPos(const Eigen::Vector3d& pos, int& grid_id, double tol) const {
  int idx = -1;
  if (findCenterIndexByPos(pos, idx, tol)) {
    if (idx >= 0 && idx < static_cast<int>(last_center_grid_ids_.size())) {
      grid_id = toCoarseId(last_center_grid_ids_[idx]);
      return true;
    }
  }

  // Fallback: map position to level-1 grid cell directly.
  Eigen::Vector3i id;
  grid1_->posToIndex(pos, id);
  if (grid1_->insideGrid(id)) {
    const int gid = grid1_->toAddress(id);
    if (gid >= 0 && gid < static_cast<int>(grid1_->grid_data_.size()) &&
        grid1_->grid_data_[gid].active_) {
      grid_id = gid;
      return true;
    }
  }
  return false;
}

bool HGrid::getCenterHullByPos(
    const Eigen::Vector3d& pos, vector<Eigen::Vector3d>& hull, double tol) const {
  hull.clear();
  int idx = -1;
  if (!findCenterIndexByPos(pos, idx, tol)) return false;
  if (idx < 0 || idx >= static_cast<int>(last_center_hulls_.size())) return false;
  hull = last_center_hulls_[idx];
  return true;
}

int HGrid::getUnknownCellsNum(const int& grid_id) {
  // Get unknown cell number of a grid
  return getGrid(grid_id).unknown_num_;
}

Eigen::Vector3d HGrid::getCenter(const int& id) {
  return getGrid(id).center_;
}

GridInfo& HGrid::getGrid(const int& id) {
  const int coarse = toCoarseId(id);
  if (coarse >= 0 && coarse < static_cast<int>(grid1_->grid_data_.size())) {
    return grid1_->grid_data_[coarse];
  }
  ROS_WARN_THROTTLE(1.0, "[HGrid] Invalid grid id %d, fallback to grid 0", id);
  return grid1_->grid_data_[0];
}

void HGrid::getExploredGrids(vector<int>& grid_ids) {
  grid_ids.clear();
  for (int i = 0; i < static_cast<int>(grid1_->grid_data_.size()); ++i) {
    // EXPLORED grids are usually no longer "relevant" to planning, so do not gate by
    // is_cur_relevant_ here, otherwise explored-history visualization will disappear.
    if (grid1_->grid_data_[i].active_ && grid1_->grid_data_[i].state_ == EXPLORED) {
      grid_ids.push_back(i);
    }
  }
}

bool HGrid::getNextGrid(const vector<int>& grid_ids, Eigen::Vector3d& grid_pos, double& grid_yaw) {
  if (grid_ids.empty()) return false;
  const int grid_id1 = toCoarseId(grid_ids.front());
  if (grid_id1 < 0 || grid_id1 >= static_cast<int>(grid1_->grid_data_.size())) return false;

  int grid_id2 = -1;
  for (int i = 1; i < static_cast<int>(grid_ids.size()); ++i) {
    const int coarse = toCoarseId(grid_ids[i]);
    if (coarse != grid_id1) {
      grid_id2 = coarse;
      break;
    }
  }

  if (grid_id2 == -1) return false;

  auto& grid1 = this->getGrid(grid_id1);
  auto& grid2 = this->getGrid(grid_id2);
  grid_pos = grid2.center_;
  Eigen::Vector3d dir = grid2.center_ - grid1.center_;
  grid_yaw = atan2(dir[1], dir[0]);

  return true;
}

bool HGrid::isClose(const int& id1, const int& id2) {
  int tmp_id1 = toCoarseId(id1);
  int tmp_id2 = toCoarseId(id2);
  if (tmp_id1 < 0 || tmp_id2 < 0) return false;
  Eigen::Vector3i idx1, idx2;
  grid1_->adrToIndex(tmp_id1, idx1);
  grid1_->adrToIndex(tmp_id2, idx2);

  for (int i = 0; i < 3; ++i) {
    if (abs(idx1[i] - idx2[i]) > 1) return false;
  }
  return true;

  // int diff = abs(tmp_id1 - tmp_id2);
  // if (diff == 1 || diff == grid1_->grid_num_[1]) return true;
  // return false;
}

bool HGrid::inSameLevel1(const int& id1, const int& id2) {
  return toCoarseId(id1) == toCoarseId(id2);
}

int HGrid::toCoarseId(const int& id) const {
  return (id >= 0 && id < static_cast<int>(grid1_->grid_data_.size())) ? id : -1;
}

double HGrid::getCoarseUnknownRatio(const int& id) {
  int coarse = toCoarseId(id);
  if (coarse < 0 || coarse >= static_cast<int>(grid1_->grid_data_.size())) return 0.0;
  const auto& grid = grid1_->grid_data_[coarse];
  double denom = static_cast<double>(grid.unknown_num_) + static_cast<double>(grid.free_num_);
  if (denom < 1.0) return 0.0;
  return static_cast<double>(grid.unknown_num_) / (denom + 1e-6);
}

int HGrid::getCoarseChebyshevDist(const int& id1, const int& id2) {
  int coarse1 = toCoarseId(id1);
  int coarse2 = toCoarseId(id2);
  if (coarse1 < 0 || coarse2 < 0) return std::numeric_limits<int>::max();

  Eigen::Vector3i idx1, idx2;
  grid1_->adrToIndex(coarse1, idx1);
  grid1_->adrToIndex(coarse2, idx2);

  int dx = abs(idx1[0] - idx2[0]);
  int dy = abs(idx1[1] - idx2[1]);
  int dz = abs(idx1[2] - idx2[2]);

  int dist = dx;
  if (dy > dist) dist = dy;
  if (dz > dist) dist = dz;
  return dist;
}

void HGrid::getFrontiersInGrid(const vector<int>& grid_ids, vector<int>& ftr_ids) {
  ftr_ids.clear();
  if (grid_ids.empty()) return;

  const int coarse = toCoarseId(grid_ids.front());
  if (coarse < 0 || coarse >= static_cast<int>(grid1_->grid_data_.size())) return;
  auto& grid = grid1_->grid_data_[coarse];
  for (auto pair : grid.contained_frontier_ids_) ftr_ids.push_back(pair.first);
}

// Get grid marker for specific grid ids (for partition visualization)
void HGrid::getGridMarker(
    const vector<int>& grid_ids, vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2) {
  pts1.clear();
  pts2.clear();

  for (const int& id : grid_ids) {
    const int coarse = toCoarseId(id);
    if (coarse < 0 || coarse >= static_cast<int>(grid1_->grid_data_.size())) continue;
    auto* grid = &grid1_->grid_data_[coarse];
    if (grid->vertices_.size() < 4) continue;

    for (int i = 0; i < 4; ++i) {
      pts1.push_back(grid->vertices_[i]);
      pts2.push_back(grid->vertices_[(i + 1) % 4]);
    }
  }

  for (auto& pt : pts1) pt[2] = 0.5;
  for (auto& pt : pts2) pt[2] = 0.5;
}

void HGrid::getConnectivityGraphVisualizationData(const int& level,
    vector<Eigen::Vector3d>& unknown_edge_start, vector<Eigen::Vector3d>& unknown_edge_end,
    vector<Eigen::Vector3d>& free_edge_start, vector<Eigen::Vector3d>& free_edge_end,
    vector<Eigen::Vector3d>& portal_edge_start, vector<Eigen::Vector3d>& portal_edge_end,
    vector<Eigen::Vector3d>& active_unknown_nodes, vector<Eigen::Vector3d>& active_free_nodes,
    vector<Eigen::Vector3d>& inactive_unknown_nodes, vector<Eigen::Vector3d>& inactive_free_nodes,
    vector<Eigen::Vector3d>& edge_cost_pos, vector<double>& edge_costs) {
  unknown_edge_start.clear();
  unknown_edge_end.clear();
  free_edge_start.clear();
  free_edge_end.clear();
  portal_edge_start.clear();
  portal_edge_end.clear();
  active_unknown_nodes.clear();
  active_free_nodes.clear();
  inactive_unknown_nodes.clear();
  inactive_free_nodes.clear();
  edge_cost_pos.clear();
  edge_costs.clear();

  if (level != 1) return;

  UniformGrid* grid = grid1_.get();
  if (!grid) return;

  auto connectivity_graph = grid->getConnectivityGraph();
  if (!connectivity_graph) return;

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;
  std::vector<ConnectivityEdge::TYPE> edge_types;
  std::vector<double> local_edge_costs;
  connectivity_graph->getFullConnectivityGraph(edges, edge_types, local_edge_costs);

  const size_t edge_num = std::min(edges.size(), edge_types.size());
  if (edges.size() != edge_types.size()) {
    ROS_WARN_THROTTLE(1.0,
        "[HGrid] Connectivity vis size mismatch: edges=%zu edge_types=%zu costs=%zu",
        edges.size(), edge_types.size(), local_edge_costs.size());
  }
  for (size_t i = 0; i < edge_num; ++i) {
    const auto& p1 = edges[i].first;
    const auto& p2 = edges[i].second;
    if (edge_types[i] == ConnectivityEdge::TYPE::UNKNOWN) {
      unknown_edge_start.push_back(p1);
      unknown_edge_end.push_back(p2);
    } else if (edge_types[i] == ConnectivityEdge::TYPE::FREE) {
      free_edge_start.push_back(p1);
      free_edge_end.push_back(p2);
    } else if (edge_types[i] == ConnectivityEdge::TYPE::PORTAL) {
      portal_edge_start.push_back(p1);
      portal_edge_end.push_back(p2);
    }

    edge_cost_pos.push_back(0.5 * (p1 + p2) + Eigen::Vector3d(0.0, 0.0, 0.15));
    edge_costs.push_back((i < local_edge_costs.size()) ? local_edge_costs[i] : 0.0);
  }

  std::vector<Eigen::Vector3d> node_positions;
  std::vector<int> node_ids;
  connectivity_graph->getNodePositionsWithIDs(node_positions, node_ids);

  std::unordered_set<int> active_free_node_ids;
  std::unordered_set<int> active_unknown_node_ids;
  for (int gid = 0; gid < static_cast<int>(grid->grid_data_.size()); ++gid) {
    const auto& local_grid = grid->grid_data_[gid];
    for (int idx : local_grid.centers_free_active_idx_) {
      if (idx < 0 || idx >= static_cast<int>(local_grid.centers_free_.size())) continue;
      const int node_id = grid->getConnectivityNodeId(gid, idx);
      if (node_id >= 0) active_free_node_ids.insert(node_id);
    }
    for (int idx : local_grid.centers_unknown_active_idx_) {
      if (idx < 0 || idx >= static_cast<int>(local_grid.centers_unknown_.size())) continue;
      const int local_center_idx = static_cast<int>(local_grid.centers_free_.size()) + idx;
      const int node_id = grid->getConnectivityNodeId(gid, local_center_idx);
      if (node_id >= 0) active_unknown_node_ids.insert(node_id);
    }
  }

  for (int i = 0; i < static_cast<int>(node_positions.size()); ++i) {
    const int node_id = node_ids[i];
    auto node = connectivity_graph->getNode(node_id);
    if (!node) continue;
    if (node->type_ == ConnectivityNode::TYPE::FREE) {
      if (active_free_node_ids.find(node_id) != active_free_node_ids.end()) {
        active_free_nodes.push_back(node_positions[i]);
      } else {
        inactive_free_nodes.push_back(node_positions[i]);
      }
    } else {
      if (active_unknown_node_ids.find(node_id) != active_unknown_node_ids.end()) {
        active_unknown_nodes.push_back(node_positions[i]);
      } else {
        inactive_unknown_nodes.push_back(node_positions[i]);
      }
    }
  }
}

}  // namespace c2_expl
