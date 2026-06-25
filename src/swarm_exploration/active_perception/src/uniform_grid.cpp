#include <active_perception/uniform_grid.h>
#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_env/raycast.h>
#include <plan_env/multi_map_manager.h>
#include <unordered_set>
#include <path_searching/matrix_hash.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace c2_expl {
namespace {
constexpr int kConvexSplitMinClusterPts = 20;
constexpr double kConvexSplitMinClusterRatio = 0.10;
constexpr int kConvexSplitMaxParts = 4;
constexpr int kConvexSplitMaxDepth = 3;
constexpr double kConvexSplitHullAreaRatio = 1.55;

struct Pt2 {
  double x = 0.0;
  double y = 0.0;
};

std::vector<Pt2> buildConvexHull2D(std::vector<Pt2> pts) {
  std::vector<Pt2> hull;
  if (pts.empty()) return hull;

  std::sort(pts.begin(), pts.end(), [](const Pt2& a, const Pt2& b) {
    if (a.x == b.x) return a.y < b.y;
    return a.x < b.x;
  });

  std::vector<Pt2> unique_pts;
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

  if (pts.size() <= 1) return pts;

  auto cross = [](const Pt2& a, const Pt2& b, const Pt2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  };

  std::vector<Pt2> lower, upper;
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

  hull.reserve(lower.size() + upper.size());
  for (const auto& p : lower) hull.push_back(p);
  for (size_t i = 1; i + 1 < upper.size(); ++i) hull.push_back(upper[i]);
  return hull;
}

double hullArea2D(const std::vector<Pt2>& hull) {
  if (hull.size() < 3) return 0.0;
  double area2 = 0.0;
  for (size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i];
    const auto& b = hull[(i + 1) % hull.size()];
    area2 += a.x * b.y - a.y * b.x;
  }
  return 0.5 * std::abs(area2);
}

Pt2 hullCentroid2D(const std::vector<Pt2>& hull) {
  Pt2 c;
  if (hull.empty()) return c;
  if (hull.size() == 1) return hull.front();
  if (hull.size() == 2) {
    c.x = 0.5 * (hull[0].x + hull[1].x);
    c.y = 0.5 * (hull[0].y + hull[1].y);
    return c;
  }

  double area2 = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  for (size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i];
    const auto& b = hull[(i + 1) % hull.size()];
    const double cross = a.x * b.y - b.x * a.y;
    area2 += cross;
    cx += (a.x + b.x) * cross;
    cy += (a.y + b.y) * cross;
  }

  if (std::abs(area2) < 1e-9) {
    for (const auto& p : hull) {
      c.x += p.x;
      c.y += p.y;
    }
    c.x /= static_cast<double>(hull.size());
    c.y /= static_cast<double>(hull.size());
    return c;
  }

  c.x = cx / (3.0 * area2);
  c.y = cy / (3.0 * area2);
  return c;
}

bool splitByKMeans2(
    const std::vector<Pt2>& pts, const std::vector<int>& in_ids,
    std::vector<int>& out0, std::vector<int>& out1) {
  out0.clear();
  out1.clear();
  if (in_ids.size() < 2) return false;

  int seed0 = 0;
  int seed1 = 1;
  double far = -1.0;
  for (int i = 1; i < static_cast<int>(in_ids.size()); ++i) {
    const auto& p = pts[in_ids[i]];
    const auto& q = pts[in_ids[seed0]];
    const double dx = p.x - q.x;
    const double dy = p.y - q.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 > far) {
      far = d2;
      seed1 = i;
    }
  }
  if (seed1 == seed0) return false;

  Pt2 c0 = pts[in_ids[seed0]];
  Pt2 c1 = pts[in_ids[seed1]];
  std::vector<int> labels(in_ids.size(), -1);
  bool changed = true;
  for (int iter = 0; iter < 10 && changed; ++iter) {
    changed = false;
    double s0x = 0.0, s0y = 0.0, s1x = 0.0, s1y = 0.0;
    int n0 = 0, n1 = 0;
    for (int i = 0; i < static_cast<int>(in_ids.size()); ++i) {
      const auto& p = pts[in_ids[i]];
      const double d0 = (p.x - c0.x) * (p.x - c0.x) + (p.y - c0.y) * (p.y - c0.y);
      const double d1 = (p.x - c1.x) * (p.x - c1.x) + (p.y - c1.y) * (p.y - c1.y);
      const int label = (d0 <= d1) ? 0 : 1;
      if (labels[i] != label) {
        labels[i] = label;
        changed = true;
      }
      if (label == 0) {
        s0x += p.x;
        s0y += p.y;
        ++n0;
      } else {
        s1x += p.x;
        s1y += p.y;
        ++n1;
      }
    }
    if (n0 == 0 || n1 == 0) return false;
    c0.x = s0x / n0;
    c0.y = s0y / n0;
    c1.x = s1x / n1;
    c1.y = s1y / n1;
  }

  for (int i = 0; i < static_cast<int>(in_ids.size()); ++i) {
    if (labels[i] == 0) {
      out0.push_back(in_ids[i]);
    } else if (labels[i] == 1) {
      out1.push_back(in_ids[i]);
    }
  }
  return !out0.empty() && !out1.empty();
}
}  // namespace

UniformGrid::UniformGrid(
    const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh, const int& level) {

  this->edt_ = edt;

  // Read min, max, resolution here
  nh.param("sdf_map/box_min_x", min_[0], 0.0);
  nh.param("sdf_map/box_min_y", min_[1], 0.0);
  nh.param("sdf_map/box_min_z", min_[2], 0.0);
  nh.param("sdf_map/box_max_x", max_[0], 0.0);
  nh.param("sdf_map/box_max_y", max_[1], 0.0);
  nh.param("sdf_map/box_max_z", max_[2], 0.0);

  nh.param("partitioning/z_samples", z_samples_, 5);
  if (z_samples_ < 1) z_samples_ = 1;
  nh.param("partitioning/w_unknown", w_unknown_, 3.5);
  nh.param("partitioning/ccl_step", ccl_step_, 1);
  nh.param("partitioning/grid_size", grid_size_, 5.0);
  nh.param("partitioning/cg_astar_res", cg_astar_resolution_, -1.0);
  nh.param("partitioning/convex_split_hull_area_ratio", convex_split_hull_area_ratio_,
      kConvexSplitHullAreaRatio);

  auto size = max_ - min_;

  // resolution_ = size / 3;
  for (int i = 0; i < 2; ++i) {
    int num = ceil(size[i] / grid_size_);
    resolution_[i] = size[i] / double(num);
    for (int j = 1; j < level; ++j) resolution_[i] *= 0.5;
  }
  resolution_[2] = size[2];
  initialized_ = false;
  level_ = level;

  path_finder_.reset(new Astar);
  path_finder_->init(nh, edt);

  caster_.reset(new RayCaster);
  resolution = edt_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size_map;
  edt_->sdf_map_->getRegion(origin, size_map);
  caster_->setParams(resolution, origin);

  if (cg_astar_resolution_ <= 0.0) {
    cg_astar_resolution_ = resolution;
  }

  connectivity_graph_ = std::make_shared<ConnectivityGraph>();
}

UniformGrid::~UniformGrid() {
}

bool UniformGrid::sameFrontierIdSet(
    const std::unordered_map<int, int>& a, const std::unordered_map<int, int>& b) {
  if (a.size() != b.size()) return false;
  for (const auto& kv : a) {
    if (b.find(kv.first) == b.end()) return false;
  }
  return true;
}

bool UniformGrid::boxesOverlap2D(const Eigen::Vector3d& min1, const Eigen::Vector3d& max1,
    const Eigen::Vector3d& min2, const Eigen::Vector3d& max2) {
  for (int m = 0; m < 2; ++m) {
    double bmin = max(min1[m], min2[m]);
    double bmax = min(max1[m], max2[m]);
    if (bmin > bmax + 1e-3) return false;
  }
  return true;
}

const char* UniformGrid::gridStateName(const GridState& state) {
  switch (state) {
    case EXPLORABLE:
      return "EXPLORABLE";
    case EXPLORING:
      return "EXPLORING";
    case EXPLORED:
      return "EXPLORED";
    case UNREACHABLE:
      return "UNREACHABLE";
    default:
      return "UNKNOWN";
  }
}

void UniformGrid::initGridData() {
  Eigen::Vector3d size = max_ - min_;
  for (int i = 0; i < 3; ++i) grid_num_(i) = ceil(size(i) / resolution_[i]);
  grid_data_.resize(grid_num_[0] * grid_num_[1] * grid_num_[2]);
  ccl_voxels_addr_.resize(grid_data_.size());
  ccl_voxels_color_.resize(grid_data_.size());
  ccl_free_unknown_states_and_centers_idx_.resize(grid_data_.size());

  std::cout << "data size: " << grid_data_.size() << std::endl;
  std::cout << "grid num: " << grid_num_.transpose() << std::endl;
  std::cout << "resolution: " << resolution_.transpose() << std::endl;

  // Init each grid info
  for (int x = 0; x < grid_num_[0]; ++x) {
    for (int y = 0; y < grid_num_[1]; ++y) {
      for (int z = 0; z < grid_num_[2]; ++z) {
        Eigen::Vector3i id(x, y, z);
        auto& grid = grid_data_[toAddress(id)];

        Eigen::Vector3d pos;
        indexToPos(id, 0.5, pos);
        if (use_swarm_tf_) {
          pos = rot_sw_ * pos + trans_sw_;
        }

        grid.center_ = pos;
        grid.unknown_num_ = resolution_[0] * resolution_[1] * resolution_[2] /
                            pow(edt_->sdf_map_->getResolution(), 3);
        grid.free_num_ = 0;

        grid.is_prev_relevant_ = true;
        grid.is_cur_relevant_ = true;
        grid.is_reachable_ = true;
        grid.need_divide_ = false;
        grid.was_in_tour_ = false;
        grid.state_ = EXPLORABLE;
        if (level_ == 1)
          grid.active_ = true;
        else
          grid.active_ = false;
      }
    }
  }

  if (!connectivity_graph_) {
    connectivity_graph_ = std::make_shared<ConnectivityGraph>();
  }
  connectivity_graph_->clearNodes();
  for (int i = 0; i < grid_data_.size(); ++i) {
    auto& grid = grid_data_[i];
    grid.centers_free_.clear();
    grid.centers_unknown_.clear();
    grid.centers_free_active_.clear();
    grid.centers_unknown_active_.clear();
    grid.centers_free_active_idx_.clear();
    grid.centers_unknown_active_idx_.clear();
    if (level_ == 1 || grid.active_) {
      // Initialize each active cell with a single unknown center at the grid center.
      grid.centers_unknown_.push_back(grid.center_);
      grid.centers_unknown_active_.push_back(grid.center_);
      grid.centers_unknown_active_idx_.push_back(0);

      ConnectivityNode::Ptr node = std::make_shared<ConnectivityNode>(
          makeNodeId(i, 0), grid.center_, ConnectivityNode::TYPE::UNKNOWN);
      connectivity_graph_->addNode(node);
    }
  }

  // Initialize neighbor edges between grid centers (UNKNOWN type).
  for (int i = 0; i < grid_data_.size(); ++i) {
    auto& grid = grid_data_[i];
    std::vector<int> neighbor_ids;
    getNearbyGridIds(i, neighbor_ids);
    for (int neighbor_id : neighbor_ids) {
      auto& neighbor_grid = grid_data_[neighbor_id];
      ConnectivityNode::Ptr node = connectivity_graph_->getNode(makeNodeId(i, 0));
      ConnectivityNode::Ptr node_neighbor =
          connectivity_graph_->getNode(makeNodeId(neighbor_id, 0));
      if (!node || !node_neighbor) continue;

      std::vector<Eigen::Vector3d> path;
      path.push_back(grid.center_);
      path.push_back(neighbor_grid.center_);
      std::vector<Eigen::Vector3d> reverse_path(path.rbegin(), path.rend());
      double cost = (grid.center_ - neighbor_grid.center_).norm();

      node->addNeighborWithPath(
          makeNodeId(neighbor_id, 0), cost, ConnectivityEdge::TYPE::UNKNOWN, path);
      node_neighbor->addNeighborWithPath(
          makeNodeId(i, 0), cost, ConnectivityEdge::TYPE::UNKNOWN, reverse_path);
    }
  }
}

void UniformGrid::updateBaseCoor() {
  for (int i = 0; i < grid_data_.size(); ++i) {
    auto& grid = grid_data_[i];
    // if (!grid.active_) continue;

    Eigen::Vector3i id;
    adrToIndex(i, id);

    // Compute vertices and box of grid in current drone's frame
    Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;
    indexToPos(id, 0.0, left_bottom);
    indexToPos(id, 1.0, right_top);
    left_top[0] = left_bottom[0];
    left_top[1] = right_top[1];
    left_top[2] = left_bottom[2];
    right_bottom[0] = right_top[0];
    right_bottom[1] = left_bottom[1];
    right_bottom[2] = left_bottom[2];
    right_top[2] = left_bottom[2];

    vector<Eigen::Vector3d> vertices = { left_bottom, right_bottom, right_top, left_top };
    if (use_swarm_tf_) {
      for (auto& vert : vertices) vert = rot_sw_ * vert + trans_sw_;
    }

    Eigen::Vector3d vmin, vmax;
    vmin = vmax = vertices[0];
    for (int j = 1; j < vertices.size(); ++j) {
      for (int k = 0; k < 2; ++k) {
        vmin[k] = min(vmin[k], vertices[j][k]);
        vmax[k] = max(vmax[k], vertices[j][k]);
      }
    }
    grid.vertices_ = vertices;
    grid.vmin_ = vmin;
    grid.vmax_ = vmax;

    // Compute normals of four separating lines
    grid.normals_.clear();
    for (int j = 0; j < 4; ++j) {
      Eigen::Vector3d dir = (vertices[(j + 1) % 4] - vertices[j]).normalized();
      grid.normals_.push_back(dir);
    }
    // std::cout << "Vertices of grid " << toAddress(id) << std::endl;
    // for (auto v : grid.vertices_)
    //   std::cout << v.transpose() << "; ";
    // std::cout << "\nNormals: " << std::endl;
    // for (auto n : grid.normals_)
    //   std::cout << n.transpose() << "; ";
    // std::cout << "\nbox: " << grid.vmin_.transpose() << ", " << grid.vmax_.transpose()
    //           << std::endl;
  }
}

std::vector<std::unordered_set<int>> UniformGrid::splitUnknownComponentByConvexity(
    const std::unordered_set<int>& voxel_addrs, const int step) const {
  std::vector<std::unordered_set<int>> split_addrs;
  if (voxel_addrs.empty()) return split_addrs;

  std::vector<int> addr_list(voxel_addrs.begin(), voxel_addrs.end());
  std::vector<Pt2> pts;
  pts.reserve(addr_list.size());
  for (const int addr : addr_list) {
    Eigen::Vector3i idx;
    edt_->sdf_map_->addressToIndex(addr, idx);
    Eigen::Vector3d pos;
    edt_->sdf_map_->indexToPos(idx, pos);
    pts.push_back({ pos.x(), pos.y() });
  }

  const double res = edt_->sdf_map_->getResolution();
  const double sample_area = res * res * std::max(1, step) * std::max(1, step);
  const int min_cluster_pts =
      std::max(kConvexSplitMinClusterPts,
          static_cast<int>(addr_list.size() * kConvexSplitMinClusterRatio));

  struct PendingCluster {
    std::vector<int> ids;
    int depth = 0;
  };

  std::vector<std::vector<int>> clusters;
  std::vector<PendingCluster> stack_ids;

  PendingCluster root;
  root.ids.resize(addr_list.size());
  std::iota(root.ids.begin(), root.ids.end(), 0);
  stack_ids.push_back(std::move(root));

  while (!stack_ids.empty()) {
    PendingCluster cur = std::move(stack_ids.back());
    stack_ids.pop_back();

    std::vector<Pt2> cur_pts;
    cur_pts.reserve(cur.ids.size());
    for (const int id : cur.ids) cur_pts.push_back(pts[id]);

    const auto hull = buildConvexHull2D(cur_pts);
    const double hull_area = hullArea2D(hull);
    const double occupancy_area = cur.ids.size() * sample_area;
    const double ratio = hull_area / std::max(occupancy_area, sample_area);

    bool can_split = true;
    if (cur.depth >= kConvexSplitMaxDepth) can_split = false;
    if (static_cast<int>(cur.ids.size()) < 2 * min_cluster_pts) can_split = false;
    if (ratio <= convex_split_hull_area_ratio_) can_split = false;
    if (static_cast<int>(clusters.size() + stack_ids.size() + 1) >= kConvexSplitMaxParts) {
      can_split = false;
    }

    std::vector<int> group0, group1;
    if (can_split && splitByKMeans2(pts, cur.ids, group0, group1) &&
        static_cast<int>(group0.size()) >= min_cluster_pts &&
        static_cast<int>(group1.size()) >= min_cluster_pts) {
      stack_ids.push_back({ std::move(group0), cur.depth + 1 });
      stack_ids.push_back({ std::move(group1), cur.depth + 1 });
    } else {
      clusters.push_back(std::move(cur.ids));
    }
  }

  split_addrs.reserve(clusters.size());
  for (const auto& ids : clusters) {
    std::unordered_set<int> addr_set;
    addr_set.reserve(ids.size());
    for (const int id : ids) addr_set.insert(addr_list[id]);
    split_addrs.push_back(std::move(addr_set));
  }

  if (split_addrs.empty()) split_addrs.push_back(voxel_addrs);
  return split_addrs;
}

Eigen::Vector3d UniformGrid::projectToNearestUnknownVoxel(
    const std::unordered_set<int>& voxel_addrs, const Eigen::Vector3d& ref_center) const {
  if (voxel_addrs.empty()) return ref_center;

  double best_d2 = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best = ref_center;
  for (const int addr : voxel_addrs) {
    Eigen::Vector3i idx;
    edt_->sdf_map_->addressToIndex(addr, idx);
    Eigen::Vector3d pos;
    edt_->sdf_map_->indexToPos(idx, pos);
    const double d2 = (pos - ref_center).squaredNorm();
    if (d2 < best_d2) {
      best_d2 = d2;
      best = pos;
    }
  }
  return best;
}

void UniformGrid::getCCLcenters(const int& grid_id, std::vector<Vector3d>& centers_free,
    std::vector<Vector3d>& centers_unknown, std::vector<int>& free_nums,
    std::vector<int>& unknown_nums) {
  centers_free.clear();
  centers_unknown.clear();
  free_nums.clear();
  unknown_nums.clear();
  ccl_voxels_addr_[grid_id].clear();
  ccl_voxels_color_[grid_id].clear();

  double height = 1.0;

  auto& grid = grid_data_[grid_id];
  Eigen::Vector3d bbox_min = grid.vmin_;
  Eigen::Vector3d bbox_max = grid.vmax_;

  // If vertices were transformed by swarm TF, transform bbox back to map frame
  if (use_swarm_tf_) {
    Eigen::Matrix3d Rt = rot_sw_.transpose();
    Eigen::Vector3d t_inv = -Rt * trans_sw_;
    bbox_min = Rt * bbox_min + t_inv;
    bbox_max = Rt * bbox_max + t_inv;
  }

  bbox_min.z() = height;
  bbox_max.z() = height;

  Eigen::Vector3i bbox_min_idx, bbox_max_idx;
  edt_->sdf_map_->posToIndex(bbox_min, bbox_min_idx);
  edt_->sdf_map_->posToIndex(bbox_max, bbox_max_idx);

  int size_x = bbox_max_idx.x() - bbox_min_idx.x();
  int size_y = bbox_max_idx.y() - bbox_min_idx.y();

  if (size_x <= 0 || size_y <= 0) return;

  std::vector<int> labels(size_x * size_y, 0);
  int step = ccl_step_;
  int freeSpaces = 0;
  int label = 1;
  int index_z = bbox_min_idx.z();
  int free_unknown_state = -1;  // 0: free, 1: unknown

  std::vector<std::pair<int, int>>& free_unknown_states_and_centers_idx =
      ccl_free_unknown_states_and_centers_idx_[grid_id];
  free_unknown_states_and_centers_idx.clear();

  auto isFree = [&](const Eigen::Vector3i& idx) {
    free_unknown_state = 0;
    return edt_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
  };

  auto isFreeInflated = [&](const Eigen::Vector3i& idx) {
    free_unknown_state = 0;
    return edt_->sdf_map_->getOccupancy(idx) == SDFMap::FREE &&
           edt_->sdf_map_->getInflateOccupancy(idx) != 1;
  };

  auto isUnknown = [&](const Eigen::Vector3i& idx) {
    free_unknown_state = 1;
    return edt_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN;
  };

  for (int y = 0; y < size_y; y = y + step) {
    for (int x = 0; x < size_x; x = x + step) {
      free_unknown_state = -1;
      int index = y * size_x + x;
      Eigen::Vector3i current_idx(x + bbox_min_idx.x(), y + bbox_min_idx.y(), index_z);

      if (labels[index] == 0 && (isFree(current_idx) || isUnknown(current_idx))) {
        std::stack<int> stack;
        stack.push(index);
        labels[index] = label;
        const int comp_state = free_unknown_state;  // 0: free, 1: unknown

        int num_voxels = 0;
        double X_avg = 0.0;
        double Y_avg = 0.0;

        std::unordered_set<int> voxel_addrs;

        while (!stack.empty()) {
          num_voxels++;
          int currIndex = stack.top();
          int currY = currIndex / size_x;
          int currX = currIndex - currY * size_x;
          Eigen::Vector3i curr_voxel_idx(
              currX + bbox_min_idx.x(), currY + bbox_min_idx.y(), index_z);
          voxel_addrs.insert(edt_->sdf_map_->toAddress(curr_voxel_idx));

          X_avg += currX;
          Y_avg += currY;

          stack.pop();
          for (int dy = -step; dy <= step; dy += step) {
            for (int dx = -step; dx <= step; dx += step) {
              if (dx == 0 && dy == 0) continue;
              int nx = currX + dx;
              int ny = currY + dy;

              if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y) continue;

              int neighborIndex = ny * size_x + nx;
              Eigen::Vector3i neighbor_voxel_idx(
                  nx + bbox_min_idx.x(), ny + bbox_min_idx.y(), index_z);

              if (labels[neighborIndex] == 0 && comp_state == 0 && isFree(neighbor_voxel_idx)) {
                bool isMiddleVoxelFree = true;
                for (int ddy = 0; ddy < step; ddy++) {
                  for (int ddx = 0; ddx < step; ddx++) {
                    if (ddx == 0 && ddy == 0) continue;
                    int mx = currX + ddx;
                    int my = currY + ddy;
                    Eigen::Vector3i mid_idx(mx + bbox_min_idx.x(), my + bbox_min_idx.y(), index_z);
                    if (!isFreeInflated(mid_idx)) {
                      isMiddleVoxelFree = false;
                      continue;
                    }
                  }
                }

                if (!isMiddleVoxelFree) continue;

                stack.push(neighborIndex);
                labels[neighborIndex] = label;
              }

              if (labels[neighborIndex] == 0 && comp_state == 1 && isUnknown(neighbor_voxel_idx)) {
                bool isMiddleVoxelUnknown = true;
                for (int ddy = 0; ddy < step; ddy++) {
                  for (int ddx = 0; ddx < step; ddx++) {
                    if (ddx == 0 && ddy == 0) continue;
                    int mx = currX + ddx;
                    int my = currY + ddy;
                    Eigen::Vector3i mid_idx(mx + bbox_min_idx.x(), my + bbox_min_idx.y(), index_z);
                    if (!isUnknown(mid_idx)) {
                      isMiddleVoxelUnknown = false;
                      continue;
                    }
                  }
                }

                if (!isMiddleVoxelUnknown) continue;

                stack.push(neighborIndex);
                labels[neighborIndex] = label;
              }
            }
          }
        }

        X_avg /= (double)num_voxels;
        Y_avg /= (double)num_voxels;

        if (num_voxels < 10) continue;
        if (comp_state == 1 && num_voxels < 0.1 * size_x * size_y / pow(step, 2)) continue;

        Eigen::Vector3d voxel_color_free =
            Eigen::Vector3d(227.0 / 255.0, 187.0 / 255.0, 247.0 / 255.0);
        Eigen::Vector3d voxel_color_unknown =
            Eigen::Vector3d(230.0 / 255.0, 230.0 / 255.0, 230.0 / 255.0);

        Eigen::Vector3d min_pos_center;
        edt_->sdf_map_->indexToPos(bbox_min_idx, min_pos_center);
        Eigen::Vector3d center_pos(min_pos_center.x() + X_avg * edt_->sdf_map_->getResolution(),
            min_pos_center.y() + Y_avg * edt_->sdf_map_->getResolution(), height);

        if (comp_state == 0) {
          ccl_voxels_addr_[grid_id].push_back(voxel_addrs);
          free_unknown_states_and_centers_idx.push_back(
              std::make_pair(comp_state, centers_free.size()));
          centers_free.push_back(center_pos);
          free_nums.push_back(num_voxels);
          ccl_voxels_color_[grid_id].push_back(voxel_color_free);
        } else if (comp_state == 1) {
          // Unknown-only components can be concave. Further split by convexity so one center is
          // generated for each convex sub-region.
          auto unknown_sub_components = splitUnknownComponentByConvexity(voxel_addrs, step);
          for (const auto& sub_voxel_addrs : unknown_sub_components) {
            if (sub_voxel_addrs.empty()) continue;

            std::vector<Pt2> sub_pts;
            sub_pts.reserve(sub_voxel_addrs.size());
            int sub_num = 0;
            for (const int addr : sub_voxel_addrs) {
              Eigen::Vector3i idx;
              edt_->sdf_map_->addressToIndex(addr, idx);
              Eigen::Vector3d pos;
              edt_->sdf_map_->indexToPos(idx, pos);
              sub_pts.push_back({ pos.x(), pos.y() });
              ++sub_num;
            }
            if (sub_num <= 0) continue;

            const auto sub_hull = buildConvexHull2D(sub_pts);
            const Pt2 c2 = sub_hull.empty() ? Pt2{ sub_pts.front().x, sub_pts.front().y } :
                                              hullCentroid2D(sub_hull);
            const Eigen::Vector3d sub_center_ref(c2.x, c2.y, height);
            const Eigen::Vector3d sub_center =
                projectToNearestUnknownVoxel(sub_voxel_addrs, sub_center_ref);

            ccl_voxels_addr_[grid_id].push_back(sub_voxel_addrs);
            free_unknown_states_and_centers_idx.push_back(
                std::make_pair(comp_state, centers_unknown.size()));
            centers_unknown.push_back(sub_center);
            unknown_nums.push_back(sub_num);
            ccl_voxels_color_[grid_id].push_back(voxel_color_unknown);
          }
        }
        freeSpaces++;
        label++;
      }
    }
  }

  int ccl_voxels_idx = 0;
  for (const auto& pair : free_unknown_states_and_centers_idx) {
    if (ccl_voxels_idx >= static_cast<int>(ccl_voxels_addr_[grid_id].size())) {
      ROS_WARN("[UniformGrid] CCL voxel/component size mismatch in grid %d.", grid_id);
      break;
    }

    const int component_idx = ccl_voxels_idx++;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    const std::unordered_set<int>& voxel_addr_set = ccl_voxels_addr_[grid_id][component_idx];
    if (voxel_addr_set.empty()) {
      ROS_WARN(
          "[UniformGrid] Empty CCL voxel set in grid %d, component %d.", grid_id, component_idx);
      continue;
    }
    for (const int& voxel_addr : voxel_addr_set) {
      Eigen::Vector3i voxel_idx;
      edt_->sdf_map_->addressToIndex(voxel_addr, voxel_idx);

      Eigen::Vector3d voxel_pos;
      edt_->sdf_map_->indexToPos(voxel_idx, voxel_pos);
      cloud->push_back(pcl::PointXYZ(voxel_pos.x(), voxel_pos.y(), voxel_pos.z()));
    }

    if (cloud->empty()) {
      ROS_WARN("[UniformGrid] Empty CCL cloud in grid %d, component %d.", grid_id, component_idx);
      continue;
    }

    kdtree.setInputCloud(cloud);

    Eigen::Vector3d center =
        pair.first == 0 ? centers_free[pair.second] : centers_unknown[pair.second];
    pcl::PointXYZ searchPoint(center.x(), center.y(), center.z());

    // Unknown centers are already projected onto their own UNKNOWN sub-component voxels.
    // Only free centers still need KNN rectification here.
    if (pair.first == 1) continue;

    // Search for more neighbors to find a valid center not in inflate occ
    int K_search = std::min(10, static_cast<int>(cloud->size()));
    if (K_search <= 0) continue;
    std::vector<int> pointIdxKSearch(K_search);
    std::vector<float> pointKSquaredDistance(K_search);

    if (kdtree.nearestKSearch(searchPoint, K_search, pointIdxKSearch, pointKSquaredDistance) > 0) {
      cloud->clear();
      std::vector<Eigen::Vector3i> cloud_indices;
      cloud_indices.reserve(voxel_addr_set.size());

      for (const int& voxel_addr : voxel_addr_set) {
        Eigen::Vector3i voxel_idx;
        edt_->sdf_map_->addressToIndex(voxel_addr, voxel_idx);

        if (edt_->sdf_map_->getInflateOccupancy(voxel_idx) == 1) continue;
        if (pair.first == 0 && edt_->sdf_map_->getOccupancy(voxel_idx) != SDFMap::FREE) continue;
        if (pair.first == 1 && edt_->sdf_map_->getOccupancy(voxel_idx) != SDFMap::UNKNOWN) continue;

        Eigen::Vector3d voxel_pos;
        edt_->sdf_map_->indexToPos(voxel_idx, voxel_pos);
        cloud->push_back(pcl::PointXYZ(voxel_pos.x(), voxel_pos.y(), voxel_pos.z()));
        cloud_indices.push_back(voxel_idx);
      }

      if (cloud->empty()) {
        ROS_WARN("[UniformGrid] No non-inflated FREE voxels in grid %d, component %d.", grid_id,
            component_idx);
      } else {
        kdtree.setInputCloud(cloud);

        int K_search =
            std::min(50, static_cast<int>(cloud->size()));  // 10 is too small; use a wider search.
        if (K_search <= 0) continue;

        std::vector<int> nn(K_search);
        std::vector<float> dist2(K_search);

        if (kdtree.nearestKSearch(searchPoint, K_search, nn, dist2) > 0) {
          bool found = false;
          for (int k = 0; k < static_cast<int>(nn.size()); ++k) {
            if (nn[k] < 0 || nn[k] >= static_cast<int>(cloud_indices.size())) continue;
            const Eigen::Vector3i& idx = cloud_indices[nn[k]];

            // Strictly validate the final selected center.
            if (edt_->sdf_map_->getInflateOccupancy(idx) == 1) continue;
            if (pair.first == 0 && edt_->sdf_map_->getOccupancy(idx) != SDFMap::FREE) continue;
            if (pair.first == 1 && edt_->sdf_map_->getOccupancy(idx) != SDFMap::UNKNOWN) continue;

            Eigen::Vector3d pos;
            edt_->sdf_map_->indexToPos(idx, pos);
            center = pos;
            found = true;
            break;
          }
          if (!found) {
            ROS_WARN("[UniformGrid] No valid center found in KNN results, fallback needed.");
          }
        }
      }
    } else {
      ROS_ERROR("[UniformGrid] K nearest neighbor search failed!");
    }

    if (pair.first == 0) {
      centers_free[pair.second] = center;
    } else if (pair.first == 1) {
      centers_unknown[pair.second] = center;
    }
  }
}

void UniformGrid::updateGridData(const int& drone_id, vector<int>& grid_ids, vector<int>& parti_ids,
    vector<int>& parti_ids_all, const Eigen::Vector3d& pos) {

  // Reset per-call marks before collecting grids affected by map or frontier updates.
  for (auto& grid : grid_data_) {
    grid.is_updated_ = false;
  }
  parti_ids.clear();

  // Reset updated box every call to avoid accumulation; keep chunk boxes (merged multi-robot)
  // intact.
  bool reset_update_box = true;
  bool reset_chunk_boxes = (level_ == 1);
  Vector3d update_min, update_max;
  edt_->sdf_map_->getUpdatedBox(update_min, update_max, reset_update_box);

  vector<Eigen::Vector3d> update_mins, update_maxs;
  edt_->sdf_map_->mm_->getChunkBoxes(update_mins, update_maxs, reset_chunk_boxes);

  std::vector<int> updated_ids;
  std::vector<int> updated_old_counts;
  updated_ids.reserve(grid_data_.size());
  updated_old_counts.reserve(grid_data_.size());

  // Refresh only grids touched by map chunks/FOV, or grids whose frontier membership changed.
  for (int i = 0; i < grid_data_.size(); ++i) {
    auto& grid = grid_data_[i];
    if (!grid.active_) continue;

    bool overlap_with_fov = false;
    if (!shouldUpdateGrid(grid, update_min, update_max, update_mins, update_maxs,
            overlap_with_fov)) {
      continue;
    }

    Eigen::Vector3i idx;
    adrToIndex(i, idx);
    int old_count = grid.centers_free_.size() + grid.centers_unknown_.size();
    updateGridInfo(idx, pos);
    updated_ids.push_back(i);
    updated_old_counts.push_back(old_count);

    if (level_ > 1 && grid.need_divide_) {
      parti_ids_all.push_back(i);
      grid.active_ = false;
    }

    if (overlap_with_fov && !grid.is_prev_relevant_ && grid.is_cur_relevant_ && level_ > 1) {
      ROS_WARN("Grid %d is rediscovered", i);
    }
  }

  if (!updated_ids.empty()) {
    std::stringstream ss_update;
    ss_update << "[UniformGrid L" << level_ << "] Incremental CG update grid_ids: [";
    for (size_t i = 0; i < updated_ids.size(); ++i) {
      ss_update << updated_ids[i];
      if (i < updated_ids.size() - 1) ss_update << ", ";
    }
    ss_update << "] (count=" << updated_ids.size() << ")";
    // ROS_INFO_STREAM(ss_update.str());

    updateConnectivityGraph(updated_ids, updated_old_counts);

    // Connectivity filtering may remove active UNKNOWN centers, so task state is finalized after
    // the graph update.
    for (const int& cell_id : updated_ids) {
      finalizeUpdatedGridState(cell_id);
    }
  }

  rebuildRelevantGridList();
  updateAssignedGridIds(drone_id, grid_ids, parti_ids);
}

bool UniformGrid::shouldUpdateGrid(const GridInfo& grid, const Eigen::Vector3d& update_min,
    const Eigen::Vector3d& update_max, const vector<Eigen::Vector3d>& chunk_mins,
    const vector<Eigen::Vector3d>& chunk_maxs, bool& overlap_with_fov) const {
  bool overlap_with_chunk = false;
  for (int j = 0; j < chunk_mins.size(); ++j) {
    if (boxesOverlap2D(grid.vmin_, grid.vmax_, chunk_mins[j], chunk_maxs[j])) {
      overlap_with_chunk = true;
      break;
    }
  }

  overlap_with_fov = boxesOverlap2D(grid.vmin_, grid.vmax_, update_min, update_max);

  // Frontier ids drive the downstream frontier tour. Refresh the grid if membership changes even
  // when no map-updated box overlaps this cell.
  const bool frontier_changed =
      !sameFrontierIdSet(grid.contained_frontier_ids_, grid.frontier_cell_nums_);

  return overlap_with_chunk || overlap_with_fov || frontier_changed;
}

void UniformGrid::finalizeUpdatedGridState(const int& cell_id) {
  auto& grid = grid_data_[cell_id];

  const bool has_unknown_active = !grid.centers_unknown_active_.empty();
  const bool has_frontier = !grid.contained_frontier_ids_.empty();
  const bool has_free_active = !grid.centers_free_active_.empty();
  const bool frontier_recovery = has_frontier && has_free_active;
  const bool task_done = !has_unknown_active && !has_free_active;

  // A grid task is complete only after both UNKNOWN and frontier-backed FREE centers are gone.
  grid.is_reachable_ = has_unknown_active || frontier_recovery;
  grid.state_ = task_done ? EXPLORED : (has_frontier ? EXPLORING : EXPLORABLE);

  if (has_unknown_active) {
    grid.center_ = grid.centers_unknown_active_[0];
  } else if (frontier_recovery) {
    grid.center_ = rectifyCenter(grid.centers_free_active_[0], 1.0);
  }

  grid.is_cur_relevant_ = isRelevant(grid);
}

void UniformGrid::rebuildRelevantGridList() {
  relevant_id_.clear();
  relevant_map_.clear();

  std::stringstream ss;
  ss << "[UniformGrid L" << level_ << "] Grid States: ";
  for (int i = 0; i < grid_data_.size(); ++i) {
    auto& grid = grid_data_[i];
    ss << i << ":" << gridStateName(grid.state_) << " ";

    if (isRelevant(grid)) {
      relevant_id_.push_back(i);
      relevant_map_[i] = 1;
    }
  }
  ss << "| Relevant=" << relevant_id_.size();
  // ROS_INFO_STREAM(ss.str());
}

void UniformGrid::updateAssignedGridIds(
    const int& drone_id, vector<int>& grid_ids, vector<int>& parti_ids) {
  if (!initialized_) {
    if (drone_id == 1 && level_ == 1) grid_ids = relevant_id_;
    ROS_WARN("Init grid allocation.");
    initialized_ = true;
    return;
  }

  for (auto it = grid_ids.begin(); it != grid_ids.end();) {
    if (relevant_map_.find(*it) == relevant_map_.end()) {
      it = grid_ids.erase(it);
    } else if (level_ > 1 && grid_data_[*it].need_divide_) {
      parti_ids.push_back(*it);
      it = grid_ids.erase(it);
    } else {
      ++it;
    }
  }
}

void UniformGrid::getNearbyGridIds(const int& adr, std::vector<int>& neighbor_ids) {
  neighbor_ids.clear();
  Eigen::Vector3i current_id;
  adrToIndex(adr, current_id);

  const bool allow_diagonal = (level_ != 1);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      if (!allow_diagonal && (std::abs(dx) + std::abs(dy) != 1)) continue;
      Eigen::Vector3i neighbor_id = current_id + Eigen::Vector3i(dx, dy, 0);
      if (!insideGrid(neighbor_id)) continue;
      neighbor_ids.push_back(toAddress(neighbor_id));
    }
  }
}

bool UniformGrid::pathInsideBBox(const std::vector<Eigen::Vector3d>& path,
    const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max) const {
  for (const auto& p : path) {
    if (p.x() < bbox_min.x() - 1e-3 || p.x() > bbox_max.x() + 1e-3 || p.y() < bbox_min.y() - 1e-3 ||
        p.y() > bbox_max.y() + 1e-3) {
      return false;
    }
  }
  return true;
}

double UniformGrid::computeFreeCostBBox(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
    const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
    std::vector<Eigen::Vector3d>& path) {
  path.clear();
  // Fast line-of-sight check in bbox (free-only).
  auto isInBox = [&](const Eigen::Vector3d& p) {
    return p.x() >= bbox_min.x() && p.x() <= bbox_max.x() && p.y() >= bbox_min.y() &&
           p.y() <= bbox_max.y();
  };
  bool clear = true;
  Eigen::Vector3i idx;
  Eigen::Vector3d pos;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    edt_->sdf_map_->indexToPos(idx, pos);
    if (!isInBox(pos) || !edt_->sdf_map_->isInBox(idx)) {
      clear = false;
      break;
    }
    if (edt_->sdf_map_->getInflateOccupancy(idx) == 1 ||
        edt_->sdf_map_->getOccupancy(idx) != SDFMap::FREE) {
      clear = false;
      break;
    }
  }
  if (clear) {
    path.push_back(p1);
    path.push_back(p2);
    return (p1 - p2).norm();
  }

  path_finder_->reset();
  path_finder_->setResolution(cg_astar_resolution_);
  if (path_finder_->search(p1, p2, false) == Astar::REACH_END) {
    path = path_finder_->getPath();
    if (pathInsideBBox(path, bbox_min, bbox_max)) {
      return Astar::pathLength(path);
    }
  }
  return 1000.0;
}

double UniformGrid::computeUnknownCostBBox(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
    const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
    std::vector<Eigen::Vector3d>& path) {
  path.clear();
  // Fast line-of-sight check in bbox (unknown allowed).
  auto isInBox = [&](const Eigen::Vector3d& p) {
    return p.x() >= bbox_min.x() && p.x() <= bbox_max.x() && p.y() >= bbox_min.y() &&
           p.y() <= bbox_max.y();
  };
  bool clear = true;
  Eigen::Vector3i idx;
  Eigen::Vector3d pos;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    edt_->sdf_map_->indexToPos(idx, pos);
    if (!isInBox(pos) || !edt_->sdf_map_->isInBox(idx)) {
      clear = false;
      break;
    }
    if (edt_->sdf_map_->getInflateOccupancy(idx) == 1) {
      clear = false;
      break;
    }
  }
  if (clear) {
    path.push_back(p1);
    path.push_back(p2);
    return (p1 - p2).norm();
  }

  path_finder_->reset();
  path_finder_->setResolution(cg_astar_resolution_);
  if (path_finder_->searchUnknownBBox(p1, p2, bbox_min, bbox_max) == Astar::REACH_END) {
    path = path_finder_->getPath();
    return Astar::pathLength(path);
  }
  return 1000.0;
}

double UniformGrid::computeUnknownOnlyCostBBox(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
    const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
    std::vector<Eigen::Vector3d>& path) {
  path.clear();
  auto isInBox = [&](const Eigen::Vector3d& p) {
    return p.x() >= bbox_min.x() && p.x() <= bbox_max.x() && p.y() >= bbox_min.y() &&
           p.y() <= bbox_max.y();
  };

  bool clear = true;
  Eigen::Vector3i idx;
  Eigen::Vector3d pos;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    edt_->sdf_map_->indexToPos(idx, pos);
    if (!isInBox(pos) || !edt_->sdf_map_->isInBox(idx)) {
      clear = false;
      break;
    }
    if (edt_->sdf_map_->getInflateOccupancy(idx) == 1 ||
        edt_->sdf_map_->getOccupancy(idx) != SDFMap::UNKNOWN) {
      clear = false;
      break;
    }
  }

  if (clear) {
    path.push_back(p1);
    path.push_back(p2);
    return (p1 - p2).norm();
  }

  path_finder_->reset();
  path_finder_->setResolution(cg_astar_resolution_);
  if (path_finder_->searchUnknownOnlyBBox(p1, p2, bbox_min, bbox_max) == Astar::REACH_END) {
    path = path_finder_->getPath();
    return Astar::pathLength(path);
  }
  return 1000.0;
}

void UniformGrid::updateConnectivityGraph(
    const std::vector<int>& update_cell_ids, const std::vector<int>& old_center_counts) {
  if (!connectivity_graph_) return;
  if (update_cell_ids.empty()) return;

  ros::Time t1 = ros::Time::now();

  // Remove old nodes for updated cells.
  for (int i = 0; i < static_cast<int>(update_cell_ids.size()); ++i) {
    const int cell_id = update_cell_ids[i];
    const int old_count = old_center_counts[i];
    for (int k = 0; k < old_count; ++k) {
      connectivity_graph_->removeNode(makeNodeId(cell_id, k));
    }
  }

  // Add new nodes for updated cells.
  for (int cell_id : update_cell_ids) {
    auto& grid = grid_data_[cell_id];
    const int total_centers = grid.centers_free_.size() + grid.centers_unknown_.size();
    if (total_centers >= kNodeStride) {
      ROS_ERROR("[ConnectivityGraph] Too many centers in cell %d: %d (stride=%d)", cell_id,
          total_centers, kNodeStride);
    }

    // Add free nodes.
    for (int i = 0; i < static_cast<int>(grid.centers_free_.size()); ++i) {
      ConnectivityNode::Ptr node = std::make_shared<ConnectivityNode>(
          makeNodeId(cell_id, i), grid.centers_free_[i], ConnectivityNode::TYPE::FREE);
      connectivity_graph_->addNode(node);
    }

    for (int i = 0; i < static_cast<int>(grid.centers_unknown_.size()); ++i) {
      // Unknown nodes are placed after free nodes.
      int node_idx = grid.centers_free_.size() + i;
      ConnectivityNode::Ptr node = std::make_shared<ConnectivityNode>(
          makeNodeId(cell_id, node_idx), grid.centers_unknown_[i], ConnectivityNode::TYPE::UNKNOWN);
      connectivity_graph_->addNode(node);
    }
  }

  // Add neighbors relationship to connectivity graph.
  for (int cell_id : update_cell_ids) {
    auto& grid = grid_data_[cell_id];
    std::vector<int> neighbor_ids;
    getNearbyGridIds(cell_id, neighbor_ids);

    // Current cell free centers connect to current cell unknown centers (PORTAL).
    for (int i = 0; i < static_cast<int>(grid.centers_free_.size()); ++i) {
      ConnectivityNode::Ptr node = connectivity_graph_->getNode(makeNodeId(cell_id, i));
      if (!node) continue;

      for (int j = 0; j < static_cast<int>(grid.centers_unknown_.size()); ++j) {
        ConnectivityNode::Ptr node_neighbor =
            connectivity_graph_->getNode(makeNodeId(cell_id, grid.centers_free_.size() + j));
        if (!node_neighbor) continue;

        std::vector<Eigen::Vector3d> path, reverse_path;
        double cost = computeUnknownCostBBox(
            grid.centers_free_[i], grid.centers_unknown_[j], grid.vmin_, grid.vmax_, path);
        reverse_path.assign(path.rbegin(), path.rend());

        node->addNeighborWithPath(makeNodeId(cell_id, grid.centers_free_.size() + j), cost,
            ConnectivityEdge::TYPE::PORTAL, path);
        node_neighbor->addNeighborWithPath(
            makeNodeId(cell_id, i), cost, ConnectivityEdge::TYPE::PORTAL, reverse_path);
      }

      // Current cell free centers connect to nearby cells' free centers (FREE).
      for (int neighbor_id : neighbor_ids) {
        auto& neighbor_grid = grid_data_[neighbor_id];
        for (int k = 0; k < static_cast<int>(neighbor_grid.centers_free_.size()); ++k) {
          ConnectivityNode::Ptr node_neighbor =
              connectivity_graph_->getNode(makeNodeId(neighbor_id, k));
          if (!node_neighbor) continue;

          Eigen::Vector3d bbox_min(std::min(grid.vmin_.x(), neighbor_grid.vmin_.x()),
              std::min(grid.vmin_.y(), neighbor_grid.vmin_.y()),
              std::min(grid.vmin_.z(), neighbor_grid.vmin_.z()));
          Eigen::Vector3d bbox_max(std::max(grid.vmax_.x(), neighbor_grid.vmax_.x()),
              std::max(grid.vmax_.y(), neighbor_grid.vmax_.y()),
              std::max(grid.vmax_.z(), neighbor_grid.vmax_.z()));

          std::vector<Eigen::Vector3d> path, reverse_path;
          double cost = computeFreeCostBBox(
              grid.centers_free_[i], neighbor_grid.centers_free_[k], bbox_min, bbox_max, path);
          reverse_path.assign(path.rbegin(), path.rend());

          node->addNeighborWithPath(
              makeNodeId(neighbor_id, k), cost, ConnectivityEdge::TYPE::FREE, path);
          node_neighbor->addNeighborWithPath(
              makeNodeId(cell_id, i), cost, ConnectivityEdge::TYPE::FREE, reverse_path);
        }
      }
    }

    // Current cell unknown centers connect to nearby cells' unknown centers (UNKNOWN).
    for (int i = 0; i < static_cast<int>(grid.centers_unknown_.size()); ++i) {
      int node_idx = grid.centers_free_.size() + i;
      ConnectivityNode::Ptr node = connectivity_graph_->getNode(makeNodeId(cell_id, node_idx));
      if (!node) continue;

      for (int neighbor_id : neighbor_ids) {
        auto& neighbor_grid = grid_data_[neighbor_id];
        for (int k = 0; k < static_cast<int>(neighbor_grid.centers_unknown_.size()); ++k) {
          int neighbor_node_idx = neighbor_grid.centers_free_.size() + k;
          ConnectivityNode::Ptr node_neighbor =
              connectivity_graph_->getNode(makeNodeId(neighbor_id, neighbor_node_idx));
          if (!node_neighbor) continue;

          Eigen::Vector3d bbox_min(std::min(grid.vmin_.x(), neighbor_grid.vmin_.x()),
              std::min(grid.vmin_.y(), neighbor_grid.vmin_.y()),
              std::min(grid.vmin_.z(), neighbor_grid.vmin_.z()));
          Eigen::Vector3d bbox_max(std::max(grid.vmax_.x(), neighbor_grid.vmax_.x()),
              std::max(grid.vmax_.y(), neighbor_grid.vmax_.y()),
              std::max(grid.vmax_.z(), neighbor_grid.vmax_.z()));

          std::vector<Eigen::Vector3d> path, reverse_path;
          double cost = computeUnknownOnlyCostBBox(grid.centers_unknown_[i],
              neighbor_grid.centers_unknown_[k], bbox_min, bbox_max, path);
          reverse_path.assign(path.rbegin(), path.rend());

          node->addNeighborWithPath(makeNodeId(neighbor_id, neighbor_node_idx), cost,
              ConnectivityEdge::TYPE::UNKNOWN, path);
          node_neighbor->addNeighborWithPath(
              makeNodeId(cell_id, node_idx), cost, ConnectivityEdge::TYPE::UNKNOWN, reverse_path);
        }
      }
    }
  }

  // FALCON planner implementation: remove disconnected unknown centers
  // These are unknown regions that form UNKNOWN-only connected components
  // (i.e., they cannot reach any FREE nodes, meaning they are isolated/unreachable)
  std::set<int> disconnected_nodes;
  connectivity_graph_->findDisconnectedNodes(disconnected_nodes);

  // if (!disconnected_nodes.empty()) {
  //   ROS_INFO("[UniformGrid L%d] Found %lu disconnected nodes", level_, disconnected_nodes.size());
  // }

  std::map<int, std::unordered_set<int>> disconnected_cells_ids_centers_ids;
  for (int node_id : disconnected_nodes) {
    int cell_id = node_id / kNodeStride;
    if (cell_id < 0 || cell_id >= static_cast<int>(grid_data_.size())) continue;
    int unknown_center_id = node_id % kNodeStride - grid_data_[cell_id].centers_free_.size();
    disconnected_cells_ids_centers_ids[cell_id].insert(unknown_center_id);
  }

  // Iterate through disconnected cells and remove disconnected centers from active set
  for (auto it = disconnected_cells_ids_centers_ids.begin();
      it != disconnected_cells_ids_centers_ids.end(); ++it) {
    int cell_id = it->first;
    std::unordered_set<int> centers_ids = it->second;

    if (disconnected_cells_ids_centers_ids.size() == grid_data_.size()) {
      ROS_WARN_THROTTLE(1, "[UniformGrid] All cells are disconnected, no need to update");
      break;
    }

    int before_count = grid_data_[cell_id].centers_unknown_active_.size();

    // Remove disconnected centers from centers_unknown_active_ (remove holes in the map)
    grid_data_[cell_id].centers_unknown_active_.clear();
    grid_data_[cell_id].centers_unknown_active_idx_.clear();
    for (int i = 0; i < static_cast<int>(grid_data_[cell_id].centers_unknown_.size()); i++) {
      if (centers_ids.find(i) == centers_ids.end()) {
        grid_data_[cell_id].centers_unknown_active_.push_back(
            grid_data_[cell_id].centers_unknown_[i]);
        grid_data_[cell_id].centers_unknown_active_idx_.push_back(i);
      }
    }

    int after_count = grid_data_[cell_id].centers_unknown_active_.size();
    // if (before_count != after_count) {
    //   ROS_INFO("[UniformGrid L%d] Cell %d: filtered %d->%d unknown centers (removed %d "
    //            "disconnected)",
    //       level_, cell_id, before_count, after_count, before_count - after_count);
    // }
  }

  connectivity_graph_t = (ros::Time::now() - t1).toSec();
  ROS_INFO("\033[1;38;2;255;182;193m[ConnectivityGraph L%d] construction time: %.3f s\033[0m",
      level_, connectivity_graph_t);
}

void UniformGrid::updateGridInfo(const Eigen::Vector3i& id, const Eigen::Vector3d& pos) {
  int adr = toAddress(id);
  auto& grid = grid_data_[adr];
  if (grid.is_updated_) {  // Ensure only one update to avoid repeated computation
    return;
  }
  grid.is_updated_ = true;

  grid.is_prev_relevant_ = grid.is_cur_relevant_;

  Eigen::Vector3d geometric_center;
  indexToPos(id, 0.5, geometric_center);
  if (use_swarm_tf_) {
    geometric_center = rot_sw_ * geometric_center + trans_sw_;
  }

  Eigen::Vector3d gmin, gmax;
  // indexToPos(id, 0.0, gmin);
  indexToPos(id, 1.0, gmax);  // Only the first 2 values of vmax is useful, should compute max here

  // Check if a voxel is inside the rotated box
  auto inside_box = [](const Eigen::Vector3d& vox, const GridInfo& grid) {
    // Check four separating planes(lines)
    for (int m = 0; m < 4; ++m) {
      if ((vox - grid.vertices_[m]).dot(grid.normals_[m]) <= 0.0) return false;
    }
    return true;
  };

  // Count known
  const double res = edt_->sdf_map_->getResolution();
  grid.center_.setZero();
  grid.unknown_num_ = 0;
  int free = 0;

  std::vector<Vector3d> centers_free, centers_unknown;
  std::vector<int> free_nums, unknown_nums;
  centers_free.clear();
  centers_unknown.clear();
  free_nums.clear();
  unknown_nums.clear();

  getCCLcenters(adr, centers_free, centers_unknown, free_nums, unknown_nums);

  grid.centers_free_ = centers_free;
  grid.centers_unknown_ = centers_unknown;
  grid.centers_unknown_active_ = centers_unknown;
  grid.centers_free_active_.clear();
  grid.centers_free_active_idx_.clear();
  grid.centers_unknown_active_idx_.clear();
  for (int i = 0; i < static_cast<int>(grid.centers_unknown_.size()); ++i) {
    grid.centers_unknown_active_idx_.push_back(i);
  }

  // FALCON-style free active centers:
  // a grid contributes free centers only when it currently has frontiers.
  const bool has_frontier_now = !grid.contained_frontier_ids_.empty();
  if (has_frontier_now) {
    if (!grid.centers_free_.empty()) {
      grid.centers_free_active_ = grid.centers_free_;
      for (int i = 0; i < static_cast<int>(grid.centers_free_.size()); ++i) {
        grid.centers_free_active_idx_.push_back(i);
      }
    } else {
      // Fallback: has frontiers but no free CCL center, keep a virtual active center for state.
      grid.centers_free_active_.push_back(geometric_center);
      grid.centers_free_active_idx_.push_back(-1);
    }
  }

  grid.free_num_ = 0;
  grid.unknown_num_ = 0;
  for (int num : free_nums) grid.free_num_ += num;
  for (int num : unknown_nums) grid.unknown_num_ += num;

  grid.free_num_ = (max_[2] - min_[2]) * grid.free_num_ / edt_->sdf_map_->getResolution();
  grid.unknown_num_ = (max_[2] - min_[2]) * grid.unknown_num_ / edt_->sdf_map_->getResolution();
  free = grid.free_num_;

  // Task-unit state determination:
  // a grid is considered EXPLORED only when both active UNKNOWN and active FREE centers are empty.
  const bool has_unknown_active = !grid.centers_unknown_active_.empty();
  const bool has_free_active = !grid.centers_free_active_.empty();
  const bool frontier_recovery = has_frontier_now && has_free_active;
  const bool task_done = !has_unknown_active && !has_free_active;

  if (task_done) {
    grid.state_ = EXPLORED;
  } else {
    grid.state_ = has_frontier_now ? EXPLORING : EXPLORABLE;
  }

  // FALCON-style: reachability is determined after connectivity-graph update
  // by filtering all-UNKNOWN disconnected components. Skip local A* here.
  if (level_ == 1) {
    grid.center_ = rectifyCenter(geometric_center);
  } else if (centers_unknown.empty()) {
    if (!centers_free.empty())
      grid.center_ = centers_free[0];
    else
      grid.center_ = rectifyCenter(geometric_center);
  } else {
    grid.center_ = centers_unknown[0];
  }

  // Provisional value before connectivity-graph filtering finalizes active centers.
  grid.is_reachable_ = has_unknown_active || frontier_recovery;
  grid.is_cur_relevant_ = isRelevant(grid);

  // Single-layer mode: grid splitting is disabled.
  grid.need_divide_ = false;
  grid.frontier_cell_nums_ = grid.contained_frontier_ids_;
  return;
}

int UniformGrid::toAddress(const Eigen::Vector3i& id) {
  return id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
}

void UniformGrid::adrToIndex(const int& adr, Eigen::Vector3i& idx) {
  // id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
  int tmp_adr = adr;
  const int a = grid_num_(1) * grid_num_(2);
  const int b = grid_num_(2);

  idx[0] = tmp_adr / a;
  tmp_adr = tmp_adr % a;
  idx[1] = tmp_adr / b;
  idx[2] = tmp_adr % b;
}

void UniformGrid::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - min_(i)) / resolution_[i]);
}

void UniformGrid::indexToPos(const Eigen::Vector3i& id, const double& inc, Eigen::Vector3d& pos) {
  // inc: 0 for min, 1 for max, 0.5 for mid point
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + inc) * resolution_[i] + min_(i);
}

bool UniformGrid::insideGrid(const Eigen::Vector3i& id) {
  // Check inside min max
  for (int i = 0; i < 3; ++i) {
    if (id[i] < 0 || id[i] >= grid_num_[i]) {
      return false;
    }
  }
  return true;
}

void UniformGrid::inputFrontiers(const vector<Eigen::Vector3d>& avgs) {
  // 1. Clear frontier ids in all grids.
  for (auto& grid : grid_data_) {
    grid.contained_frontier_ids_.clear();
  }

  int total_frontiers = static_cast<int>(avgs.size());
  int out_of_grid = 0;
  int assigned = 0;

  // 2. Prepare coordinate transform.
  Eigen::Vector3i id;
  Eigen::Matrix3d Rt = rot_sw_.transpose();
  Eigen::Vector3d t_inv = -Rt * trans_sw_;

  // 3. Iterate through each frontier center.
  for (int i = 0; i < avgs.size(); ++i) {
    Eigen::Vector3d pos = avgs[i];

    // 4. Transform coordinates.
    if (use_swarm_tf_) {
      pos = Rt * pos + t_inv;
    }
    // 5. Convert position to grid index.
    posToIndex(pos, id);
    if (!insideGrid(id)) {
      ++out_of_grid;
      continue;
    }
    auto& grid = grid_data_[toAddress(id)];
    grid.contained_frontier_ids_[i] = 1;
    ++assigned;
  }
}

bool UniformGrid::isRelevant(const GridInfo& grid) {
  // Follow FALCON behavior: only grids with active UNKNOWN centers are relevant.
  return grid.is_reachable_ && (grid.state_ == EXPLORABLE || grid.state_ == EXPLORING);
}

Eigen::Vector3d UniformGrid::rectifyCenter(const Eigen::Vector3d& center, double search_radius) {
  // Check if current center is already valid: not OCCUPIED and not in inflate OCCUPIED
  Eigen::Vector3i center_idx;
  edt_->sdf_map_->posToIndex(center, center_idx);

  int center_occ = edt_->sdf_map_->getOccupancy(center_idx);
  int center_inflate_occ = edt_->sdf_map_->getInflateOccupancy(center_idx);

  // Valid if: getOccupancy != OCCUPIED AND getInflateOccupancy != OCCUPIED
  if (center_occ != SDFMap::OCCUPIED && center_inflate_occ != SDFMap::OCCUPIED) {
    // ROS_ERROR("[UniformGrid] Center is already valid");
    return center;  // Already valid
  }

  // Search in expanding circles for valid position
  const double res = edt_->sdf_map_->getResolution();
  Eigen::Vector3d best_center = center;

  // Collect all candidates within search_radius and sort by distance
  std::vector<std::pair<double, Eigen::Vector3d>> candidates;

  for (double dx = -search_radius; dx <= search_radius; dx += res) {
    for (double dy = -search_radius; dy <= search_radius; dy += res) {
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist > search_radius) continue;
      if (dist < 1e-6) continue;  // Skip the center itself

      Eigen::Vector3d candidate = center + Eigen::Vector3d(dx, dy, 0);
      Eigen::Vector3i candidate_idx;
      edt_->sdf_map_->posToIndex(candidate, candidate_idx);

      // Check if in map bounds
      if (!edt_->sdf_map_->isInBox(candidate_idx)) continue;

      // Check if valid: getOccupancy != OCCUPIED AND getInflateOccupancy != OCCUPIED
      int occ = edt_->sdf_map_->getOccupancy(candidate_idx);
      int inflate_occ = edt_->sdf_map_->getInflateOccupancy(candidate_idx);

      if (occ != SDFMap::OCCUPIED && inflate_occ != SDFMap::OCCUPIED) {
        candidates.push_back({ dist, candidate });
      }
    }
  }

  // Sort by distance and pick the closest
  if (!candidates.empty()) {
    std::sort(candidates.begin(), candidates.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });
    best_center = candidates[0].second;
  } else {
    ROS_WARN("[rectifyCenter] Could not find valid position within %.1fm of (%.2f, %.2f, %.2f), "
             "occ=%d, inflate_occ=%d",
        search_radius, center.x(), center.y(), center.z(), center_occ, center_inflate_occ);
  }

  return best_center;
}

void UniformGrid::getUpdatedBoxMarker(
    vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2) {
  pts1.clear();
  pts2.clear();

  double z = 1.0;  // Visualization height

  // Helper to add box edges
  auto addBoxEdges = [&](const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax) {
    // Four corners at z height
    Eigen::Vector3d p0(bmin.x(), bmin.y(), z);
    Eigen::Vector3d p1(bmax.x(), bmin.y(), z);
    Eigen::Vector3d p2(bmax.x(), bmax.y(), z);
    Eigen::Vector3d p3(bmin.x(), bmax.y(), z);

    // Four edges of the rectangle
    pts1.push_back(p0);
    pts2.push_back(p1);
    pts1.push_back(p1);
    pts2.push_back(p2);
    pts1.push_back(p2);
    pts2.push_back(p3);
    pts1.push_back(p3);
    pts2.push_back(p0);
  };

  // FOV updated box (main)
  if ((updated_box_max_ - updated_box_min_).norm() > 1e-3) {
    addBoxEdges(updated_box_min_, updated_box_max_);
  }

  // Chunk boxes (from multi-robot)
  for (size_t i = 0; i < chunk_box_mins_.size(); ++i) {
    addBoxEdges(chunk_box_mins_[i], chunk_box_maxs_[i]);
  }
}

}  // namespace c2_expl
