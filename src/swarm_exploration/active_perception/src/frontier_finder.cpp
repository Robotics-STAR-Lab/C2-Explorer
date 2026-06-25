#include <active_perception/frontier_finder.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/edt_environment.h>
#include <plan_env/multi_map_manager.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>

#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>

namespace c2_expl {
FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr& edt, ros::NodeHandle& nh) {
  this->edt_env_ = edt;
  int voxel_num = edt->sdf_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);

  nh.param("perception_utils/lidar_top_angle", lidar_fov_up_, -1.0);
  nh.param("perception_utils/lidar_bottom_angle", lidar_fov_down_, -1.0);
  nh.param("perception_utils/lidar_max_dist", lidar_max_dist_, -1.0);
  nh.param("perception_utils/lidar_pitch", lidar_pitch_, 0.0);

  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);

  percep_utils_.reset(new PerceptionUtils(nh));
}

FrontierFinder::~FrontierFinder() {
}

void FrontierFinder::searchFrontiers() {
  tmp_frontiers_.clear();

  // Bounding box of updated region
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, false);

  // Bounding box of external chunks
  vector<Eigen::Vector3d> chunk_mins, chunk_maxs;
  edt_env_->sdf_map_->mm_->getChunkBoxes(chunk_mins, chunk_maxs, false);

  vector<Eigen::Vector3d> mins, maxs;
  mins.push_back(update_min);
  mins.insert(mins.end(), chunk_mins.begin(), chunk_mins.end());
  maxs.push_back(update_max);
  maxs.insert(maxs.end(), chunk_maxs.begin(), chunk_maxs.end());

  // Removed changed frontiers in updated map
  auto resetFlag = [&](vector<Frontier>::iterator& iter, vector<Frontier>& frontiers) {
    Eigen::Vector3i idx;
    for (const auto& cell : iter->cells_) {
      edt_env_->sdf_map_->posToIndex(cell, idx);
      int adr = toadr(idx);
      if (adr >= 0 && adr < frontier_flag_.size()) {
        frontier_flag_[adr] = 0;
      }
    }
    iter = frontiers.erase(iter);
  };

  // ROS_INFO("[Frontier] Before remove: %lu", frontiers_.size());

  removed_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    // haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max)
    if (haveAnyOverlap(iter->box_min_, iter->box_max_, mins, maxs) && isFrontierChanged(*iter)) {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx;
      ++iter;
    }
  }
  // ROS_INFO("[Frontier] After remove: %lu", frontiers_.size());
  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveAnyOverlap(iter->box_min_, iter->box_max_, mins, maxs) && isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  // Search new frontier within box slightly inflated from updated box
  Vector3d box_min, box_max;
  edt_env_->sdf_map_->getBox(box_min, box_max);

  vector<Eigen::Vector3d> search_mins, search_maxs;
  for (int i = 0; i < static_cast<int>(mins.size()); ++i) {
    search_mins.push_back(mins[i] - Vector3d(1, 1, 0.2));
    search_maxs.push_back(maxs[i] + Vector3d(1, 1, 0.2));
    for (int k = 0; k < 3; ++k) {
      search_mins[i][k] = max(search_mins[i][k], box_min[k]);
      search_maxs[i][k] = min(search_maxs[i][k], box_max[k]);
    }
  }
  vector<Eigen::Vector3i> min_ids(mins.size()), max_ids(mins.size());
  for (int i = 0; i < static_cast<int>(mins.size()); ++i) {
    edt_env_->sdf_map_->posToIndex(search_mins[i], min_ids[i]);
    edt_env_->sdf_map_->posToIndex(search_maxs[i], max_ids[i]);
  }

  for (int i = 0; i < static_cast<int>(min_ids.size()); ++i) {
    auto min_id = min_ids[i];
    auto max_id = max_ids[i];

    for (int z = min_id(2); z <= max_id(2); ++z)
      for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y) {
          // Scanning the updated region to find seeds of frontiers
          Eigen::Vector3i cur(x, y, z);
          int cur_adr = toadr(cur);
          if (cur_adr >= 0 && cur_adr < frontier_flag_.size() && frontier_flag_[cur_adr] == 0 &&
              knownfree(cur) && isNeighborUnknown(cur)) {
            // Expand from the seed cell to find a complete frontier cluster
            expandFrontier(cur);
          }
        }
  }

  splitLargeFrontiers(tmp_frontiers_);
}

void FrontierFinder::expandFrontier(
    const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */) {

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  int first_adr = toadr(first);
  if (first_adr >= 0 && first_adr < frontier_flag_.size()) {
    frontier_flag_[first_adr] = 1;
  }

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and frontier cell not clustered
      int adr = toadr(nbr);
      // Add boundary check to prevent segmentation fault
      if (adr < 0 || adr >= frontier_flag_.size()) continue;
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) ||
          !(knownfree(nbr) && isNeighborUnknown(nbr)))
        continue;

      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 1.0 || pos[2] > 1.5) continue;  // Remove noise close to ground
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }
  if (expanded.size() > cluster_min_) {
    // Compute detailed info
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  }
}

void FrontierFinder::splitLargeFrontiers(vector<Frontier>& frontiers) {
  vector<Frontier> splits, tmps;
  for (const auto& frontier : frontiers) {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(frontier, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(frontier);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitHorizontally(const Frontier& frontier, vector<Frontier>& splits) {
  // Split a frontier into small piece if it is too large
  auto mean = frontier.average_.head<2>();
  bool need_split = false;
  for (const auto& cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split) return false;

  // Compute principal component
  // Covariance matrix of cells
  Eigen::Matrix2d cov;
  cov.setZero();
  for (const auto& cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);
  // std::cout << "max idx: " << max_idx << std::endl;
  // std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() <<
  // std::endl;

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (const auto& cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  vector<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

void FrontierFinder::updateFrontierCostMatrix() {
  if (!removed_ids_.empty()) {
    // Delete path and cost for removed clusters
    auto it_first_new = frontiers_.begin() + first_new_ftr_;
    for (auto it = frontiers_.begin(); it != it_first_new; ++it) {
      auto cost_iter = it->costs_.begin();
      auto path_iter = it->paths_.begin();
      int iter_idx = 0;
      for (int i = 0; i < static_cast<int>(removed_ids_.size()); ++i) {
        // Step iterator to the item to be removed
        while (iter_idx < removed_ids_[i]) {
          ++cost_iter;
          ++path_iter;
          ++iter_idx;
        }
        cost_iter = it->costs_.erase(cost_iter);
        path_iter = it->paths_.erase(path_iter);
      }
    }
    removed_ids_.clear();
  }

  auto updateCost = [](const vector<Frontier>::iterator& it1,
                        const vector<Frontier>::iterator& it2) {
    // Search path from old cluster's top viewpoint to new cluster'
    Viewpoint& vui = it1->viewpoints_.front();
    Viewpoint& vuj = it2->viewpoints_.front();
    vector<Vector3d> path_ij;
    double cost_ij = ViewNode::computeCost(
        vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
    // Insert item for both old and new clusters
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  // Compute path and cost between old and new clusters
  auto it_first_new = frontiers_.begin() + first_new_ftr_;
  for (auto it1 = frontiers_.begin(); it1 != it_first_new; ++it1)
    for (auto it2 = it_first_new; it2 != frontiers_.end(); ++it2) updateCost(it1, it2);

  // Compute path and cost between new clusters
  for (auto it1 = it_first_new; it1 != frontiers_.end(); ++it1)
    for (auto it2 = it1; it2 != frontiers_.end(); ++it2) {
      if (it1 == it2) {
        it1->costs_.push_back(0);
        it1->paths_.push_back({});
      } else
        updateCost(it1, it2);
    }
  first_new_ftr_ = frontiers_.size();
}

bool FrontierFinder::haveOverlap(
    const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
  // Check if two box have overlap part
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3) return false;
  }
  return true;
}

bool FrontierFinder::haveAnyOverlap(const Vector3d& min1, const Vector3d& max1,
    const vector<Vector3d>& mins, const vector<Vector3d>& maxs) {

  for (int i = 0; i < static_cast<int>(mins.size()); ++i) {
    if (haveOverlap(min1, max1, mins[i], maxs[i])) return true;
  }
  return false;
}

bool FrontierFinder::isFrontierChanged(const Frontier& ft) {
  for (const auto& cell : ft.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    if (!(knownfree(idx) && isNeighborUnknown(idx))) return true;
  }
  return false;
}

void FrontierFinder::computeFrontierInfo(Frontier& ftr) {
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  for (const auto& cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());

  // Compute downsampled cluster
  downsample(ftr.cells_, ftr.filtered_cells_);
}

void FrontierFinder::computeFrontiersToVisit() {
  first_new_ftr_ = frontiers_.size();
  int new_num = 0;
  int new_dormant_num = 0;
  // Try find viewpoints for each cluster and categorize them according to viewpoint number
  for (auto& tmp_ftr : tmp_frontiers_) {
    // Search viewpoints around frontier
    sampleViewpoints(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty()) {
      ++new_num;
      frontiers_.push_back(tmp_ftr);
      auto& inserted = frontiers_.back();
      // Sort the viewpoints by coverage fraction, best view in front
      sort(inserted.viewpoints_.begin(), inserted.viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
    } else {
      // Find no viewpoint, move cluster to dormant list
      dormant_frontiers_.push_back(tmp_ftr);
      ++new_dormant_num;
    }
  }
  // Reset indices of frontiers
  int idx = 0;
  for (auto& ft : frontiers_) {
    ft.id_ = idx++;
  }
  // ROS_INFO(
  //     "[Frontier] Frontier num: %lu, dormant: %lu", frontiers_.size(),
  //     dormant_frontiers_.size());
}

void FrontierFinder::getTopViewpointsInfo(const Vector3d& cur_pos, vector<Eigen::Vector3d>& points,
    vector<double>& yaws, vector<Eigen::Vector3d>& averages, vector<Viewpoint>& viewpoints) {
  points.clear();
  yaws.clear();
  averages.clear();
  viewpoints.clear();

  auto appendViewpoint = [&](const Frontier& frontier, const Viewpoint& view) {
    points.push_back(view.pos_);
    yaws.push_back(view.yaw_);
    averages.push_back(frontier.average_);
    viewpoints.push_back(view);
  };

  for (const auto& frontier : frontiers_) {
    bool no_view = true;
    for (const auto& view : frontier.viewpoints_) {
      // Retrieve the first viewpoint that is far enough and has highest coverage
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      appendViewpoint(frontier, view);
      no_view = false;
      break;
    }
    if (no_view) {
      // All viewpoints are very close, just use the first one (with highest coverage).
      appendViewpoint(frontier, frontier.viewpoints_.front());
    }
  }
}

void FrontierFinder::getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids,
    const int& view_num, const double& max_decay, vector<vector<Eigen::Vector3d>>& points,
    vector<vector<double>>& yaws) {
  points.clear();
  yaws.clear();
  for (auto id : ids) {
    if (id < 0 || id >= static_cast<int>(frontiers_.size())) continue;

    const auto& frontier = frontiers_[id];
    if (frontier.viewpoints_.empty()) continue;

    // Get several top viewpoints that are far enough
    vector<Eigen::Vector3d> pts;
    vector<double> ys;
    int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
    for (const auto& view : frontier.viewpoints_) {
      if (static_cast<int>(pts.size()) >= view_num || view.visib_num_ <= visib_thresh) break;
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      pts.push_back(view.pos_);
      ys.push_back(view.yaw_);
    }
    if (pts.empty()) {
      // All viewpoints are very close, ignore the distance limit
      for (const auto& view : frontier.viewpoints_) {
        if (static_cast<int>(pts.size()) >= view_num || view.visib_num_ <= visib_thresh) break;
        pts.push_back(view.pos_);
        ys.push_back(view.yaw_);
      }
    }
    points.push_back(pts);
    yaws.push_back(ys);
  }
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters) {
  clusters.clear();
  for (const auto& frontier : frontiers_) clusters.push_back(frontier.cells_);
}

void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters) {
  clusters.clear();
  for (const auto& ft : dormant_frontiers_) clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes) {
  boxes.clear();
  for (const auto& frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}

void FrontierFinder::getPathForTour(
    const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path) {
  if (frontier_ids.empty()) return;
  auto valid_frontier_id = [&](int id) {
    return id >= 0 && id < static_cast<int>(frontiers_.size());
  };

  // Compute the path from current pos to the first frontier
  const int first_id = frontier_ids.front();
  if (!valid_frontier_id(first_id) || frontiers_[first_id].viewpoints_.empty()) {
    ROS_WARN("[Frontier] Invalid first frontier id %d for tour path extraction", first_id);
    return;
  }
  vector<Vector3d> segment;
  ViewNode::searchPath(pos, frontiers_[first_id].viewpoints_.front().pos_, segment);
  path.insert(path.end(), segment.begin(), segment.end());

  // Get paths of tour passing all clusters
  for (int i = 0; i < static_cast<int>(frontier_ids.size()) - 1; ++i) {
    const int from_id = frontier_ids[i];
    const int to_id = frontier_ids[i + 1];
    if (!valid_frontier_id(from_id) || !valid_frontier_id(to_id)) {
      ROS_WARN("[Frontier] Skip invalid edge (%d -> %d) in frontier tour", from_id, to_id);
      continue;
    }
    if (frontiers_[from_id].viewpoints_.empty() || frontiers_[to_id].viewpoints_.empty()) {
      ROS_WARN("[Frontier] Skip frontier edge (%d -> %d) due to empty viewpoint list", from_id,
          to_id);
      continue;
    }

    const auto& cached_paths = frontiers_[from_id].paths_;
    if (to_id < 0 || to_id >= static_cast<int>(cached_paths.size())) {
      // Fallback to online search when cached path matrix is inconsistent with frontier ids.
      vector<Vector3d> fallback_segment;
      ViewNode::searchPath(frontiers_[from_id].viewpoints_.front().pos_,
          frontiers_[to_id].viewpoints_.front().pos_, fallback_segment);
      path.insert(path.end(), fallback_segment.begin(), fallback_segment.end());
      continue;
    }
    auto path_iter = cached_paths.begin();
    for (int step = 0; step < to_id && path_iter != cached_paths.end(); ++step) {
      ++path_iter;
    }
    if (path_iter == cached_paths.end()) {
      vector<Vector3d> fallback_segment;
      ViewNode::searchPath(frontiers_[from_id].viewpoints_.front().pos_,
          frontiers_[to_id].viewpoints_.front().pos_, fallback_segment);
      path.insert(path.end(), fallback_segment.begin(), fallback_segment.end());
      continue;
    }
    const auto& cached_segment = *path_iter;
    path.insert(path.end(), cached_segment.begin(), cached_segment.end());
  }
}

void FrontierFinder::getSwarmCostMatrix(const vector<Vector3d>& positions,
    const vector<Vector3d>& velocities, const vector<double>& yaws, Eigen::MatrixXd& mat) {

  const int drone_num = positions.size();
  const int ftr_num = frontiers_.size();
  const int dimen = 1 + drone_num + ftr_num;
  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // Virtual depot to drones
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = 0;     // depot to drone: allow starting from drone origins.
    mat(1 + i, 0) = 1000;  // drone to depot: positive cost strongly discourages returning to drone origins.
  }
  // Virtual depot to frontiers
  for (int i = 0; i < ftr_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;  // depot to frontier: discourage starting directly from a frontier.
    mat(1 + drone_num + i, 0) = 0;     // frontier to depot: allow finishing at the depot with zero cost.
  }
  // Costs between drones
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;  // Drone-to-drone transitions get a huge cost, effectively disabled.
    }
  }

  // Costs from drones to frontiers
  for (int i = 0; i < drone_num; ++i) {
    int j = 0;
    for (const auto& ftr : frontiers_) {
      const Viewpoint& vj = ftr.viewpoints_.front();
      vector<Vector3d> path;
      mat(1 + i, 1 + drone_num + j) =
          ViewNode::computeCost(positions[i], vj.pos_, yaws[0], vj.yaw_, velocities[i], 0.0, path);
      mat(1 + drone_num + j, 1 + i) = 0;
      ++j;
    }
  }
  // Costs between frontiers
  int i = 0, j = 0;
  for (const auto& ftr : frontiers_) {
    for (const auto& cs : ftr.costs_) {
      mat(1 + drone_num + i, 1 + drone_num + j) = cs;
      ++j;
    }
    ++i;
    j = 0;
  }
  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 1000;
  }

}

// Cost matrix for the frontier subset inside a grid.
void FrontierFinder::getSwarmCostMatrix(const vector<Vector3d>& positions,
    const vector<Vector3d>& velocities, const vector<double>& yaws, const vector<int>& ftr_ids,
    const vector<Eigen::Vector3d>& grid_pos, Eigen::MatrixXd& mat) {

  Eigen::MatrixXd full_mat;
  // Build the full cost matrix for lookup.
  getSwarmCostMatrix(positions, velocities, yaws, full_mat);

  // Get part of the full matrix according to selected frontier

  const int drone_num = positions.size();
  const int ftr_num = ftr_ids.size();
  int dimen = 1 + drone_num + ftr_num;
  if (!grid_pos.empty()) dimen += 1;

  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // Virtual depot to drones
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = 0;
    mat(1 + i, 0) = 1000;
  }
  // Virtual depot to frontiers
  for (int i = 0; i < ftr_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;
    mat(1 + drone_num + i, 0) = 0;
  }
  // Costs between drones
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;
    }
  }

  // Costs from drones to frontiers
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < ftr_num; ++j) {
      mat(1 + i, 1 + drone_num + j) = full_mat(1 + i, 1 + drone_num + ftr_ids[j]);
      mat(1 + drone_num + j, 1 + i) = 0;
    }
  }
  // Costs between frontiers
  for (int i = 0; i < ftr_num; ++i) {
    for (int j = 0; j < ftr_num; ++j) {
      mat(1 + drone_num + i, 1 + drone_num + j) =
          full_mat(1 + drone_num + ftr_ids[i], 1 + drone_num + ftr_ids[j]);
    }
  }
  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 1000;
  }

  // Consider next grid in global tour
  if (!grid_pos.empty()) {
    // Depot
    mat(0, 1 + drone_num + ftr_num) = 1000;
    mat(1 + drone_num + ftr_num, 0) = 0;

    // Drone
    for (int i = 0; i < drone_num; ++i) {
      mat(1 + i, 1 + drone_num + ftr_num) = 1000;
      mat(1 + drone_num + ftr_num, 1 + i) = 1000;
    }

    // Frontier
    vector<Eigen::Vector3d> points, tmps;
    vector<double> yaws;
    vector<Viewpoint> vps;
    getTopViewpointsInfo(positions[0], points, yaws, tmps, vps);
    Eigen::Vector3d next_grid = grid_pos[0];

    for (int i = 0; i < ftr_num; ++i) {
      double cost = ViewNode::computeCost(
          next_grid, points[ftr_ids[i]], 0, 0, Eigen::Vector3d(0, 0, 0), 0, tmps);
      mat(1 + drone_num + i, 1 + drone_num + ftr_num) = cost;
      mat(1 + drone_num + ftr_num, 1 + drone_num + i) = cost;
    }
  }
}

// Sample viewpoints around frontier's average position, check coverage to the frontier cells
void FrontierFinder::sampleViewpoints(Frontier& frontier) {

  // Evaluate sample viewpoints on circles, find ones that cover most cells
  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
      rc <= candidate_rmax_ + 1e-3; rc += dr)
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);

      // Qualified viewpoint is in bounding box and in safe region
      if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
          edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
        continue;

      // Compute average yaw
      auto& cells = frontier.filtered_cells_;
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      double avg_yaw = 0.0;
      for (int i = 1; i < static_cast<int>(cells.size()); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        double yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
        avg_yaw += yaw;
      }
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);
      // Compute the fraction of covered and visible cells
      int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
      if (visib_num > min_visib_num_) {
        Viewpoint vp = { sample_pos, avg_yaw, visib_num };
        frontier.viewpoints_.push_back(vp);
      }
    }
}

bool FrontierFinder::isFrontierCovered() {
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max);

  // Bounding box of external chunks
  vector<Eigen::Vector3d> chunk_mins, chunk_maxs;
  edt_env_->sdf_map_->mm_->getChunkBoxes(chunk_mins, chunk_maxs, false);

  vector<Eigen::Vector3d> mins, maxs;
  mins.push_back(update_min);
  mins.insert(mins.end(), chunk_mins.begin(), chunk_mins.end());
  maxs.push_back(update_max);
  maxs.insert(maxs.end(), chunk_maxs.begin(), chunk_maxs.end());

  auto checkChanges = [&](const vector<Frontier>& frontiers) {
    for (const auto& ftr : frontiers) {
      // haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)
      if (!haveAnyOverlap(ftr.box_min_, ftr.box_max_, mins, maxs)) continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (const auto& cell : ftr.cells_) {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_)) return true;

  return false;
}

bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos) {
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN) return true;
      }
  return false;
}

int FrontierFinder::countVisibleCells(
    const Eigen::Vector3d& pos, const double& yaw, const vector<Eigen::Vector3d>& cluster) {
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  for (const auto& cell : cluster) {
    // Check if frontier cell is inside FOV
    // if (!percep_utils_->insideFOV(cell)) continue;
    if (!isInLidarFOV(pos, yaw, cell)) continue;

    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 ||
          edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
        visib = false;
        break;
      }
    }
    if (visib) visib_num += 1;
  }
  return visib_num;
}

void FrontierFinder::downsample(
    const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out) {
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& cell : cluster_in) cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const double leaf_size = edt_env_->sdf_map_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (const auto& pt : cloudf->points) cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double& yaw) {
  while (yaw < -M_PI) yaw += 2 * M_PI;
  while (yaw > M_PI) yaw -= 2 * M_PI;
}

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  // At least one neighbor is unknown
  auto nbrs = tenNeighbors(voxel);
  for (const auto& nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN &&
        edt_env_->sdf_map_->isInMap(nbr))
      return true;
  }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->toAddress(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
}

bool FrontierFinder::isInLidarFOV(
    const Eigen::Vector3d& vp_pos, const double& vp_yaw, const Vector3d& frt_cell) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  // transform.rotate(Eigen::AngleAxisd(-lidar_roll_, Eigen::Vector3d::UnitX()));
  transform.rotate(Eigen::AngleAxisd(-lidar_pitch_, Eigen::Vector3d::UnitY()));
  transform.rotate(Eigen::AngleAxisd(-vp_yaw, Eigen::Vector3d::UnitZ()));

  if ((vp_pos - frt_cell).norm() > lidar_max_dist_) return false;
  Eigen::Vector3d pt2see = transform * (frt_cell - vp_pos);
  // pitch
  float pitch = atan2(pt2see.z(), sqrt(pt2see.x() * pt2see.x() + pt2see.y() * pt2see.y()));
  if (pitch > lidar_fov_up_ || pitch < -lidar_fov_down_) return false;

  return true;
}

}  // namespace c2_expl
