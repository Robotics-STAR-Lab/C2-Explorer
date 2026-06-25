#ifndef _UNIFORM_GRID_H_
#define _UNIFORM_GRID_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <active_perception/connectivity_graph.h>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::unordered_set;
using std::vector;

class RayCaster;

namespace c2_expl {

class EDTEnvironment;
class Astar;
class HGrid;

enum GridState { EXPLORED, EXPLORING, EXPLORABLE, UNREACHABLE };

// struct GridInfo {};

class GridInfo {
public:
  GridInfo() {
  }
  ~GridInfo() {
  }

  int unknown_num_;
  int free_num_;
  int frontier_num_;
  Eigen::Vector3d center_;
  vector<Eigen::Vector3d> centers_free_;
  vector<Eigen::Vector3d> centers_unknown_;
  vector<Eigen::Vector3d> centers_free_active_;
  vector<Eigen::Vector3d> centers_unknown_active_;
  vector<int> centers_free_active_idx_;
  vector<int> centers_unknown_active_idx_;
  unordered_map<int, int> frontier_cell_nums_;
  unordered_map<int, int> contained_frontier_ids_;
  bool is_updated_;
  bool need_divide_, active_;

  bool is_prev_relevant_;
  bool is_cur_relevant_;
  bool is_reachable_;
  bool was_in_tour_;  // Track if this grid has ever been added to tour

  GridState state_;

  // Vertices and their box in xy plane, in current drone's frame
  Eigen::Vector3d vmin_, vmax_;
  vector<Eigen::Vector3d> vertices_;

  // Normals of separating lines in xy plane, associated with vertices_
  vector<Eigen::Vector3d> normals_;
};

class UniformGrid {

public:
  UniformGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh, const int& level);
  ~UniformGrid();

  void initGridData();
  void updateBaseCoor();
  void getCCLcenters(const int& grid_id, std::vector<Vector3d>& centers_free,
      std::vector<Vector3d>& centers_unknown, std::vector<int>& free_nums,
      std::vector<int>& unknown_nums);
  void updateGridData(const int& drone_id, vector<int>& grid_ids, vector<int>& parti_ids,
      vector<int>& parti_ids_all, const Eigen::Vector3d& pos);

  void inputFrontiers(const vector<Eigen::Vector3d>& avgs);
  void getUpdatedBoxMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2);
  Eigen::Vector3d rectifyCenter(const Eigen::Vector3d& center, double search_radius = 2.0);
  ConnectivityGraph::Ptr getConnectivityGraph() {
    return connectivity_graph_;
  }
  int getConnectivityNodeId(const int& cell_id, const int& center_id) const {
    return cell_id * kNodeStride + center_id;
  }

private:
  static bool sameFrontierIdSet(
      const std::unordered_map<int, int>& a, const std::unordered_map<int, int>& b);
  static bool boxesOverlap2D(const Eigen::Vector3d& min1, const Eigen::Vector3d& max1,
      const Eigen::Vector3d& min2, const Eigen::Vector3d& max2);
  static const char* gridStateName(const GridState& state);
  bool shouldUpdateGrid(const GridInfo& grid, const Eigen::Vector3d& update_min,
      const Eigen::Vector3d& update_max, const vector<Eigen::Vector3d>& chunk_mins,
      const vector<Eigen::Vector3d>& chunk_maxs, bool& overlap_with_fov) const;
  void finalizeUpdatedGridState(const int& cell_id);
  void rebuildRelevantGridList();
  void updateAssignedGridIds(const int& drone_id, vector<int>& grid_ids, vector<int>& parti_ids);
  void updateGridInfo(const Eigen::Vector3i& id, const Eigen::Vector3d& pos);
  std::vector<std::unordered_set<int>> splitUnknownComponentByConvexity(
      const std::unordered_set<int>& voxel_addrs, const int step) const;
  Eigen::Vector3d projectToNearestUnknownVoxel(
      const std::unordered_set<int>& voxel_addrs, const Eigen::Vector3d& ref_center) const;
  void updateConnectivityGraph(
      const std::vector<int>& update_cell_ids, const std::vector<int>& old_center_counts);
  void getNearbyGridIds(const int& adr, std::vector<int>& neighbor_ids);
  int makeNodeId(const int& cell_id, const int& center_id) const {
    return cell_id * kNodeStride + center_id;
  }
  bool pathInsideBBox(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& bbox_min,
      const Eigen::Vector3d& bbox_max) const;
  double computeFreeCostBBox(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
      const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
      std::vector<Eigen::Vector3d>& path);
  double computeUnknownCostBBox(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
      const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
      std::vector<Eigen::Vector3d>& path);
  double computeUnknownOnlyCostBBox(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
      const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
      std::vector<Eigen::Vector3d>& path);

  int toAddress(const Eigen::Vector3i& id);
  void adrToIndex(const int& adr, Eigen::Vector3i& idx);
  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, const double& inc, Eigen::Vector3d& pos);
  bool insideGrid(const Eigen::Vector3i& id);
  bool isRelevant(const GridInfo& grid);

  shared_ptr<EDTEnvironment> edt_;
  unique_ptr<RayCaster> caster_;
  unique_ptr<Astar> path_finder_;
  vector<GridInfo> grid_data_;

  vector<int> relevant_id_;
  unordered_map<int, int> relevant_map_;
  bool initialized_;

  Eigen::Vector3d resolution_;
  Eigen::Vector3d min_, max_;
  Eigen::Vector3i grid_num_;
  int level_;
  int ccl_step_;

  int z_samples_;              // Z samples for free/unknown estimation
  double w_unknown_;
  double grid_size_;
  double cg_astar_resolution_;
  double convex_split_hull_area_ratio_;

  // Swarm tf
  Eigen::Matrix3d rot_sw_;
  Eigen::Vector3d trans_sw_;
  bool use_swarm_tf_;

  static constexpr int kNodeStride = 10000;

  ConnectivityGraph::Ptr connectivity_graph_;
  double connectivity_graph_t = 0.0;

  vector<vector<unordered_set<int>>> ccl_voxels_addr_;
  vector<vector<Vector3d>> ccl_voxels_color_;
  vector<vector<pair<int, int>>> ccl_free_unknown_states_and_centers_idx_;

  // Updated box for visualization
  Eigen::Vector3d updated_box_min_, updated_box_max_;
  vector<Eigen::Vector3d> chunk_box_mins_, chunk_box_maxs_;

  double resolution;

  friend HGrid;
};

}  // namespace c2_expl
#endif
