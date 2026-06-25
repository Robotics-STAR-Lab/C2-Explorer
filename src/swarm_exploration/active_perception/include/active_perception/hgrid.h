#ifndef _HGRID_H_
#define _HGRID_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>
#include <active_perception/uniform_grid.h>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

class RayCaster;

namespace c2_expl {

class EDTEnvironment;
class Astar;
class GridInfo;
class UniformGrid;
class ConnectivityGraph;

// struct GridInfo {};

// Single-layer grid manager (Falcon-style)
class HGrid {

public:
  enum CenterType {
    UNKNOWN_ACTIVE_CENTER = 1,
    FREE_ACTIVE_CENTER = 2,
  };

  HGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~HGrid();

  struct HullTask {
    int grid_id = -1;
    vector<Eigen::Vector3d> hull;
    bool empty() const {
      return grid_id < 0 || hull.empty();
    }
  };

  void updateGridData(const int& drone_id, vector<int>& grid_ids, bool reallocated,
      const vector<int>& last_grid_ids, vector<int>& first_ids, vector<int>& second_ids,
      const Eigen::Vector3d& pos);

  bool updateBaseCoor();
  void inputFrontiers(const vector<Eigen::Vector3d>& avgs);
  void getAllocationCostMatrix(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, const vector<int>& grid_ids,
      Eigen::MatrixXd& mat, const vector<HullTask>* hull_tasks = nullptr);
  void getTourCostMatrix(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, const vector<int>& grid_ids,
      Eigen::MatrixXd& mat, const vector<HullTask>* hull_tasks = nullptr);
  bool findCenterIndexByPos(const Eigen::Vector3d& pos, int& center_idx, double tol = 0.05) const;
  bool getGridIdByCenterPos(const Eigen::Vector3d& pos, int& grid_id, double tol = 0.05) const;
  bool getCenterHullByPos(
      const Eigen::Vector3d& pos, vector<Eigen::Vector3d>& hull, double tol = 0.05) const;
  const vector<int>& getLastCostMatrixCenterGridIds() const {
    return last_center_grid_ids_;
  }
  const vector<Eigen::Vector3d>& getLastCostMatrixCenterPositions() const {
    return last_center_positions_;
  }
  const vector<vector<Eigen::Vector3d>>& getLastCostMatrixCenterHulls() const {
    return last_center_hulls_;
  }
  const vector<int>& getLastCostMatrixCenterIds() const {
    return last_center_ids_;
  }
  const vector<int>& getLastCostMatrixCenterTypes() const {
    return last_center_types_;
  }
  void getFrontiersInGrid(const vector<int>& grid_ids, vector<int>& ftr_ids);
  bool getNextGrid(const vector<int>& grid_ids, Eigen::Vector3d& grid_pos, double& grid_yaw);
  void getConsistentGrid(const vector<int>& last_ids, const vector<int>& cur_ids,
      vector<int>& first_ids, vector<int>& second_ids);

  void getGridMarker(
      const vector<int>& grid_ids, vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2);
  void getConnectivityGraphVisualizationData(const int& level,
      vector<Eigen::Vector3d>& unknown_edge_start, vector<Eigen::Vector3d>& unknown_edge_end,
      vector<Eigen::Vector3d>& free_edge_start, vector<Eigen::Vector3d>& free_edge_end,
      vector<Eigen::Vector3d>& portal_edge_start, vector<Eigen::Vector3d>& portal_edge_end,
      vector<Eigen::Vector3d>& active_unknown_nodes, vector<Eigen::Vector3d>& active_free_nodes,
      vector<Eigen::Vector3d>& inactive_unknown_nodes,
      vector<Eigen::Vector3d>& inactive_free_nodes,
      vector<Eigen::Vector3d>& edge_cost_pos, vector<double>& edge_costs);
  void getExploredGrids(vector<int>& grid_ids);
  int getUnknownCellsNum(const int& grid_id);
  Eigen::Vector3d getCenter(const int& grid_id);
  unique_ptr<Astar> path_finder_;

private:
  static constexpr int kConvexSplitMinClusterPts = 20;
  static constexpr double kConvexSplitMinClusterRatio = 0.10;
  static constexpr int kConvexSplitMaxParts = 4;
  static constexpr int kConvexSplitMaxDepth = 3;
  static constexpr double kConvexSplitHullAreaRatio = 1.55;

  struct Pt2 {
    double x = 0.0;
    double y = 0.0;
  };

  struct SplitHull {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    vector<Eigen::Vector3d> hull;
  };

  struct CenterMeta {
    UniformGrid* ug = nullptr;
    int node_id = -1;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  };

  int toCoarseId(const int& id) const;
  double getCoarseUnknownRatio(const int& id);

  bool isClose(const int& id1, const int& id2);
  bool inSameLevel1(const int& id1, const int& id2);
  int getCoarseChebyshevDist(const int& id1, const int& id2);

  GridInfo& getGrid(const int& id);
  vector<Eigen::Vector3d> buildConvexHull(vector<Pt2> pts, double z) const;
  double getHullArea2D(const vector<Eigen::Vector3d>& hull) const;
  bool pointInsideHull2D(
      const vector<Eigen::Vector3d>& hull, const Eigen::Vector3d& p, double tol = 0.25) const;
  bool hullContainsFrontier(const vector<Eigen::Vector3d>& hull) const;
  Eigen::Vector3d getHullCentroid(const vector<Eigen::Vector3d>& hull) const;
  bool splitByKMeans2(const vector<Pt2>& pts, vector<Pt2>& g0, vector<Pt2>& g1) const;
  vector<SplitHull> collectUnknownCenterHulls(
      UniformGrid* ug, int local_id, int unknown_idx, const Eigen::Vector3d& center) const;
  void appendCenter(vector<CenterMeta>& centers, UniformGrid* ug, int node_id,
      const Eigen::Vector3d& pos, int grid_id, int center_id, int center_type,
      const vector<Eigen::Vector3d>& hull);
  bool appendHullFreeCenter(vector<CenterMeta>& centers, UniformGrid* ug, const int local_id,
      const int grid_id, const vector<Eigen::Vector3d>& hull);
  void collectCostMatrixCenters(const vector<int>& grid_ids, bool active_centers,
      const vector<HullTask>* hull_tasks, vector<CenterMeta>& centers);
  void buildCostMatrix(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, const vector<int>& grid_ids,
      Eigen::MatrixXd& mat, bool for_global_tour, bool active_centers,
      const vector<HullTask>* hull_tasks);
  double computeGridAdjacencyConsistencyFactor(const double path_cost) const;
  void computePathExtraCostsFromPoints(const vector<Eigen::Vector3d>& path,
      const Eigen::Vector3d* start_vel, double& vel_change_cost, double& z_change_cost) const;
  void computePathExtraCostsFromGraph(ConnectivityGraph* cg, const std::vector<int>& node_path,
      const Eigen::Vector3d* start_vel, double& vel_change_cost, double& z_change_cost) const;
  double computeDroneToCenterCost(const Eigen::Vector3d& drone_pos, const Eigen::Vector3d& drone_vel,
      const Eigen::Vector3d& center_pos, UniformGrid* center_ug, int center_node_id,
      ConnectivityGraph* drone_cg, int drone_cg_node_id, int center_idx, double hybrid_radius,
      bool for_global_tour);
  double computeCenterToCenterCost(const Eigen::Vector3d& center_i_pos, UniformGrid* center_i_ug,
      int center_i_node_id, int center_i_idx, const Eigen::Vector3d& center_j_pos,
      UniformGrid* center_j_ug, int center_j_node_id, int center_j_idx, double hybrid_radius,
      bool for_global_tour);

  unique_ptr<UniformGrid> grid1_;  // Active planning layer

  shared_ptr<EDTEnvironment> edt_;
  // Swarm tf
  Eigen::Matrix3d rot_sw_;
  Eigen::Vector3d trans_sw_;
  bool use_swarm_tf_;
  double grid_size_;
  double w_distance_;         // Weight for distance cost (default: 1.0)
  double w_global_vel_change_ = 1.0;  // Legacy combined weight, kept as fallback.
  double w_global_dist_time_ = 1.0;   // Weight for dist_time = dist_cost / vm.
  double w_global_turn_time_ = 1.0;   // Weight for turning/velocity-change time.
  double w_global_z_change_ = 1.0;    // Weight for z-change time.
  double global_vm_ = 2.0;            // Reference max speed for global tour extra cost.
  double global_am_ = 2.0;            // Reference max acceleration for global tour extra cost.
  double global_vz_ = 2.0;            // Reference max vertical speed for z-change time.
  double alpha_consistency_;  // Alpha in rho = path_cost / (alpha * grid_size), default 1.2.
  double path_cost_adjacent_thresh_ = 1.0;  // Rho threshold for path-cost consistency factor.
  double convex_split_hull_area_ratio_;
  int drone_id_ = -1;

  // Mapping for the last cost matrix: center index -> grid id / position
  vector<Eigen::Vector3d> last_frontier_averages_;
  vector<int> last_center_grid_ids_;
  vector<Eigen::Vector3d> last_center_positions_;
  vector<vector<Eigen::Vector3d>> last_center_hulls_;
  vector<int> last_center_ids_;
  vector<int> last_center_types_;
};

}  // namespace c2_expl
#endif
