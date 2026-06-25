#ifndef _C2_EXPLORATION_MANAGER_H_
#define _C2_EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <limits>

#include <active_perception/hgrid.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::vector;

namespace c2_expl {
class EDTEnvironment;
class SDFMap;
class CommunicationGraph;
class FastPlannerManager;
// class UniformGrid;
class FrontierFinder;
struct ExplorationParam;
struct ExplorationData;
struct DroneState;

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED, NO_GRID };

class C2ExplorationManager {
public:
  C2ExplorationManager();
  ~C2ExplorationManager();

  void initialize(ros::NodeHandle& nh);

  int planExploreMotion(
      const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw);

  int planTrajToView(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
      const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw);

  int updateFrontierStruct(const Eigen::Vector3d& pos);

  struct AllocationRequest {
    vector<Eigen::Vector3d> drone_positions;
    vector<Eigen::Vector3d> drone_velocities;
    vector<int> drone_ids;
    vector<int> grid_ids;
    const vector<DroneState>* drone_states = nullptr;
    // Non-hull centers are protected by connectivity graph node id.
    vector<int> blocked_center_node_ids;
    // Split hull tasks are protected by hull geometry.
    vector<vector<Vector3d>> blocked_center_hulls;
  };

  struct AllocationResult {
    vector<vector<Vector3d>> centers;
    vector<vector<vector<Vector3d>>> center_hulls;
  };

  void allocateTasks(const AllocationRequest& request, AllocationResult& result);

  // Find optimal tour visiting unknown grid
  bool findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, vector<int>& ids, vector<vector<int>>& others,
      bool init = false);

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<CommunicationGraph> comm_graph_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<HGrid> hgrid_;
  shared_ptr<SDFMap> sdf_map_;
  // shared_ptr<UniformGrid> uniform_grid_;

private:
  static constexpr int kLkhCostScale = 100;
  static constexpr int kLkhPrecision = 1;
  static constexpr int kLkhMaxEdgeWeight =
      std::numeric_limits<int>::max() / 2 / kLkhPrecision - 1;
  static constexpr double kLkhFallbackBlockedCost = 1000.0;

  struct AllocationCandidateSet {
    Eigen::MatrixXd mat;
    vector<int> center_grid_ids;
    vector<Vector3d> center_positions;
    vector<vector<Vector3d>> center_hulls;
    vector<int> center_ids;
    vector<int> center_types;
    int full_center_num = 0;
  };

  // Find optimal coordinated tour for quadrotor swarm
  void findGridAndFrontierPath(const Vector3d& cur_pos, const Vector3d& cur_vel,
      const Vector3d& cur_yaw, vector<int>& grid_ids, vector<int>& ftr_ids);

  void shortenPath(vector<Vector3d>& path);

  void findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
      const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos, vector<int>& ids);

  bool buildAllocationCandidateSet(
      const AllocationRequest& request, AllocationCandidateSet& candidates);

  static int toLkhEdgeWeight(const double raw_cost, int& sanitized_count);
  static void parseMultiTours(
      const vector<int>& ids, const int drone_num, const int dimension, vector<vector<int>>& tours);
  static bool pointInsideConvexHull2D(
      const vector<Vector3d>& hull, const Vector3d& p, const double tol = 0.25);
  static Vector3d hullCentroid(const vector<Vector3d>& hull);
  static bool hasExplicitSplitCenter(
      const vector<int>& candidate_indices, const vector<vector<Vector3d>>& center_hulls);
  static bool candidateBlockedByHull(
      const Vector3d& center, const vector<vector<Vector3d>>& blocked_center_hulls);
  static void filterMeetingOptCentersByParticipantTasks(const shared_ptr<HGrid>& hgrid,
      const vector<DroneState>& participant_states, const vector<int>& center_grid_ids,
      const vector<Vector3d>& center_positions, const vector<int>& center_ids,
      const vector<int>& center_types, const vector<vector<Vector3d>>& center_hulls,
      vector<int>& selected_indices);
  static void collectAssignedHullTasks(const shared_ptr<HGrid>& hgrid,
      const vector<DroneState>* states, vector<HGrid::HullTask>& hull_tasks);
  static void keepCenterSubset(const vector<int>& keep_indices, const int drone_num,
      Eigen::MatrixXd& mat, vector<int>& center_grid_ids, vector<Vector3d>& center_positions,
      vector<vector<Vector3d>>& center_hulls, vector<int>& center_ids, vector<int>& center_types);

  shared_ptr<EDTEnvironment> edt_environment_;
  ros::ServiceClient tsp_client_, acvrp_client_;

public:
  typedef shared_ptr<C2ExplorationManager> Ptr;
};

}  // namespace c2_expl

#endif
