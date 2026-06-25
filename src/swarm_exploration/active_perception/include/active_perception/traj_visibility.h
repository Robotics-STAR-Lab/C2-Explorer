#ifndef _TRAJ_VISIBILITY_H_
#define _TRAJ_VISIBILITY_H_

#include <Eigen/Eigen>

namespace c2_expl {
struct VisiblePair {
  int from_;            // idx of qi, see qj
  int to_;              // idx of qj, can be seen by qi
  Eigen::Vector3d qb_;  // cell blocking view from qi->qj
};

struct ViewConstraint {
  Eigen::Vector3d pt_;     // unknown point along the traj
  Eigen::Vector3d pc_;     // critical view point
  Eigen::Vector3d dir_;    // critical view direction with safe length
  Eigen::Vector3d pcons_;  // pt to add view constraint
  int idx_;                // idx to add view constraint
};

}  // namespace c2_expl
#endif
