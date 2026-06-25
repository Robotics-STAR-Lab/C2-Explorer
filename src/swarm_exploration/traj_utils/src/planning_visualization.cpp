#include <traj_utils/planning_visualization.h>
#include <iomanip>
#include <sstream>
#include <unordered_map>

using std::cout;
using std::endl;
namespace c2_expl {
namespace {
constexpr double kConnectivityEdgeCostTextScale = 0.30;
}  // namespace

PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh) {
  node = nh;

  traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 100);
  pubs_.push_back(traj_pub_);

  predict_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/prediction", 100);
  pubs_.push_back(predict_pub_);

  visib_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/"
                                                          "visib_constraint",
      100);
  pubs_.push_back(visib_pub_);

  frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 10000);
  pubs_.push_back(frontier_pub_);

  yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 100);
  pubs_.push_back(yaw_pub_);

  viewpoint_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/viewpoints", 1000);
  pubs_.push_back(viewpoint_pub_);

  hgrid_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/hgrid", 1000);
  pubs_.push_back(hgrid_pub_);

  pose_pub_ = node.advertise<geometry_msgs::PoseStamped>("/planning_vis/pose", 1000);
  pubs_.push_back(pose_pub_);

  pose_array_pub_ = node.advertise<geometry_msgs::PoseArray>("/planning_vis/pose_array", 1000);
  pubs_.push_back(pose_array_pub_);

  connectivity_graph_pub_ =
      node.advertise<visualization_msgs::Marker>("/planning_vis/connectivity_graph", 1000);
  pubs_.push_back(connectivity_graph_pub_);

  last_bspline_phase1_num_ = 0;
  last_bspline_phase2_num_ = 0;
  last_frontier_num_ = 0;
}

void PlanningVisualization::fillBasicInfo(visualization_msgs::Marker& mk,
    const Eigen::Vector3d& scale, const Eigen::Vector4d& color, const string& ns, const int& id,
    const int& shape) {
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = id;
  mk.ns = ns;
  mk.type = shape;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = scale[0];
  mk.scale.y = scale[1];
  mk.scale.z = scale[2];
}

void PlanningVisualization::fillGeometryInfo(
    visualization_msgs::Marker& mk, const vector<Eigen::Vector3d>& list) {
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    if (!list[i].allFinite()) continue;
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
}

void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker& mk,
    const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2) {
  geometry_msgs::Point pt;
  const int n = std::min(list1.size(), list2.size());
  for (int i = 0; i < n; ++i) {
    if (!list1[i].allFinite() || !list2[i].allFinite()) continue;

    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
}

void PlanningVisualization::drawBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale,
    const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::Marker::CUBE);
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  mk.pose.position.x = center[0];
  mk.pose.position.y = center[1];
  mk.pose.position.z = center[2];
  mk.action = visualization_msgs::Marker::ADD;

  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawText(const Eigen::Vector3d& pos, const string& text,
    const double& scale, const Eigen::Vector4d& color, const string& ns, const int& id,
    const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
      visualization_msgs::Marker::TEXT_VIEW_FACING);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  // pub new marker
  mk.text = text;
  mk.pose.position.x = pos[0];
  mk.pose.position.y = pos[1];
  mk.pose.position.z = pos[2];
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawConnectivityGraph(const vector<Eigen::Vector3d>& unknown_edge_start,
    const vector<Eigen::Vector3d>& unknown_edge_end, const vector<Eigen::Vector3d>& free_edge_start,
    const vector<Eigen::Vector3d>& free_edge_end,
    const vector<Eigen::Vector3d>& portal_edge_start,
    const vector<Eigen::Vector3d>& portal_edge_end,
    const vector<Eigen::Vector3d>& active_unknown_nodes,
    const vector<Eigen::Vector3d>& active_free_nodes,
    const vector<Eigen::Vector3d>& inactive_unknown_nodes,
    const vector<Eigen::Vector3d>& inactive_free_nodes,
    const vector<Eigen::Vector3d>& edge_cost_pos, const vector<double>& edge_costs,
    const string& ns_prefix, const int& pub_id) {
  drawLines(unknown_edge_start, unknown_edge_end, 0.15, Color::Gray(), ns_prefix + "_edge", 0, pub_id);
  drawLines(free_edge_start, free_edge_end, 0.15, Color::Yellow(), ns_prefix + "_edge", 1, pub_id);
  drawLines(
      portal_edge_start, portal_edge_end, 0.15, Color::LightBlue(), ns_prefix + "_edge", 2, pub_id);
  const Eigen::Vector4d active_unknown_color(100.0 / 255.0, 149.0 / 255.0, 237.0 / 255.0, 1.0);
  const Eigen::Vector4d active_free_color(255.0 / 255.0, 99.0 / 255.0, 71.0 / 255.0, 1.0);
  const Eigen::Vector4d inactive_unknown_color(80.0 / 255.0, 80.0 / 255.0, 80.0 / 255.0, 1.0);
  const Eigen::Vector4d inactive_free_color(233.0 / 255.0, 150.0 / 255.0, 122.0 / 255.0, 1.0);
  // Make graph centers visually dominant over edges.
  drawSpheres(active_unknown_nodes, 0.38, active_unknown_color, ns_prefix + "_node", 3, pub_id);
  drawSpheres(active_free_nodes, 0.38, active_free_color, ns_prefix + "_node", 4, pub_id);
  drawSpheres(inactive_unknown_nodes, 0.38, inactive_unknown_color, ns_prefix + "_node", 5, pub_id);
  drawSpheres(inactive_free_nodes, 0.38, inactive_free_color, ns_prefix + "_node", 6, pub_id);

  if (pubs_[pub_id].getNumSubscribers() == 0) return;

  static std::unordered_map<std::string, int> last_text_num;
  const std::string key = ns_prefix + "_edge_cost_" + std::to_string(pub_id);
  const int prev = (last_text_num.count(key) > 0) ? last_text_num[key] : 0;
  const int cur = static_cast<int>(std::min(edge_cost_pos.size(), edge_costs.size()));

  visualization_msgs::Marker mk;
  fillBasicInfo(mk,
      Eigen::Vector3d(kConnectivityEdgeCostTextScale, kConnectivityEdgeCostTextScale,
          std::max(0.08, kConnectivityEdgeCostTextScale)),
      Color::White(), ns_prefix + "_edge_cost", 0, visualization_msgs::Marker::TEXT_VIEW_FACING);
  mk.pose.orientation.w = 1.0;

  for (int i = cur; i < prev; ++i) {
    mk.id = 100000 + i;
    mk.action = visualization_msgs::Marker::DELETE;
    pubs_[pub_id].publish(mk);
  }

  for (int i = 0; i < cur; ++i) {
    mk.id = 100000 + i;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = edge_cost_pos[i].x();
    mk.pose.position.y = edge_cost_pos[i].y();
    mk.pose.position.z = edge_cost_pos[i].z();
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << edge_costs[i];
    mk.text = oss.str();
    pubs_[pub_id].publish(mk);
  }

  last_text_num[key] = cur;
}

void PlanningVisualization::drawSpheres(const vector<Eigen::Vector3d>& list, const double& scale,
    const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
      visualization_msgs::Marker::SPHERE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list.size() == 0) return;

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawCubes(const vector<Eigen::Vector3d>& list, const double& scale,
    const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
      visualization_msgs::Marker::CUBE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list.size() == 0) return;

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d>& list1,
    const vector<Eigen::Vector3d>& list2, const double& scale, const Eigen::Vector4d& color,
    const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
      visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list1.size() == 0) return;

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d>& list, const double& scale,
    const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
      visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list.size() == 0) return;

  // split the single list into two
  vector<Eigen::Vector3d> list1, list2;
  for (int i = 0; i < list.size() - 1; ++i) {
    list1.push_back(list[i]);
    list2.push_back(list[i + 1]);
  }

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d>& list,
    double resolution, const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
    const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d>& list1,
    const vector<Eigen::Vector3d>& list2, double line_width, const Eigen::Vector4d& color, int id,
    int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawPose(const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation, const string& ns, const int& id) {
  if (pubs_[PlanningVisualization::PUBLISHER::POSE].getNumSubscribers() == 0) return;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
  pose.pose.orientation.w = orientation.w();

  pubs_[PlanningVisualization::PUBLISHER::POSE].publish(pose);
}

void PlanningVisualization::drawPoses(
    const vector<pair<Eigen::Vector3d, Eigen::Quaterniond>>& poses, const string& ns) {
  if (pubs_[PlanningVisualization::PUBLISHER::POSE_ARRAY].getNumSubscribers() == 0) return;

  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = "world";
  pose_array.header.stamp = ros::Time::now();

  for (const auto& pose : poses) {
    geometry_msgs::Pose p;
    p.position.x = pose.first[0];
    p.position.y = pose.first[1];
    p.position.z = pose.first[2];
    p.orientation.x = pose.second.x();
    p.orientation.y = pose.second.y();
    p.orientation.z = pose.second.z();
    p.orientation.w = pose.second.w();
    pose_array.poses.push_back(p);
  }

  pubs_[PlanningVisualization::PUBLISHER::POSE_ARRAY].publish(pose_array);
}

void PlanningVisualization::drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size) {
  vector<Eigen::Vector3d> empty;

  for (int i = 0; i < last_bspline_phase1_num_; ++i) {
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + i % 100);
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + i % 100);
  }
  last_bspline_phase1_num_ = bsplines.size();

  for (int i = 0; i < bsplines.size(); ++i) {
    drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.2), false, 2 * size,
        getColor(double(i) / bsplines.size()), i);
  }
}

void PlanningVisualization::drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size) {
  vector<Eigen::Vector3d> empty;

  for (int i = 0; i < last_bspline_phase2_num_; ++i) {
    drawSpheres(empty, size, Eigen::Vector4d(1, 0, 0, 1), "B-Spline", i, 0);
    drawSpheres(empty, size, Eigen::Vector4d(1, 0, 0, 1), "B-Spline", i + 50, 0);
    // displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + (50 + i) % 100);
    // displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + (50 + i) %
    // 100);
  }
  last_bspline_phase2_num_ = bsplines.size();

  for (int i = 0; i < bsplines.size(); ++i) {
    drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.6), false, 1.5 * size,
        getColor(double(i) / bsplines.size()), i);
  }
}

void PlanningVisualization::drawBspline(NonUniformBspline& bspline, double size,
    const Eigen::Vector4d& color, bool show_ctrl_pts, double size2, const Eigen::Vector4d& color2,
    int id1) {
  if (bspline.getControlPoint().size() == 0) return;

  vector<Eigen::Vector3d> traj_pts;
  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);

  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  // displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);
  drawSpheres(traj_pts, size, color, "B-Spline", id1, 0);

  // draw the control point
  if (show_ctrl_pts) {
    Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
    vector<Eigen::Vector3d> ctp;
    for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
      Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
      ctp.push_back(pt);
    }
    // displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
    drawSpheres(ctp, size2, color2, "B-Spline", id1 + 50, 0);
  }
}

void PlanningVisualization::drawGoal(
    Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> goal_vec = { goal };
  displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

void PlanningVisualization::drawGeometricPath(
    const vector<Eigen::Vector3d>& path, double resolution, const Eigen::Vector4d& color, int id) {
  displaySphereList(path, resolution, color, PATH + id % 100);
}

void PlanningVisualization::drawPolynomialTraj(
    PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> poly_pts;
  poly_traj.getSamplePoints(poly_pts);
  displaySphereList(poly_pts, resolution, color, POLY_TRAJ + id % 100);
}

void PlanningVisualization::drawPrediction(
    ObjPrediction pred, double resolution, const Eigen::Vector4d& color, int id) {
  ros::Time time_now = ros::Time::now();
  double start_time = (time_now - ObjHistory::global_start_time_).toSec();
  const double range = 5.6;

  vector<Eigen::Vector3d> traj;
  for (int i = 0; i < pred->size(); i++) {
    PolynomialPrediction poly = pred->at(i);
    if (!poly.valid()) continue;

    for (double t = start_time; t <= start_time + range; t += 0.8) {
      Eigen::Vector3d pt = poly.evaluateConstVel(t);
      traj.push_back(pt);
    }
  }
  displaySphereList(traj, resolution, color, id % 100, PlanningVisualization::PUBLISHER::PREDICTION);
}

void PlanningVisualization::drawVisibConstraint(
    const Eigen::MatrixXd& ctrl_pts, const vector<Eigen::Vector3d>& block_pts) {
  int visible_num = ctrl_pts.rows() - block_pts.size();

  /* draw block points, their projection rays and visible pairs */
  vector<Eigen::Vector3d> pts1, pts2, pts3, pts4;
  int n = ctrl_pts.rows() - visible_num;

  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d qb = block_pts[i];

    if (fabs(qb[2] + 10086) > 1e-3) {
      // compute the projection
      Eigen::Vector3d qi = ctrl_pts.row(i);
      Eigen::Vector3d qj = ctrl_pts.row(i + visible_num);
      Eigen::Vector3d dir = (qj - qi).normalized();
      Eigen::Vector3d qp = qi + dir * ((qb - qi).dot(dir));

      pts1.push_back(qb);
      pts2.push_back(qp);
      pts3.push_back(qi);
      pts4.push_back(qj);
    }
  }

  displayCubeList(pts1, 0.1, Eigen::Vector4d(1, 1, 0, 1), 0, PlanningVisualization::PUBLISHER::VISIBILITY);
  displaySphereList(
      pts4, 0.2, Eigen::Vector4d(0, 1, 0, 1), 1, PlanningVisualization::PUBLISHER::VISIBILITY);
  displayLineList(
      pts1, pts2, 0.015, Eigen::Vector4d(0, 1, 1, 1), 2, PlanningVisualization::PUBLISHER::VISIBILITY);
  displayLineList(
      pts3, pts4, 0.015, Eigen::Vector4d(0, 1, 0, 1), 3, PlanningVisualization::PUBLISHER::VISIBILITY);
}

void PlanningVisualization::drawVisibConstraint(
    const Eigen::MatrixXd& pts, const vector<VisiblePair>& pairs) {
  vector<Eigen::Vector3d> pts1, pts2, pts3, pts4;
  for (auto pr : pairs) {
    Eigen::Vector3d qb = pr.qb_;
    Eigen::Vector3d qi = pts.row(pr.from_);
    Eigen::Vector3d qj = pts.row(pr.to_);
    Eigen::Vector3d dir = (qj - qi).normalized();
    Eigen::Vector3d qp = qi + dir * ((qb - qi).dot(dir));
    pts1.push_back(qb);
    pts2.push_back(qp);
    pts3.push_back(qi);
    pts4.push_back(qj);
  }
  displayCubeList(pts1, 0.1, Eigen::Vector4d(1, 1, 0, 1), 0, PlanningVisualization::PUBLISHER::VISIBILITY);
  displaySphereList(
      pts4, 0.2, Eigen::Vector4d(0, 1, 0, 1), 1, PlanningVisualization::PUBLISHER::VISIBILITY);
  displayLineList(
      pts1, pts2, 0.015, Eigen::Vector4d(0, 1, 1, 1), 2, PlanningVisualization::PUBLISHER::VISIBILITY);
  displayLineList(
      pts3, pts4, 0.015, Eigen::Vector4d(0, 1, 0, 1), 3, PlanningVisualization::PUBLISHER::VISIBILITY);
}

void PlanningVisualization::drawViewConstraint(const ViewConstraint& vc) {
  if (vc.idx_ < 0) return;
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.type = visualization_msgs::Marker::ARROW;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = 0.1;
  mk.scale.y = 0.2;
  mk.scale.z = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.5;
  mk.color.b = 0.0;
  mk.color.a = 1.0;

  geometry_msgs::Point pt;
  pt.x = vc.pt_[0];
  pt.y = vc.pt_[1];
  pt.z = vc.pt_[2];
  mk.points.push_back(pt);
  pt.x = vc.pt_[0] + vc.dir_[0];
  pt.y = vc.pt_[1] + vc.dir_[1];
  pt.z = vc.pt_[2] + vc.dir_[2];
  mk.points.push_back(pt);
  pubs_[PlanningVisualization::PUBLISHER::VISIBILITY].publish(mk);

  vector<Eigen::Vector3d> pts = { vc.pcons_ };
  displaySphereList(pts, 0.2, Eigen::Vector4d(0, 1, 0, 1), 1,
      PlanningVisualization::PUBLISHER::VISIBILITY);
}

void PlanningVisualization::drawFrontier(const vector<vector<Eigen::Vector3d>>& frontiers) {
  for (int i = 0; i < frontiers.size(); ++i) {
    // displayCubeList(frontiers[i], 0.1, getColor(double(i) / frontiers.size(),
    // 0.4), i, 4);
    drawCubes(frontiers[i], 0.1, getColor(double(i) / frontiers.size(), 0.8), "frontier", i,
        PlanningVisualization::PUBLISHER::FRONTIER);
  }

  vector<Eigen::Vector3d> frontier;
  for (int i = frontiers.size(); i < last_frontier_num_; ++i) {
    // displayCubeList(frontier, 0.1, getColor(1), i, 4);
    drawCubes(frontier, 0.1, getColor(1), "frontier", i, PlanningVisualization::PUBLISHER::FRONTIER);
  }
  last_frontier_num_ = frontiers.size();
}

void PlanningVisualization::drawYawTraj(
    NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt) {
  double duration = pos.getTimeSum();
  vector<Eigen::Vector3d> pts1, pts2;

  for (double tc = 0.0; tc <= duration + 1e-3; tc += dt) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    pc[2] += 0.15;
    double yc = yaw.evaluateDeBoorT(tc)[0];
    Eigen::Vector3d dir(cos(yc), sin(yc), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(
      pts1, pts2, 0.04, Eigen::Vector4d(1, 0.5, 0, 1), 0, PlanningVisualization::PUBLISHER::YAW_TRAJ);
}

void PlanningVisualization::drawYawPath(
    NonUniformBspline& pos, const vector<double>& yaw, const double& dt) {
  vector<Eigen::Vector3d> pts1, pts2;

  for (int i = 0; i < yaw.size(); ++i) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(i * dt);
    pc[2] += 0.3;
    Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(
      pts1, pts2, 0.04, Eigen::Vector4d(1, 0, 1, 1), 1, PlanningVisualization::PUBLISHER::YAW_TRAJ);
}

Eigen::Vector4d PlanningVisualization::getColor(const double& h, double alpha) {
  double h1 = h;
  if (h1 < 0.0 || h1 > 1.0) {
    std::cout << "h out of range" << std::endl;
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;
  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = (h1 - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);
  } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);
  } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);
  } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);
  } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
    lambda = (h1 - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}
// PlanningVisualization::
}  // namespace c2_expl
