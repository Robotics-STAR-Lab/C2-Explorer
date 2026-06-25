#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "bspline/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <active_perception/perception_utils.h>
#include <pcl_ros/point_cloud.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}
using c2_expl::NonUniformBspline;
using c2_expl::PerceptionUtils;

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub, traj_vel_pub;
;
nav_msgs::Odometry odom;
quadrotor_msgs::PositionCommand cmd;

// Info of generated traj
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
int pub_traj_id_, pub_traj_vel_id_;

shared_ptr<PerceptionUtils> percep_utils_;

// Info of replan
bool receive_traj_ = false;
double replan_time_;

// Executed traj, commanded and real ones
vector<Eigen::Vector3d> traj_cmd_, traj_real_, vel_cmd_;

// Data for benchmark comparison
ros::Time start_time, end_time, last_time;
double energy;

// Loop correction
Eigen::Matrix3d R_loop;
Eigen::Vector3d T_loop;
bool isLoopCorrection = false;

// Swarm
int drone_id_;
double color_r, color_g, color_b, color_a;
double max_vel_;

double calcPathLength(const vector<Eigen::Vector3d>& path) {
  if (path.empty()) return 0;
  double len = 0.0;
  for (int i = 0; i < path.size() - 1; ++i) {
    len += (path[i + 1] - path[i]).norm();
  }
  return len;
}

Eigen::Vector4d getColor(const double& h, const Eigen::Vector4d& color, double alpha) {
  double h1 = h;
  if (h1 < 0.0) {
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;

  if (h1 >= -1e-4 && h1 < 1.0 / 2) {
    lambda = (h1 - 0.0) * 2;
    color1 = Eigen::Vector4d(1, 1, 1, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else {
    lambda = (h1 - 1.0 / 2) * 2;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}

void displayTrajWithColor(
    vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

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
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void displayTrajVel(const vector<Eigen::Vector3d> positions,
    const vector<Eigen::Vector3d> velocities, Eigen::Vector4d color, int id) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.points.resize(positions.size());

  for (int i = 0; i < int(positions.size()); i++) {
    cloud.points[i].x = positions[i](0);
    cloud.points[i].y = positions[i](1);
    cloud.points[i].z = positions[i](2);

    double vel_norm = std::min(std::max(velocities[i].norm() / max_vel_, 0.0), 1.0);
    Eigen::Vector4d color = getColor(vel_norm, color, 1.0);
    cloud.points[i].r = color(0) * 255;
    cloud.points[i].g = color(1) * 255;
    cloud.points[i].b = color(2) * 255;
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  traj_vel_pub.publish(cloud_msg);
}

void drawFOV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  Eigen::Vector4d color(0, 0, 0, 1);

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  cmd_vis_pub.publish(mk);

  if (list1.size() == 0) return;

  // Pub new marker
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
  mk.action = visualization_msgs::Marker::ADD;
  cmd_vis_pub.publish(mk);
}

void replanCallback(std_msgs::Empty msg) {
  // Informed of new replan, end the current traj after some time
  const double time_out = 0.2;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out + replan_time_;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(std_msgs::Empty msg) {
  // Clear the executed traj data
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;
  odom = msg;
  traj_real_.push_back(Eigen::Vector3d(
      odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void pgTVioCallback(geometry_msgs::Pose msg) {
  // World to odom
  Eigen::Quaterniond q = Eigen::Quaterniond(
      msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  R_loop = q.toRotationMatrix();
  T_loop << msg.position.x, msg.position.y, msg.position.z;

  // cout << "R_loop: " << R_loop << endl;
  // cout << "T_loop: " << T_loop << endl;
}

void visCallback(const ros::TimerEvent& e) {
  // Draw the executed traj (desired state)
  // displayTrajWithColor(
  //     traj_cmd_, 0.2, Eigen::Vector4d(0.55, 0.27, 0.007, 1), pub_traj_id_);
  displayTrajWithColor(
      traj_cmd_, 0.3, Eigen::Vector4d(color_r, color_g, color_b, color_a), pub_traj_id_);
  displayTrajVel(
      traj_cmd_, vel_cmd_, Eigen::Vector4d(color_r, color_g, color_b, color_a), pub_traj_vel_id_);
}

void bsplineCallback(const bspline::BsplineConstPtr& msg) {
  // Received traj should have ascending traj_id
  if (msg->traj_id <= traj_id_) {
    ROS_ERROR("out of order bspline.");
    return;
  }

  // Parse the msg
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }
  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i) yaw_pts(i, 0) = msg->yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_.push_back(traj_[2].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;

  // Record the start time of flight
  if (start_time.isZero()) {
    ROS_WARN("start flight");
    start_time = ros::Time::now();
  }
}

void cmdCallback(const ros::TimerEvent& e) {
  // No publishing before receive traj data
  if (!receive_traj_) return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();
  Eigen::Vector3d pos, vel, acc, jer;
  double yaw, yawdot;

  if (t_cur < 0.0) {
    ROS_WARN_THROTTLE(1.0, "[Traj server]: traj start in future (%.3fs), hold last cmd", -t_cur);
    pos = Eigen::Vector3d(cmd.position.x, cmd.position.y, cmd.position.z);
    vel = Eigen::Vector3d(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
    acc = Eigen::Vector3d(cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z);
    yaw = cmd.yaw;
    yawdot = cmd.yaw_dot;

    jer.setZero();
  } else if (t_cur < traj_duration_) {
    // Current time within range of planned traj
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    jer = traj_[5].evaluateDeBoorT(t_cur);
  } else {
    // Current time exceed range of planned traj
    // keep publishing the final position and yaw
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = 0.0;
  }

  // Do not rotate yaw.
  yaw = 0;
  yawdot = 0;

  // Report info of the whole flight
  double len = calcPathLength(traj_cmd_);
  double flight_t = (end_time - start_time).toSec();
  ROS_WARN_THROTTLE(1.0,
      "\033[1;33mDrone %d [%lf,%lf,%lf,%lf], [time, length, vel, energy] \033[1;33m", drone_id_,
      flight_t, len, len / flight_t, energy);

  if (isLoopCorrection) {
    pos = R_loop.transpose() * (pos - T_loop);
    vel = R_loop.transpose() * vel;
    acc = R_loop.transpose() * acc;

    Eigen::Vector3d yaw_dir(cos(yaw), sin(yaw), 0);
    yaw_dir = R_loop.transpose() * yaw_dir;
    yaw = atan2(yaw_dir[1], yaw_dir[0]);
  }

  cmd.header.stamp = time_now;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;
  pos_cmd_pub.publish(cmd);

  percep_utils_->setPose(pos, yaw);
  vector<Eigen::Vector3d> l1, l2;
  percep_utils_->getFOV(l1, l2);
  drawFOV(l1, l2);

  // Record info of the executed traj
  if (traj_cmd_.size() == 0) {
    // Add the first position
    traj_cmd_.push_back(pos);
    vel_cmd_.push_back(vel);
  } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
    // Add new different commanded position
    traj_cmd_.push_back(pos);
    vel_cmd_.push_back(vel);
    double dt = (time_now - last_time).toSec();
    energy += jer.squaredNorm() * dt;
    end_time = ros::Time::now();

    if (energy > 10000) {
      std::cout << "jer: " << jer.transpose() << ", dt: " << dt << std::endl;
    }
  }
  last_time = time_now;

  // if (traj_cmd_.size() > 100000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);
  ros::Subscriber pg_T_vio_sub = node.subscribe("/loop_fusion/pg_T_vio", 10, pgTVioCallback);

  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);
  traj_vel_pub = node.advertise<sensor_msgs::PointCloud2>("planning/travel_traj_vel", 10);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  nh.param("traj_server/pub_traj_id", pub_traj_id_, -1);
  nh.param("traj_server/pub_traj_vel_id", pub_traj_vel_id_, -1);
  nh.param("fsm/replan_time", replan_time_, 0.1);
  nh.param("loop_correction/isLoopCorrection", isLoopCorrection, false);
  nh.param("traj_server/drone_id", drone_id_, 1);
  nh.param("traj_server/max_vel", max_vel_, 2.0);
  nh.param("color_r", color_r, 1.0);
  nh.param("color_g", color_g, 0.0);
  nh.param("color_b", color_b, 0.0);
  nh.param("color_a", color_a, 1.0);

  ROS_WARN("[Traj server]: init...");
  ros::Duration(1.0).sleep();

  // Control parameter
  cmd.kx = { 5.7, 5.7, 6.2 };
  cmd.kv = { 3.4, 3.4, 4.0 };

  std::cout << start_time.toSec() << std::endl;
  std::cout << end_time.toSec() << std::endl;

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = -4.5;
  cmd.position.y = -3.0;
  cmd.position.z = 0.0;
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  cmd.yaw = 0.0;
  cmd.yaw_dot = 0.0;

  percep_utils_.reset(new PerceptionUtils(nh));

  // Initialization for exploration
  // ros::Duration(2.0).sleep();
  // for (int i = 0; i < 100; ++i) {
  //   cmd.position.z += 0.01;
  //   pos_cmd_pub.publish(cmd);
  //   ros::Duration(0.01).sleep();
  // }
  // for (int i = 0; i < 100; ++i) {
  //   cmd.position.z -= 0.01;
  //   pos_cmd_pub.publish(cmd);
  //   ros::Duration(0.01).sleep();
  // }
  // for (int i = 0; i < 100; ++i) {
  //   cmd.position.z += 0.01;
  //   pos_cmd_pub.publish(cmd);
  //   ros::Duration(0.01).sleep();
  // }

  R_loop = Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();
  T_loop = Eigen::Vector3d(0, 0, 0);

  ROS_WARN("[Traj server]: ready.");
  ros::spin();

  return 0;
}
