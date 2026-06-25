#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <string>
#include <fstream>
#include <vector>
#include <sys/wait.h>

#include <lkh_mtsp_solver/lkh3_interface.h>
#include <lkh_mtsp_solver/SolveMTSP.h>

using std::string;

std::string mtsp_dir1_;
std::string mtsp_dir2_;
std::string mtsp_dir3_;
int drone_id_, problem_id_;

bool forcePrecisionOne(const std::string& par_file) {
  std::ifstream fin(par_file);
  if (!fin.is_open()) {
    ROS_ERROR("Failed to open par file for precision patch: %s", par_file.c_str());
    return false;
  }

  std::vector<std::string> lines;
  lines.reserve(64);
  std::string line;
  while (std::getline(fin, line)) {
    if (line.rfind("PRECISION", 0) == 0) continue;
    lines.push_back(line);
  }
  fin.close();

  std::ofstream fout(par_file, std::ios::trunc);
  if (!fout.is_open()) {
    ROS_ERROR("Failed to rewrite par file for precision patch: %s", par_file.c_str());
    return false;
  }
  for (const auto& l : lines) {
    fout << l << "\n";
  }
  fout << "PRECISION = 1\n";
  fout.close();
  return true;
}

bool mtspCallback(
    lkh_mtsp_solver::SolveMTSP::Request& req, lkh_mtsp_solver::SolveMTSP::Response& res) {

  (void)res;
  if (req.prob == 1)
    solveMTSPWithLKH3(mtsp_dir1_.c_str());
  else if (req.prob == 2)
    solveMTSPWithLKH3(mtsp_dir2_.c_str());
  else if (req.prob == 3) {
    // solveMTSPWithLKH3(mtsp_dir3_.c_str());
    if (!forcePrecisionOne(mtsp_dir3_)) {
      return false;
    }
    string cmd = "/usr/local/bin/LKH \"" + mtsp_dir3_ + "\"";
    const int ret = system(cmd.c_str());
    if (ret != 0) {
      if (WIFSIGNALED(ret)) {
        ROS_ERROR("LKH process terminated by signal %d (par: %s)", WTERMSIG(ret),
            mtsp_dir3_.c_str());
      } else {
        ROS_ERROR("LKH process failed with status %d (par: %s)", ret, mtsp_dir3_.c_str());
      }
      return false;
    }
  } else {
    ROS_ERROR("Unknown MTSP request type: %u", req.prob);
    return false;
  }

  // ROS_INFO("MTSP server %d solve prob", drone_id_);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mtsp_node");
  ros::NodeHandle nh("~");

  // Read mtsp file dir
  std::string mtsp_dir;
  nh.param("exploration/mtsp_dir", mtsp_dir, std::string("null"));
  nh.param("exploration/drone_id", drone_id_, 1);
  nh.param("exploration/problem_id", problem_id_, 1);

  mtsp_dir1_ = mtsp_dir + "/amtsp_" + std::to_string(drone_id_) + ".par";
  mtsp_dir2_ = mtsp_dir + "/amtsp2_" + std::to_string(drone_id_) + ".par";
  mtsp_dir3_ = mtsp_dir + "/amtsp3_" + std::to_string(drone_id_) + ".par";

  string service_name;
  if (problem_id_ == 1) {  // TSP
    service_name = "/solve_tsp_" + std::to_string(drone_id_);
  } else if (problem_id_ == 2) {  // ACVRP
    service_name = "/solve_acvrp_" + std::to_string(drone_id_);
  }
  ros::ServiceServer mtsp_server = nh.advertiseService(service_name, mtspCallback);

  ROS_WARN("MTSP server %d is ready.", drone_id_);
  ros::spin();

  return 1;
}
