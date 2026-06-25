#include <ros/ros.h>
#include <exploration_manager/c2_exploration_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace c2_expl;

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  C2ExplorationFSM expl_fsm;
  expl_fsm.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
