#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

// Swarm exploration messages
#include <exploration_manager/DroneState.h>
#include <exploration_manager/MeetingOpt.h>
#include <exploration_manager/MeetingOptResponse.h>
#include <bspline/Bspline.h>
#include <plan_env/Heartbeat.h>
#include <plan_env/ChunkStamps.h>
#include <plan_env/ChunkData.h>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#define UDP_PORT 8081
#define BUF_LEN 1048576  // 1MB

using namespace std;

int udp_server_fd_, udp_send_fd_;
string udp_ip_;
int drone_id_;
char udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
struct sockaddr_in addr_udp_send_;

// Subscribers
ros::Subscriber drone_state_sub_;
ros::Subscriber meeting_opt_sub_;
ros::Subscriber meeting_opt_res_sub_;
ros::Subscriber swarm_traj_sub_;
ros::Subscriber heartbeat_sub_;
ros::Subscriber chunk_stamps_sub_;
ros::Subscriber chunk_data_sub_;

// Publishers
ros::Publisher drone_state_pub_;
ros::Publisher meeting_opt_pub_;
ros::Publisher meeting_opt_res_pub_;
ros::Publisher swarm_traj_pub_;
ros::Publisher heartbeat_pub_;
ros::Publisher chunk_stamps_pub_;
ros::Publisher chunk_data_pub_;

// Message buffers for receiving
exploration_manager::DroneState drone_state_msg_;
exploration_manager::MeetingOpt meeting_opt_msg_;
exploration_manager::MeetingOptResponse meeting_opt_res_msg_;
bspline::Bspline swarm_traj_msg_;
plan_env::Heartbeat heartbeat_msg_;
plan_env::ChunkStamps chunk_stamps_msg_;
plan_env::ChunkData chunk_data_msg_;

enum MESSAGE_TYPE {
  DRONE_STATE = 100,
  MEETING_OPT,
  MEETING_OPT_RES,
  SWARM_TRAJ,
  HEARTBEAT = 106,
  CHUNK_STAMPS,
  CHUNK_DATA
};

int init_broadcast(const char* ip, const int port) {
  int fd;

  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {
    ROS_ERROR("[bridge_node]Socket sender creation error!");
    exit(EXIT_FAILURE);
  }

  int so_broadcast = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0) {
    cout << "Error in setting Broadcast option";
    exit(EXIT_FAILURE);
  }

  addr_udp_send_.sin_family = AF_INET;
  addr_udp_send_.sin_port = htons(port);

  if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0) {
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }

  return fd;
}

int udp_bind_to_port(const int port, int& server_fd) {
  struct sockaddr_in address;
  int opt = 1;

  if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
    perror("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  return server_fd;
}

template <typename T>
int serializeTopic(const MESSAGE_TYPE msg_type, const T& msg) {
  auto ptr = (uint8_t*)(udp_send_buf_);

  *((MESSAGE_TYPE*)ptr) = msg_type;
  ptr += sizeof(MESSAGE_TYPE);

  namespace ser = ros::serialization;
  uint32_t msg_size = ser::serializationLength(msg);

  *((uint32_t*)ptr) = msg_size;
  ptr += sizeof(uint32_t);

  ser::OStream stream(ptr, msg_size);
  ser::serialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

template <typename T>
int deserializeTopic(T& msg) {
  auto ptr = (uint8_t*)(udp_recv_buf_ + sizeof(MESSAGE_TYPE));

  uint32_t msg_size = *((uint32_t*)ptr);
  ptr += sizeof(uint32_t);

  namespace ser = ros::serialization;
  ser::IStream stream(ptr, msg_size);
  ser::deserialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

// ==================== Callbacks for sending ====================

void droneStateCallback(const exploration_manager::DroneStatePtr& msg) {
  int len = serializeTopic(MESSAGE_TYPE::DRONE_STATE, *msg);
  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr*)&addr_udp_send_,
          sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("UDP SEND ERROR (DRONE_STATE)!!!");
  }
}

void meetingOptCallback(const exploration_manager::MeetingOptPtr& msg) {
  int len = serializeTopic(MESSAGE_TYPE::MEETING_OPT, *msg);
  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr*)&addr_udp_send_,
          sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("UDP SEND ERROR (MEETING_OPT)!!!");
  }
}

void meetingOptResCallback(const exploration_manager::MeetingOptResponsePtr& msg) {
  int len = serializeTopic(MESSAGE_TYPE::MEETING_OPT_RES, *msg);
  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr*)&addr_udp_send_,
          sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("UDP SEND ERROR (MEETING_OPT_RES)!!!");
  }
}

void swarmTrajCallback(const bspline::BsplinePtr& msg) {
  int len = serializeTopic(MESSAGE_TYPE::SWARM_TRAJ, *msg);
  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr*)&addr_udp_send_,
          sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("UDP SEND ERROR (SWARM_TRAJ)!!!");
  }
}

void heartbeatCallback(const plan_env::HeartbeatPtr& msg) {
  int len = serializeTopic(MESSAGE_TYPE::HEARTBEAT, *msg);
  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr*)&addr_udp_send_,
          sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("UDP SEND ERROR (HEARTBEAT)!!!");
  }
}

void chunkStampsCallback(const plan_env::ChunkStampsPtr& msg) {
  int len = serializeTopic(MESSAGE_TYPE::CHUNK_STAMPS, *msg);
  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr*)&addr_udp_send_,
          sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("UDP SEND ERROR (CHUNK_STAMPS)!!!");
  }
}

void chunkDataCallback(const plan_env::ChunkDataPtr& msg) {
  int len = serializeTopic(MESSAGE_TYPE::CHUNK_DATA, *msg);
  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr*)&addr_udp_send_,
          sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("UDP SEND ERROR (CHUNK_DATA)!!!");
  }
}

// ==================== UDP Receive Thread ====================

void udp_recv_fun() {
  int valread;
  struct sockaddr_in addr_client;
  socklen_t addr_len = sizeof(addr_client);

  if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0) {
    ROS_ERROR("[bridge_node]Socket receiver creation error!");
    exit(EXIT_FAILURE);
  }

  while (true) {
    if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0,
             (struct sockaddr*)&addr_client, (socklen_t*)&addr_len)) < 0) {
      perror("recvfrom() < 0, error:");
      exit(EXIT_FAILURE);
    }

    char* ptr = udp_recv_buf_;
    switch (*((MESSAGE_TYPE*)ptr)) {
      case MESSAGE_TYPE::DRONE_STATE: {
        if (valread == deserializeTopic(drone_state_msg_)) {
          drone_state_pub_.publish(drone_state_msg_);
        } else {
          ROS_ERROR("Received message length mismatch (DRONE_STATE)!!!");
        }
        break;
      }

      case MESSAGE_TYPE::MEETING_OPT: {
        if (valread == deserializeTopic(meeting_opt_msg_)) {
          meeting_opt_pub_.publish(meeting_opt_msg_);
        } else {
          ROS_ERROR("Received message length mismatch (MEETING_OPT)!!!");
        }
        break;
      }

      case MESSAGE_TYPE::MEETING_OPT_RES: {
        if (valread == deserializeTopic(meeting_opt_res_msg_)) {
          meeting_opt_res_pub_.publish(meeting_opt_res_msg_);
        } else {
          ROS_ERROR("Received message length mismatch (MEETING_OPT_RES)!!!");
        }
        break;
      }

      case MESSAGE_TYPE::SWARM_TRAJ: {
        if (valread == deserializeTopic(swarm_traj_msg_)) {
          swarm_traj_pub_.publish(swarm_traj_msg_);
        } else {
          ROS_ERROR("Received message length mismatch (SWARM_TRAJ)!!!");
        }
        break;
      }

      case MESSAGE_TYPE::HEARTBEAT: {
        if (valread == deserializeTopic(heartbeat_msg_)) {
          heartbeat_pub_.publish(heartbeat_msg_);
        } else {
          ROS_ERROR("Received message length mismatch (HEARTBEAT)!!!");
        }
        break;
      }

      case MESSAGE_TYPE::CHUNK_STAMPS: {
        if (valread == deserializeTopic(chunk_stamps_msg_)) {
          chunk_stamps_pub_.publish(chunk_stamps_msg_);
        } else {
          ROS_ERROR("Received message length mismatch (CHUNK_STAMPS)!!!");
        }
        break;
      }

      case MESSAGE_TYPE::CHUNK_DATA: {
        if (valread == deserializeTopic(chunk_data_msg_)) {
          chunk_data_pub_.publish(chunk_data_msg_);
        } else {
          ROS_ERROR("Received message length mismatch (CHUNK_DATA)!!!");
        }
        break;
      }

      default:
        ROS_WARN("Unknown received message type: %d", *((MESSAGE_TYPE*)ptr));
        break;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  nh.param("broadcast_ip", udp_ip_, string("127.0.0.255"));
  nh.param("drone_id", drone_id_, -1);

  if (drone_id_ == -1) {
    ROS_WARN("[swarm bridge] Wrong drone_id!");
    exit(EXIT_FAILURE);
  }

  // Subscribers (receive from local nodes, send via UDP)
  drone_state_sub_ = nh.subscribe(
      "/swarm_expl/drone_state_send", 10, droneStateCallback, ros::TransportHints().tcpNoDelay());
  meeting_opt_sub_ = nh.subscribe(
      "/swarm_expl/meeting_opt_send", 100, meetingOptCallback, ros::TransportHints().tcpNoDelay());
  meeting_opt_res_sub_ = nh.subscribe(
      "/swarm_expl/meeting_opt_res_send", 10, meetingOptResCallback, ros::TransportHints().tcpNoDelay());
  swarm_traj_sub_ = nh.subscribe(
      "/planning/swarm_traj_send", 100, swarmTrajCallback, ros::TransportHints().tcpNoDelay());
  heartbeat_sub_ = nh.subscribe(
      "/communication/heartbeat_send", 100, heartbeatCallback, ros::TransportHints().tcpNoDelay());
  chunk_stamps_sub_ = nh.subscribe("/multi_map_manager/chunk_stamps_send", 10,
      chunkStampsCallback, ros::TransportHints().tcpNoDelay());
  chunk_data_sub_ = nh.subscribe("/multi_map_manager/chunk_data_send", 5000, chunkDataCallback,
      ros::TransportHints().tcpNoDelay());

  // Publishers (receive from UDP, publish to local nodes)
  drone_state_pub_ =
      nh.advertise<exploration_manager::DroneState>("/swarm_expl/drone_state_recv", 10);
  meeting_opt_pub_ = nh.advertise<exploration_manager::MeetingOpt>("/swarm_expl/meeting_opt_recv", 100);
  meeting_opt_res_pub_ =
      nh.advertise<exploration_manager::MeetingOptResponse>("/swarm_expl/meeting_opt_res_recv", 10);
  swarm_traj_pub_ = nh.advertise<bspline::Bspline>("/planning/swarm_traj_recv", 100);
  heartbeat_pub_ = nh.advertise<plan_env::Heartbeat>("/communication/heartbeat_recv", 100);
  chunk_stamps_pub_ = nh.advertise<plan_env::ChunkStamps>("/multi_map_manager/chunk_stamps_recv", 10);
  chunk_data_pub_ = nh.advertise<plan_env::ChunkData>("/multi_map_manager/chunk_data_recv", 5000);

  boost::thread udp_recv_thd(udp_recv_fun);
  udp_recv_thd.detach();
  ros::Duration(0.1).sleep();

  // UDP connect
  udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);

  ROS_INFO("[swarm_bridge] Started, drone_id=%d, broadcast_ip=%s", drone_id_, udp_ip_.c_str());

  ros::spin();

  close(udp_server_fd_);
  close(udp_send_fd_);

  return 0;
}
