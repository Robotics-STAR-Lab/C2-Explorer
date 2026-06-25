#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>
#include <plan_env/communication_graph.h>

#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <fstream>
#include <limits>

namespace c2_expl {

MultiMapManager::MultiMapManager() {
}

MultiMapManager::~MultiMapManager() {
}

void MultiMapManager::setMap(SDFMap* map) {
  this->map_ = map;
}

void MultiMapManager::setCommunicationGraph(shared_ptr<CommunicationGraph> comm_graph) {
  this->comm_graph_ = comm_graph;
}

void MultiMapManager::init() {
  nh_.param("exploration/drone_id", drone_id_, 1);
  nh_.param("exploration/vis_drone_id", vis_drone_id_, -1);
  nh_.param("exploration/drone_num", map_num_, 2);

  nh_.param("multi_map_manager/chunk_size", chunk_size_, 200);
  nh_.param("multi_map_manager/chunk_partial_flush_interval", chunk_partial_flush_interval_, 0.5);
  if (chunk_size_ <= 0) {
    ROS_WARN("[MultiMapManager] Invalid chunk_size=%d, fallback to 200", chunk_size_);
    chunk_size_ = 200;
  }
  if (chunk_partial_flush_interval_ < 0.0) {
    chunk_partial_flush_interval_ = 0.0;
  }
  last_adr_buffer_update_time_ = ros::Time::now().toSec();

  stamp_timer_ = nh_.createTimer(ros::Duration(0.1), &MultiMapManager::stampTimerCallback, this);
  chunk_timer_ = nh_.createTimer(ros::Duration(0.1), &MultiMapManager::chunkTimerCallback, this);

  stamp_pub_ = nh_.advertise<plan_env::ChunkStamps>("/multi_map_manager/chunk_stamps_send", 10);
  chunk_pub_ = nh_.advertise<plan_env::ChunkData>("/multi_map_manager/chunk_data_send", 5000);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "/multi_map_manager/marker_" + std::to_string(drone_id_), 10);

  stamp_sub_ = nh_.subscribe(
      "/multi_map_manager/chunk_stamps_recv", 10, &MultiMapManager::stampMsgCallback, this);
  chunk_sub_ = nh_.subscribe("/multi_map_manager/chunk_data_recv", 5000,
      &MultiMapManager::chunkCallback, this, ros::TransportHints().tcpNoDelay());

  multi_map_chunks_.resize(map_num_);
  for (auto& data : multi_map_chunks_) {
    data.idx_list_ = {};
  }
  chunk_boxes_.resize(map_num_);
  for (auto& box : chunk_boxes_) {
    box.valid_ = false;
  }
  chunk_buffer_.resize(map_num_);
  buffer_map_.resize(map_num_);
  last_chunk_stamp_time_.assign(map_num_, 0.0);

  // // Test the idx list operation

  // // Find missed
  // vector<int> self1 = { 1, 1000 };
  // vector<int> self2 = { 5, 100, 105, 105, 120, 300, 400, 600, 700, 1000 };
  // vector<vector<int>> selfs = { self1, self2 };

  // vector<int> other1 = {};
  // vector<int> other2 = { 10, 80 };
  // vector<int> other3 = { 10, 80, 150, 200, 501, 550 };
  // vector<vector<int>> others = { other1, other2, other3 };

  // for (auto sf : selfs) {
  //   for (auto ot : others) {
  //     vector<int> missed;
  //     findMissedChunkIds(sf, ot, missed);
  //     std::cout << "Missed result: ";
  //     for (auto id : missed) std::cout << id << ", ";
  //     std::cout << "" << std::endl;
  //   }
  // }

  // // Merge chunk ids
  // // result: 1, 1000,
  // // result: 1, 9, 81, 1000,
  // // result: 1, 9, 81, 149, 201, 500, 551, 1000,
  // std::cout << "Compute merged ids" << std::endl;
  // vector<int> input1 = { 1, 1000 };
  // vector<int> input2 = { 1, 9, 81, 1000 };
  // vector<int> input3 = { 1, 7, 85, 140, 250, 450, 560, 900 };
  // vector<vector<int>> inputs = { input1, input2, input3 };
  // for (auto ot : others) {
  //   for (auto ip : inputs) {
  //     vector<int> output;
  //     mergeChunkIds(ot, ip, output);
  //     std::cout << "Merged result: ";
  //     for (auto id : output) std::cout << id << ", ";
  //     std::cout << "" << std::endl;
  //   }
  // }
}

void MultiMapManager::flushAdrBuffer(bool allow_partial) {
  if (drone_id_ < 1 || drone_id_ > map_num_) return;
  auto& own_chunks = multi_map_chunks_[drone_id_ - 1];

  const auto append_chunk = [&](const size_t begin, const size_t end) {
    MapChunk chunk;
    chunk.voxel_adrs_.insert(
        chunk.voxel_adrs_.end(), adr_buffer_.begin() + begin, adr_buffer_.begin() + end);
    chunk.idx_ = own_chunks.chunks_.size() + 1;
    chunk.need_query_ = true;
    chunk.empty_ = false;
    own_chunks.chunks_.push_back(std::move(chunk));
  };

  size_t processed = 0;
  while (processed + static_cast<size_t>(chunk_size_) <= adr_buffer_.size()) {
    append_chunk(processed, processed + static_cast<size_t>(chunk_size_));
    processed += static_cast<size_t>(chunk_size_);
  }

  if (allow_partial && processed < adr_buffer_.size()) {
    append_chunk(processed, adr_buffer_.size());
    processed = adr_buffer_.size();
  }

  if (processed > 0) {
    if (own_chunks.idx_list_.empty()) {
      own_chunks.idx_list_ = { 1, static_cast<int>(own_chunks.chunks_.back().idx_) };
    } else {
      own_chunks.idx_list_.back() = static_cast<int>(own_chunks.chunks_.back().idx_);
    }
    adr_buffer_.erase(adr_buffer_.begin(), adr_buffer_.begin() + processed);
  }
}

void MultiMapManager::updateMapChunk(const vector<uint32_t>& adrs) {
  if (drone_id_ < 1 || drone_id_ > map_num_) {
    ROS_WARN_THROTTLE(1.0, "[MultiMapManager] Invalid drone_id=%d (drone_num=%d), skip chunk update",
        drone_id_, map_num_);
    return;
  }
  if (adrs.empty()) return;

  adr_buffer_.insert(adr_buffer_.end(), adrs.begin(), adrs.end());
  last_adr_buffer_update_time_ = ros::Time::now().toSec();
  flushAdrBuffer(false);
}

bool MultiMapManager::canTransmitChunkTo(const int& other_drone_id) const {
  if (other_drone_id < 1 || other_drone_id > map_num_) return false;
  if (other_drone_id == drone_id_) return true;
  if (!comm_graph_) return true;

  vector<int> neighbor_ids;
  comm_graph_->getNeighborIds(neighbor_ids);
  return std::find(neighbor_ids.begin(), neighbor_ids.end(), other_drone_id) !=
         neighbor_ids.end();
}

void MultiMapManager::stampTimerCallback(const ros::TimerEvent& e) {
  const double now = ros::Time::now().toSec();
  if (!adr_buffer_.empty() &&
      now - last_adr_buffer_update_time_ >= chunk_partial_flush_interval_) {
    // Flush tail addresses to avoid permanent lag when update rate is low.
    flushAdrBuffer(true);
  }

  // Send stamp of chunks to other drones
  plan_env::ChunkStamps msg;
  msg.from_drone_id = drone_id_;
  msg.time = now;

  msg.idx_lists.reserve(multi_map_chunks_.size());
  for (const auto& chunks : multi_map_chunks_) {
    plan_env::IdxList idx_list;
    idx_list.ids = chunks.idx_list_;
    msg.idx_lists.push_back(idx_list);
  }
  stamp_pub_.publish(msg);
}

void MultiMapManager::stampMsgCallback(const plan_env::ChunkStampsConstPtr& msg) {
  if (msg->from_drone_id < 1 || msg->from_drone_id > map_num_) {
    ROS_WARN_THROTTLE(1.0, "[MultiMapManager] Ignore stamp from invalid drone_id=%d (drone_num=%d)",
        msg->from_drone_id, map_num_);
    return;
  }
  if (msg->from_drone_id == drone_id_) return;
  if (!canTransmitChunkTo(msg->from_drone_id)) return;
  // NOTE: Commented out ground node check - this was preventing drone with highest ID from sending
  // chunks If you have a dedicated ground node, you should use a separate flag instead of comparing
  // with map_num_ if (drone_id_ == map_num_) return;  // Ground node does not send chunk

  // auto t1 = ros::Time::now();

  // Throttle by local receive time to avoid cross-robot clock skew issues.
  const double now = ros::Time::now().toSec();
  if (now - last_chunk_stamp_time_[msg->from_drone_id - 1] < 2.0) return;
  last_chunk_stamp_time_[msg->from_drone_id - 1] = now;

  if (msg->idx_lists.size() < static_cast<size_t>(map_num_)) {
    ROS_WARN_THROTTLE(1.0,
        "[MultiMapManager] stamp idx_lists too short: from=%d got=%zu expect>=%d",
        msg->from_drone_id, msg->idx_lists.size(), map_num_);
  }

  // Check others' stamp info and send chunks unknown by them
  for (int i = 0; i < multi_map_chunks_.size(); ++i) {
    if (i == msg->from_drone_id - 1) continue;
    vector<int> missed;
    const vector<int> empty_ids;
    const vector<int>& peer_ids =
        (i < static_cast<int>(msg->idx_lists.size())) ? msg->idx_lists[i].ids : empty_ids;
    findMissedChunkIds(multi_map_chunks_[i].idx_list_, peer_ids, missed);
    sendChunks(i + 1, msg->from_drone_id, missed);
  }

  // ROS_ERROR("Stamp time: %lf", (ros::Time::now() - t1).toSec());
}

void MultiMapManager::findMissedChunkIds(
    const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids) {
  // Compute the complement set of other idx
  if (other_idx_list.empty()) {
    miss_ids = self_idx_list;
    return;
  }

  vector<int> not_in_other;
  if (other_idx_list[0] > 1) {
    not_in_other.push_back(1);
    not_in_other.push_back(other_idx_list[0] - 1);
  }
  for (int i = 1; i < other_idx_list.size(); i += 2) {
    not_in_other.push_back(other_idx_list[i] + 1);
    if (i == other_idx_list.size() - 1) {
      int infinite = std::numeric_limits<int>::max();
      not_in_other.push_back(infinite);
    } else {
      not_in_other.push_back(other_idx_list[i + 1] - 1);
    }
  }

  // Compute the intersection of self and not_in_other (brute-force, O(n^2))
  for (int i = 0; i < self_idx_list.size(); i += 2) {
    for (int j = 0; j < not_in_other.size(); j += 2) {
      int minr, maxr;
      if (findIntersect(self_idx_list[i], self_idx_list[i + 1], not_in_other[j],
              not_in_other[j + 1], minr, maxr)) {
        miss_ids.push_back(minr);
        miss_ids.push_back(maxr);
      }
    }
  }
}

bool MultiMapManager::findIntersect(
    const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr) {
  minr = max(min1, min2);
  maxr = min(max1, max2);
  if (minr <= maxr) return true;
  return false;
}

void MultiMapManager::sendChunks(
    const int& chunk_drone_id, const int& to_drone_id, const vector<int>& idx_list) {
  if (chunk_drone_id < 1 || chunk_drone_id > map_num_ || to_drone_id < 1 || to_drone_id > map_num_) {
    return;
  }
  if (!canTransmitChunkTo(to_drone_id)) return;
  if (idx_list.empty()) return;
  if (idx_list.size() % 2 != 0) {
    ROS_WARN_THROTTLE(1.0, "[MultiMapManager] Invalid idx_list length=%zu for sendChunks",
        idx_list.size());
    return;
  }

  auto& data = multi_map_chunks_[chunk_drone_id - 1];
  if (data.chunks_.empty()) return;

  for (int i = 0; i < idx_list.size(); i += 2) {
    int begin = std::max(1, idx_list[i]);
    int end = idx_list[i + 1];
    if (begin > end) continue;
    if (begin > static_cast<int>(data.chunks_.size())) continue;
    end = std::min(end, static_cast<int>(data.chunks_.size()));

    for (int j = begin; j <= end; ++j) {
      auto& chunk = data.chunks_[j - 1];
      if (chunk.empty_) continue;

      plan_env::ChunkData msg;
      msg.from_drone_id = drone_id_;
      msg.to_drone_id = to_drone_id;
      msg.chunk_drone_id = chunk_drone_id;
      msg.idx = chunk.idx_;
      msg.voxel_adrs = chunk.voxel_adrs_;
      if (chunk_drone_id == drone_id_ && chunk.need_query_) {
        // Should query the occ info in map if they are still empty
        getOccOfChunk(chunk.voxel_adrs_, chunk.voxel_occ_);
        chunk.need_query_ = false;
      }
      if (chunk.voxel_occ_.size() != chunk.voxel_adrs_.size()) {
        ROS_WARN_THROTTLE(1.0,
            "[MultiMapManager] Skip malformed chunk owner=%d idx=%u adrs=%zu occ=%zu",
            chunk_drone_id, chunk.idx_, chunk.voxel_adrs_.size(), chunk.voxel_occ_.size());
        continue;
      }
      msg.voxel_occ_ = chunk.voxel_occ_;

      chunk_pub_.publish(msg);
      // std::cout << "Drone " << drone_id_ << " send chunk " << msg.idx << " of drone "
      //           << int(msg.chunk_drone_id) << " to drone " << int(msg.to_drone_id) << std::endl;
    }
  }

  // for (int i = idx; i < data.chunks_.size(); ++i) {
  // }
}

void MultiMapManager::getOccOfChunk(const vector<uint32_t>& adrs, vector<uint8_t>& occs) {
  for (auto adr : adrs) {
    uint8_t occ = map_->md_->occupancy_buffer_[adr] > map_->mp_->min_occupancy_log_ ? 1 : 0;
    occs.push_back(occ);
  }
}

void MultiMapManager::chunkCallback(const plan_env::ChunkDataConstPtr& msg) {
  if (msg->from_drone_id < 1 || msg->from_drone_id > map_num_ || msg->to_drone_id < 1 ||
      msg->to_drone_id > map_num_ || msg->chunk_drone_id < 1 || msg->chunk_drone_id > map_num_) {
    ROS_WARN_THROTTLE(1.0,
        "[MultiMapManager] Ignore invalid chunk ids: from=%d to=%d chunk_owner=%d (drone_num=%d)",
        msg->from_drone_id, msg->to_drone_id, msg->chunk_drone_id, map_num_);
    return;
  }

  // Receive chunks from other drones, store them in chunk buffer
  if (msg->from_drone_id == drone_id_) return;
  // Unicast filter: only process chunks addressed to this drone
  if (msg->to_drone_id != drone_id_) return;
  if (!canTransmitChunkTo(msg->from_drone_id)) return;
  if (msg->idx == 0) return;
  if (msg->voxel_adrs.size() != msg->voxel_occ_.size()) {
    ROS_WARN_THROTTLE(1.0,
        "[MultiMapManager] Drop malformed chunk from=%d owner=%d idx=%u adrs=%zu occ=%zu",
        msg->from_drone_id, msg->chunk_drone_id, msg->idx, msg->voxel_adrs.size(),
        msg->voxel_occ_.size());
    return;
  }

  // Ignore chunks that are in the insertion buffer
  if (buffer_map_[msg->chunk_drone_id - 1].find(msg->idx) !=
      buffer_map_[msg->chunk_drone_id - 1].end())
    return;

  // ROS_ERROR("received msg idx: %d, from %d to %d", msg->idx, msg->from_drone_id,
  // msg->to_drone_id);
  chunk_buffer_[msg->chunk_drone_id - 1].push_back(*msg);
  buffer_map_[msg->chunk_drone_id - 1][msg->idx] = 1;

  return;
}

void MultiMapManager::chunkTimerCallback(const ros::TimerEvent& e) {

  // Not process chunk until swarm basecoor transform is available
  // if (!map_->getBaseCoor(1, tmp)) {
  //   ROS_WARN("basecoor not available yet.");
  //   return;
  // }

  // auto t1 = ros::Time::now();

  // Process chunks in buffers
  for (int i = 0; i < chunk_buffer_.size(); ++i) {
    auto& buffer = chunk_buffer_[i];
    if (buffer.empty()) continue;

    // Compute the idx list of buffered chunks
    sort(buffer.begin(), buffer.end(),
        [](const plan_env::ChunkData& chunk1, const plan_env::ChunkData& chunk2) {
          return chunk1.idx < chunk2.idx;
        });
    if (buffer.front().idx == 0) {
      buffer.clear();
      buffer_map_[i].clear();
      continue;
    }
    vector<int> idx_list = { int(buffer.front().idx) };
    int last_idx = idx_list[0];
    for (int j = 1; j < buffer.size(); ++j) {
      if (buffer[j].idx == 0) continue;
      if (buffer[j].idx - last_idx > 1) {
        idx_list.push_back(last_idx);
        idx_list.push_back(buffer[j].idx);
      }
      last_idx = buffer[j].idx;
    }
    if (last_idx <= 0) {
      buffer.clear();
      buffer_map_[i].clear();
      continue;
    }
    idx_list.push_back(last_idx);

    // std::cout << "process drone " << i + 1 << "'s chunks, input idx list: ";
    // for (auto id : idx_list) std::cout << id << ", ";
    // std::cout << "" << std::endl;

    // Update ChunksData's chunks_
    auto& chunks_data = multi_map_chunks_[i];

    // std::cout << "self idx list " << i + 1 << ": ";
    // for (auto id : chunks_data.idx_list_) std::cout << id << ", ";
    // std::cout << "" << std::endl;

    // Add placeholder for chunks
    const int len_inc = last_idx - static_cast<int>(chunks_data.chunks_.size());
    for (int j = 0; j < len_inc; ++j) {
      chunks_data.chunks_.push_back(MapChunk());
      auto& back_chunk = chunks_data.chunks_.back();
      back_chunk.idx_ = chunks_data.chunks_.size();
      back_chunk.empty_ = true;
      back_chunk.need_query_ = false;
    }

    // Process data in buffer
    for (const auto& msg : buffer) {
      if (msg.idx == 0 || msg.idx > chunks_data.chunks_.size()) continue;
      auto& chunk = chunks_data.chunks_[msg.idx - 1];
      if (chunk.empty_) {  // Only insert a chunk once
        if (msg.voxel_adrs.size() != msg.voxel_occ_.size()) continue;
        chunk.voxel_adrs_ = msg.voxel_adrs;
        chunk.voxel_occ_ = msg.voxel_occ_;
        insertChunkToMap(chunk, msg.chunk_drone_id);
        chunk.empty_ = false;
      }
    }

    // Update ChunksData's idx_list_
    vector<int> union_list;
    mergeChunkIds(idx_list, chunks_data.idx_list_, union_list);
    chunks_data.idx_list_ = union_list;

    // std::cout << "merged idx list " << i + 1 << ": ";
    // for (auto id : union_list) std::cout << id << ", ";
    // std::cout << "" << std::endl;

    buffer.clear();
    buffer_map_[i].clear();
  }
  // ROS_ERROR("chunk time: %lf", (ros::Time::now() - t1).toSec());
}

void MultiMapManager::mergeChunkIds(
    const vector<int>& list1, const vector<int>& list2, vector<int>& output) {

  // std::cout << "list1: ";
  // for (auto id : list1) std::cout << id << ", ";
  // std::cout << "" << std::endl;

  // std::cout << "list2: ";
  // for (auto id : list2) std::cout << id << ", ";
  // std::cout << "" << std::endl;

  if (list1.empty()) {
    output = list2;
    return;
  }

  output = list1;
  int tmp1, tmp2;
  for (int i = 0; i < list2.size(); i += 2) {
    // For each interval in list2, merge it into output list
    bool intersect = false;
    for (int j = 0; j < output.size(); j += 2) {
      if (findIntersect(output[j], output[j + 1], list2[i], list2[i + 1], tmp1, tmp2)) {
        output[j] = min(output[j], list2[i]);
        output[j + 1] = max(output[j + 1], list2[i + 1]);
        intersect = true;
      }
    }
    if (!intersect) {  // Insert the interval in appropriate position
      vector<int> tmp = { list2[i], list2[i + 1] };
      if (list2[i + 1] < output.front()) {
        output.insert(output.begin(), tmp.begin(), tmp.end());
      } else if (list2[i] > output.back()) {
        output.insert(output.end(), tmp.begin(), tmp.end());
      } else {
        for (auto iter = output.begin() + 1; iter != output.end(); iter += 2) {
          if (*iter < list2[i] && *(iter + 1) > list2[i + 1]) {
            output.insert(iter + 1, tmp.begin(), tmp.end());
            break;
          }
        }
      }
    }
    // Remove redundant idx
    for (auto iter = output.begin() + 1; iter != output.end() - 1;) {
      if (*iter >= *(iter + 1) - 1) {
        iter = output.erase(iter);
        iter = output.erase(iter);
      } else {
        iter += 2;
      }
    }
    // std::cout << "output: ";
    // for (auto id : output) std::cout << id << ", ";
    // std::cout << "" << std::endl;
  }
}

void MultiMapManager::adrToIndex(const uint32_t& adr, Eigen::Vector3i& idx) {
  // x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z
  uint32_t tmp_adr = adr;
  const int a = map_->mp_->map_voxel_num_[1] * map_->mp_->map_voxel_num_[2];
  const int b = map_->mp_->map_voxel_num_[2];

  idx[0] = tmp_adr / a;
  tmp_adr = tmp_adr % a;
  idx[1] = tmp_adr / b;
  idx[2] = tmp_adr % b;
}

void MultiMapManager::insertChunkToMap(const MapChunk& chunk, const int& drone_id) {
  if (drone_id < 1 || drone_id > map_num_) return;
  if (chunk.voxel_adrs_.size() != chunk.voxel_occ_.size()) {
    ROS_WARN_THROTTLE(1.0, "[MultiMapManager] Skip insert malformed chunk owner=%d idx=%u",
        drone_id, chunk.idx_);
    return;
  }

  // // Transform from other drone's local frame to this drone's
  // Eigen::Vector4d transform;
  // map_->getBaseCoor(drone_id, transform);

  // double yaw = transform[3];
  // Eigen::Matrix3d rot;
  // rot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  // Eigen::Vector3d trans = transform.head<3>();

  for (size_t i = 0; i < chunk.voxel_adrs_.size(); ++i) {
    // Insert occ info

    auto& adr = chunk.voxel_adrs_[i];

    Eigen::Vector3i idx;
    adrToIndex(adr, idx);

    Eigen::Vector3d pos;
    map_->indexToPos(idx, pos);

    // pos = rot * pos + trans;
    if (!map_->isInMap(pos)) continue;

    map_->posToIndex(pos, idx);
    auto adr_tf = map_->toAddress(idx);

    // map_->md_->occupancy_buffer_[adr] =
    //     chunk.voxel_occ_[i] == 1 ? map_->mp_->clamp_max_log_ : map_->mp_->clamp_min_log_;
    map_->md_->occupancy_buffer_[adr_tf] =
        chunk.voxel_occ_[i] == 1 ? map_->mp_->clamp_max_log_ : map_->mp_->clamp_min_log_;

    // Update the chunk box

    if (chunk_boxes_[drone_id - 1].valid_) {
      for (int k = 0; k < 3; ++k) {
        chunk_boxes_[drone_id - 1].min_[k] = min(chunk_boxes_[drone_id - 1].min_[k], pos[k]);
        chunk_boxes_[drone_id - 1].max_[k] = max(chunk_boxes_[drone_id - 1].max_[k], pos[k]);
      }
    } else {
      chunk_boxes_[drone_id - 1].min_ = chunk_boxes_[drone_id - 1].max_ = pos;
      chunk_boxes_[drone_id - 1].valid_ = true;
    }

    // Update the all box
    for (int k = 0; k < 3; ++k) {
      map_->md_->all_min_[k] = min(map_->md_->all_min_[k], pos[k]);
      map_->md_->all_max_[k] = max(map_->md_->all_max_[k], pos[k]);
    }
    // Inflate for the occupied
    if (chunk.voxel_occ_[i] == 1) {
      static const int inf_step = ceil(map_->mp_->obstacles_inflation_ / map_->mp_->resolution_);
      for (int inf_x = -inf_step; inf_x <= inf_step; ++inf_x)
        for (int inf_y = -inf_step; inf_y <= inf_step; ++inf_y)
          for (int inf_z = -inf_step; inf_z <= inf_step; ++inf_z) {
            Eigen::Vector3i inf_pt(idx[0] + inf_x, idx[1] + inf_y, idx[2] + inf_z);
            if (!map_->isInMap(inf_pt)) continue;
            int inf_adr = map_->toAddress(inf_pt);
            map_->md_->occupancy_buffer_inflate_[inf_adr] = 1;
          }
      // vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
      // map_->inflatePoint(idx, inf_step, inf_pts);

      // for (auto inf_pt : inf_pts) {
      //   if (!map_->isInMap(inf_pt)) continue;
      //   int idx_inf = map_->toAddress(inf_pt);
      //   if (idx_inf >= 0 &&
      //       idx_inf < map_->mp_->map_voxel_num_(0) * map_->mp_->map_voxel_num_(1) *
      //                     map_->mp_->map_voxel_num_(2)) {
      //     map_->md_->occupancy_buffer_inflate_[idx_inf] = 1;
      //   }
      // }
    }
  }
}

void MultiMapManager::getChunkBoxes(
    vector<Eigen::Vector3d>& mins, vector<Eigen::Vector3d>& maxs, bool reset) {
  for (auto& box : chunk_boxes_) {
    if (box.valid_) {
      mins.push_back(box.min_);
      maxs.push_back(box.max_);
      if (reset) box.valid_ = false;
    }
  }
}

// MultiMapManager::
}  // namespace c2_expl
