#include <active_perception/connectivity_graph.h>

namespace c2_expl {

void ConnectivityGraph::addNode(ConnectivityNode::Ptr node) {
  if (!node) return;
  // Replace existing node with same id to avoid stale pointer retention.
  nodes_[node->id_] = node;
}

void ConnectivityGraph::removeNode(const int& id) {
  auto it_node = nodes_.find(id);
  if (it_node == nodes_.end()) {
    return;
  }

  ConnectivityNode::Ptr node = it_node->second;
  if (!node) {
    nodes_.erase(it_node);
    return;
  }

  // Remove this node from its neighbors' lists
  for (auto it = node->neighbors_.begin(); it != node->neighbors_.end(); ++it) {
    auto nbr_it = nodes_.find(it->id2_);
    if (nbr_it != nodes_.end()) {
      nbr_it->second->removeNeighbor(id);
    }
  }

  nodes_.erase(id);
}

void ConnectivityGraph::clearNodes() { nodes_.clear(); }

ConnectivityNode::Ptr ConnectivityGraph::getNode(const int& id) {
  auto it = nodes_.find(id);
  if (it == nodes_.end()) {
    return nullptr;
  }
  return it->second;
}

void ConnectivityGraph::getNodeNum(int& num) { num = nodes_.size(); }

int ConnectivityGraph::getNodeNum() { return nodes_.size(); }

void ConnectivityGraph::getNodePositions(std::vector<Position>& node_positions) {
  node_positions.clear();
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    node_positions.push_back(it->second->pos_);
  }
}

void ConnectivityGraph::getNodePositionsWithIDs(std::vector<Position>& node_positions,
    std::vector<int>& node_ids) {
  node_positions.clear();
  node_ids.clear();
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    node_positions.push_back(it->second->pos_);
    node_ids.push_back(it->second->id_);
  }
}

double ConnectivityGraph::searchConnectivityGraphBFS(
    const int& id1, const int& id2, std::vector<int>& path) {
  path.clear();

  if (id1 == id2) {
    path.push_back(id1);
    return 0.0;
  }

  if (nodes_.find(id1) == nodes_.end() || nodes_.find(id2) == nodes_.end()) {
    ROS_ERROR("[ConnectivityGraph] id1 %d or id2 %d does not exist", id1, id2);
    return 1000.0;
  }

  std::unordered_map<int, bool> visited_flags;
  visited_flags.reserve(nodes_.size());
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    visited_flags.insert(std::make_pair(it->first, false));
  }

  std::queue<int> queue;
  std::unordered_map<int, int> parent_map;
  queue.push(id1);
  visited_flags[id1] = true;
  parent_map.insert(std::make_pair(id1, -1));

  bool found = false;
  while (!queue.empty()) {
    int id = queue.front();
    queue.pop();

    if (id == id2) {
      found = true;
      break;
    }

    auto node_it = nodes_.find(id);
    if (node_it == nodes_.end() || !node_it->second) {
      ROS_WARN_THROTTLE(
          1.0, "[ConnectivityGraph] Missing node %d while BFS traversal, skip", id);
      continue;
    }
    for (auto it = node_it->second->neighbors_.begin(); it != node_it->second->neighbors_.end();
         ++it) {
      if (it->cost_ > 499.0) {
        continue;
      }
      auto nbr_it = nodes_.find(it->id2_);
      if (nbr_it == nodes_.end() || !nbr_it->second) {
        ROS_WARN_THROTTLE(1.0,
            "[ConnectivityGraph] Skip dangling edge %d -> %d in BFS", id, it->id2_);
        continue;
      }
      auto v_it = visited_flags.find(it->id2_);
      if (v_it == visited_flags.end()) {
        continue;
      }
      if (v_it->second) continue;
      queue.push(it->id2_);
      v_it->second = true;
      parent_map[it->id2_] = id;
    }
  }

  if (!found) {
    return 1000.0;
  }

  int id = id2;
  while (id != -1) {
    path.push_back(id);
    auto it = parent_map.find(id);
    if (it == parent_map.end()) {
      ROS_WARN_THROTTLE(1.0, "[ConnectivityGraph] Parent missing for node %d in BFS path", id);
      path.clear();
      return 1000.0;
    }
    id = it->second;
  }
  std::reverse(path.begin(), path.end());

  double cost = 0.0;
  for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i) {
    auto node_it = nodes_.find(path[i]);
    if (node_it == nodes_.end() || !node_it->second) {
      ROS_WARN_THROTTLE(1.0, "[ConnectivityGraph] Missing node %d in BFS cost parse", path[i]);
      return 1000.0;
    }
    for (auto it = node_it->second->neighbors_.begin(); it != node_it->second->neighbors_.end();
         ++it) {
      if (it->id2_ == path[i + 1]) {
        cost += it->cost_;
        break;
      }
    }
  }

  if (cost < 1e-6) {
    ROS_ERROR("[ConnectivityGraph] Path cost is zero");
  }

  return cost;
}

void ConnectivityGraph::getFullConnectivityGraph(std::vector<std::pair<Position, Position>>& edges,
    std::vector<ConnectivityEdge::TYPE>& edges_types, std::vector<double>& edges_costs) {
  edges.clear();
  edges_types.clear();
  edges_costs.clear();

  std::unordered_map<int, bool> visited_flags;
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    visited_flags.insert(std::make_pair(it->first, false));
  }
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    if (!it->second) continue;
    for (auto it2 = it->second->neighbors_.begin(); it2 != it->second->neighbors_.end(); ++it2) {
      auto v_it = visited_flags.find(it2->id2_);
      if (v_it != visited_flags.end() && v_it->second) {
        continue;
      }
      if (it2->cost_ > 499.0) {
        continue;
      }

      auto nbr_it = nodes_.find(it2->id2_);
      if (nbr_it == nodes_.end() || !nbr_it->second) {
        ROS_WARN_THROTTLE(1.0, "[ConnectivityGraph] Skip dangling edge %d -> %d in full graph",
            it->first, it2->id2_);
        continue;
      }
      edges.push_back(std::make_pair(it->second->pos_, nbr_it->second->pos_));
      edges_types.push_back(it2->type_);
      edges_costs.push_back(it2->cost_);
    }
    visited_flags[it->first] = true;
  }

  if (edges.size() != edges_types.size() || edges.size() != edges_costs.size()) {
    ROS_ERROR("[ConnectivityGraph] edges size mismatch");
  }
}

void ConnectivityGraph::getFullConnectivityGraphPath(
    std::vector<std::vector<Position>>& paths, std::vector<ConnectivityEdge::TYPE>& paths_types) {
  paths.clear();
  paths_types.clear();

  std::unordered_map<int, bool> visited_flags;
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    visited_flags.insert(std::make_pair(it->first, false));
  }

  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    if (!it->second) continue;
    for (auto it2 = it->second->neighbors_.begin(); it2 != it->second->neighbors_.end(); ++it2) {
      auto v_it = visited_flags.find(it2->id2_);
      if (v_it != visited_flags.end() && v_it->second) {
        continue;
      }
      if (it2->cost_ > 499.0) {
        continue;
      }

      if (it2->path_.empty()) {
        Position pos1, pos2;
        pos1 = it->second->pos_;
        auto nbr_it = nodes_.find(it2->id2_);
        if (nbr_it == nodes_.end() || !nbr_it->second) {
          ROS_WARN_THROTTLE(
              1.0, "[ConnectivityGraph] Skip dangling edge %d -> %d in graph path", it->first,
              it2->id2_);
          continue;
        }
        pos2 = nbr_it->second->pos_;
        std::vector<Position> path;
        path.push_back(pos1);
        path.push_back(pos2);
        paths.push_back(path);
      } else {
        paths.push_back(it2->path_);
      }
      paths_types.push_back(it2->type_);
    }

    visited_flags[it->first] = true;
  }
}

void ConnectivityGraph::findDisconnectedNodes(std::set<int>& disconnected_nodes) {
  ros::Time t1 = ros::Time::now();

  disconnected_nodes.clear();
  std::unordered_map<int, int> labels;
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    labels.insert(std::make_pair(it->first, -1));
  }

  int label = 0;
  std::vector<int> ids;
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    if (labels[it->first] != -1) {
      continue;
    }
    std::stack<int> stack;
    stack.push(it->first);
    labels[it->first] = label;

    ids.clear();
    ids.push_back(it->first);

    bool is_all_unknown = true;
    while (!stack.empty()) {
      int id = stack.top();
      stack.pop();
      auto node_it = nodes_.find(id);
      if (node_it == nodes_.end() || !node_it->second) continue;
      for (auto it2 = node_it->second->neighbors_.begin(); it2 != node_it->second->neighbors_.end();
           ++it2) {
        if (labels[it2->id2_] != -1) {
          continue;
        }
        if (it2->cost_ > 499.0) {
          continue;
        }
        auto nbr_it = nodes_.find(it2->id2_);
        if (nbr_it == nodes_.end() || !nbr_it->second) {
          continue;
        }

        stack.push(it2->id2_);
        labels[it2->id2_] = label;
        ids.push_back(it2->id2_);

        if (nbr_it->second->type_ != ConnectivityNode::TYPE::UNKNOWN) {
          is_all_unknown = false;
        }
      }
    }

    label++;

    if (is_all_unknown) {
      for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
        disconnected_nodes.insert(ids[i]);
      }
    }
  }

  ros::Time t2 = ros::Time::now();
  (void)t1;
  (void)t2;
}



}  // namespace c2_expl
