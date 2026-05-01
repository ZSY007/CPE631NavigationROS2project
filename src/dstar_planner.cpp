#include "cpe631_ros2/dstar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cpe631_ros2
{
namespace
{

constexpr double INF = std::numeric_limits<double>::infinity();

struct QueueItem
{
  double k1;
  double k2;
  unsigned int index;
};

struct QueueCompare
{
  bool operator()(const QueueItem & a, const QueueItem & b) const
  {
    if (a.k1 == b.k1) {
      return a.k2 > b.k2;
    }
    return a.k1 > b.k1;
  }
};

bool lessKey(const std::pair<double, double> & a, const std::pair<double, double> & b)
{
  if (std::isinf(a.first) && std::isinf(b.first)) {
    return false;
  }
  if (a.first < b.first) {
    return true;
  }
  if (a.first > b.first) {
    return false;
  }
  return a.second < b.second;
}

bool sameKey(
  const std::pair<double, double> & a,
  const std::pair<double, double> & b)
{
  return std::abs(a.first - b.first) < 1e-9 &&
         std::abs(a.second - b.second) < 1e-9;
}

}  // namespace

void DStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node in DStarPlanner::configure");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_final_approach_orientation",
    rclcpp::ParameterValue(use_final_approach_orientation_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".theta_any_angle", rclcpp::ParameterValue(theta_any_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smooth_path", rclcpp::ParameterValue(smooth_path_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".neutral_cost", rclcpp::ParameterValue(neutral_cost_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_factor", rclcpp::ParameterValue(cost_factor_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lethal_cost_threshold",
    rclcpp::ParameterValue(static_cast<int>(lethal_cost_threshold_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoothing_cost_threshold",
    rclcpp::ParameterValue(static_cast<int>(smoothing_cost_threshold_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_pose_spacing", rclcpp::ParameterValue(max_pose_spacing_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(max_iterations_));

  node->get_parameter(name_ + ".tolerance", tolerance_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(
    name_ + ".use_final_approach_orientation", use_final_approach_orientation_);
  node->get_parameter(name_ + ".theta_any_angle", theta_any_angle_);
  node->get_parameter(name_ + ".smooth_path", smooth_path_);
  node->get_parameter(name_ + ".neutral_cost", neutral_cost_);
  node->get_parameter(name_ + ".cost_factor", cost_factor_);
  int lethal_threshold = static_cast<int>(lethal_cost_threshold_);
  node->get_parameter(name_ + ".lethal_cost_threshold", lethal_threshold);
  lethal_cost_threshold_ = static_cast<unsigned char>(
    std::clamp(lethal_threshold, 1, 253));
  int smoothing_threshold = static_cast<int>(smoothing_cost_threshold_);
  node->get_parameter(name_ + ".smoothing_cost_threshold", smoothing_threshold);
  smoothing_cost_threshold_ = static_cast<unsigned char>(
    std::clamp(smoothing_threshold, 1, 253));
  node->get_parameter(name_ + ".max_pose_spacing", max_pose_spacing_);
  max_pose_spacing_ = std::max(max_pose_spacing_, costmap_->getResolution());
  node->get_parameter(name_ + ".max_iterations", max_iterations_);

  RCLCPP_INFO(
    logger_,
    "Configured D* global planner '%s' on frame '%s' "
    "(tolerance=%.2f, allow_unknown=%s, theta_any_angle=%s, smooth_path=%s, "
    "max_pose_spacing=%.2f)",
    name_.c_str(), global_frame_.c_str(), tolerance_,
    allow_unknown_ ? "true" : "false", theta_any_angle_ ? "true" : "false",
    smooth_path_ ? "true" : "false", max_pose_spacing_);
}

void DStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up D* planner '%s'", name_.c_str());
}

void DStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating D* planner '%s'", name_.c_str());
}

void DStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating D* planner '%s'", name_.c_str());
}

nav_msgs::msg::Path DStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw std::runtime_error(
      "D* planner requires start and goal in the global costmap frame");
  }

  GridNode start_node;
  GridNode goal_node;
  if (!worldToMap(start, start_node)) {
    throw std::runtime_error("D* planner start pose is outside the global costmap");
  }
  if (!worldToMap(goal, goal_node)) {
    throw std::runtime_error("D* planner goal pose is outside the global costmap");
  }
  GridNode reachable_start = start_node;
  if (!isTraversable(start_node.x, start_node.y)) {
    if (!findReachableGoal(start_node, reachable_start)) {
      throw std::runtime_error("D* planner start cell is not traversable");
    }
  }

  GridNode reachable_goal = goal_node;
  if (!findReachableGoal(goal_node, reachable_goal)) {
    throw std::runtime_error("D* planner could not find a reachable goal cell");
  }

  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const unsigned int cell_count = size_x * size_y;
  const unsigned int start_index = toIndex(reachable_start.x, reachable_start.y);
  const unsigned int goal_index = toIndex(reachable_goal.x, reachable_goal.y);

  std::vector<double> g(cell_count, INF);
  std::vector<double> rhs(cell_count, INF);
  std::vector<unsigned int> theta_next(
    cell_count, std::numeric_limits<unsigned int>::max());
  std::vector<std::pair<double, double>> queue_key(
    cell_count, std::make_pair(INF, INF));
  std::priority_queue<QueueItem, std::vector<QueueItem>, QueueCompare> open;

  auto calc_key = [&](unsigned int index) {
      const double best = std::min(g[index], rhs[index]);
      return std::make_pair(best + heuristic(start_node, toNode(index)), best);
    };

  auto push = [&](unsigned int index, const std::pair<double, double> & key) {
      queue_key[index] = key;
      open.push(QueueItem{key.first, key.second, index});
    };

  auto pop_valid = [&]() -> std::pair<std::pair<double, double>, unsigned int> {
      while (!open.empty()) {
        const auto item = open.top();
        open.pop();
        const auto key = std::make_pair(item.k1, item.k2);
        if (sameKey(queue_key[item.index], key)) {
          queue_key[item.index] = std::make_pair(INF, INF);
          return {key, item.index};
        }
      }
      return {std::make_pair(INF, INF), std::numeric_limits<unsigned int>::max()};
    };

  auto top_key = [&]() {
      while (!open.empty()) {
        const auto item = open.top();
        const auto key = std::make_pair(item.k1, item.k2);
        if (sameKey(queue_key[item.index], key)) {
          return key;
        }
        open.pop();
      }
      return std::make_pair(INF, INF);
    };

  auto update_vertex = [&](unsigned int index) {
      if (index != goal_index) {
        const GridNode node = toNode(index);
        double best_rhs = INF;
        unsigned int best_next = std::numeric_limits<unsigned int>::max();
        for (const auto & succ : neighbors(node)) {
          const unsigned int succ_index = toIndex(succ.x, succ.y);
          const double adjacent = traversalCost(node, succ) + g[succ_index];
          if (adjacent < best_rhs) {
            best_rhs = adjacent;
            best_next = succ_index;
          }

          if (!theta_any_angle_) {
            continue;
          }

          const unsigned int succ_next = theta_next[succ_index];
          if (
            succ_next == std::numeric_limits<unsigned int>::max() ||
            succ_next == succ_index)
          {
            continue;
          }

          const GridNode next_node = toNode(succ_next);
          const double shortcut_cost = lineTraversalCost(node, next_node);
          const double shortcut = shortcut_cost + g[succ_next];
          if (shortcut < best_rhs) {
            best_rhs = shortcut;
            best_next = succ_next;
          }
        }
        rhs[index] = best_rhs;
        theta_next[index] = best_next;
      } else {
        theta_next[index] = index;
      }
      queue_key[index] = std::make_pair(INF, INF);
      if (std::abs(g[index] - rhs[index]) > 1e-9) {
        push(index, calc_key(index));
      }
    };

  rhs[goal_index] = 0.0;
  theta_next[goal_index] = goal_index;
  push(goal_index, calc_key(goal_index));

  int iterations = 0;
  while (
    lessKey(top_key(), calc_key(start_index)) ||
    std::abs(rhs[start_index] - g[start_index]) > 1e-9)
  {
    if (cancel_checker && cancel_checker()) {
      RCLCPP_WARN(logger_, "D* planning canceled");
      return nav_msgs::msg::Path();
    }
    if (++iterations > max_iterations_) {
      throw std::runtime_error("D* planner exceeded max_iterations");
    }

    const auto [old_key, index] = pop_valid();
    if (index == std::numeric_limits<unsigned int>::max()) {
      break;
    }

    const auto new_key = calc_key(index);
    if (lessKey(old_key, new_key)) {
      push(index, new_key);
      continue;
    }

    const GridNode node = toNode(index);
    if (g[index] > rhs[index]) {
      g[index] = rhs[index];
      for (const auto & pred : neighbors(node)) {
        update_vertex(toIndex(pred.x, pred.y));
      }
    } else {
      g[index] = INF;
      update_vertex(index);
      for (const auto & pred : neighbors(node)) {
        update_vertex(toIndex(pred.x, pred.y));
      }
    }
  }

  if (std::isinf(g[start_index]) && std::isinf(rhs[start_index])) {
    throw std::runtime_error("D* planner failed to find a path");
  }

  std::vector<unsigned int> path_indices;
  path_indices.reserve(1024);
  path_indices.push_back(start_index);
  unsigned int current_index = start_index;
  std::vector<bool> visited(cell_count, false);
  visited[current_index] = true;

  const unsigned int max_path_len = cell_count;
  for (unsigned int i = 0; i < max_path_len && current_index != goal_index; ++i) {
    const GridNode current = toNode(current_index);
    double best_value = INF;
    double best_h = INF;
    unsigned int best_index = theta_any_angle_ ? theta_next[current_index] : current_index;
    if (
      best_index != std::numeric_limits<unsigned int>::max() &&
      best_index != current_index)
    {
      const GridNode best_node = toNode(best_index);
      best_value = lineTraversalCost(current, best_node) + g[best_index];
      best_h = heuristic(best_node, reachable_goal);
      if (std::isinf(best_value)) {
        best_index = current_index;
        best_h = INF;
      }
    } else {
      best_index = current_index;
    }

    for (const auto & succ : neighbors(current)) {
      const unsigned int succ_index = toIndex(succ.x, succ.y);
      const double value = traversalCost(current, succ) + g[succ_index];
      const double h = heuristic(succ, reachable_goal);
      if (value < best_value || (value == best_value && h < best_h)) {
        best_value = value;
        best_h = h;
        best_index = succ_index;
      }
    }

    if (best_index == current_index || visited[best_index] || std::isinf(best_value)) {
      throw std::runtime_error("D* planner could not extract a valid path");
    }
    current_index = best_index;
    visited[current_index] = true;
    path_indices.push_back(current_index);
  }

  if (path_indices.back() != goal_index) {
    throw std::runtime_error("D* planner path extraction exceeded map size");
  }

  const auto output_indices = smooth_path_ ? smoothPath(path_indices) : path_indices;
  RCLCPP_DEBUG(
    logger_, "D* planner path cells: raw=%zu smoothed=%zu",
    path_indices.size(), output_indices.size());
  return makePath(output_indices, start, goal);
}

bool DStarPlanner::worldToMap(
  const geometry_msgs::msg::PoseStamped & pose,
  GridNode & node) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    return false;
  }
  node = GridNode{mx, my};
  return true;
}

bool DStarPlanner::findReachableGoal(
  const GridNode & requested_goal,
  GridNode & reachable_goal) const
{
  if (isTraversable(requested_goal.x, requested_goal.y)) {
    reachable_goal = requested_goal;
    return true;
  }

  const int radius = std::max(1, static_cast<int>(
    std::ceil(tolerance_ / costmap_->getResolution())));
  double best_dist = INF;
  bool found = false;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int nx = static_cast<int>(requested_goal.x) + dx;
      const int ny = static_cast<int>(requested_goal.y) + dy;
      if (nx < 0 || ny < 0) {
        continue;
      }
      const auto ux = static_cast<unsigned int>(nx);
      const auto uy = static_cast<unsigned int>(ny);
      if (ux >= costmap_->getSizeInCellsX() || uy >= costmap_->getSizeInCellsY()) {
        continue;
      }
      if (!isTraversable(ux, uy)) {
        continue;
      }
      const double dist = std::hypot(dx, dy);
      if (dist < best_dist) {
        best_dist = dist;
        reachable_goal = GridNode{ux, uy};
        found = true;
      }
    }
  }
  return found;
}

bool DStarPlanner::isTraversable(unsigned int x, unsigned int y) const
{
  const unsigned char cost = costmap_->getCost(x, y);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < lethal_cost_threshold_;
}

double DStarPlanner::traversalCost(const GridNode & from, const GridNode & to) const
{
  if (!isTraversable(from.x, from.y) || !isTraversable(to.x, to.y)) {
    return INF;
  }
  const unsigned char raw_cost = costmap_->getCost(to.x, to.y);
  const double map_cost =
    raw_cost == nav2_costmap_2d::NO_INFORMATION ? 0.0 : static_cast<double>(raw_cost);
  const double distance = (from.x != to.x && from.y != to.y) ? std::sqrt(2.0) : 1.0;
  return distance * (neutral_cost_ + cost_factor_ * map_cost / 252.0);
}

double DStarPlanner::lineTraversalCost(const GridNode & from, const GridNode & to) const
{
  if (from.x == to.x && from.y == to.y) {
    return 0.0;
  }
  if (!hasLineOfSight(from, to)) {
    return INF;
  }

  int x0 = static_cast<int>(from.x);
  int y0 = static_cast<int>(from.y);
  const int x1 = static_cast<int>(to.x);
  const int y1 = static_cast<int>(to.y);

  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;

  double weight_sum = 0.0;
  int samples = 0;
  bool skip_start = true;

  while (true) {
    if (skip_start) {
      skip_start = false;
    } else {
      const unsigned char raw_cost = costmap_->getCost(
        static_cast<unsigned int>(x0), static_cast<unsigned int>(y0));
      const double map_cost =
        raw_cost == nav2_costmap_2d::NO_INFORMATION ? 0.0 : static_cast<double>(raw_cost);
      weight_sum += neutral_cost_ + cost_factor_ * map_cost / 252.0;
      ++samples;
    }
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  const double distance = std::hypot(
    static_cast<double>(from.x) - static_cast<double>(to.x),
    static_cast<double>(from.y) - static_cast<double>(to.y));
  const double average_weight = samples > 0 ? weight_sum / samples : neutral_cost_;
  return distance * average_weight;
}

double DStarPlanner::heuristic(const GridNode & a, const GridNode & b) const
{
  const double dx = std::abs(static_cast<double>(a.x) - static_cast<double>(b.x));
  const double dy = std::abs(static_cast<double>(a.y) - static_cast<double>(b.y));
  return neutral_cost_ * std::hypot(dx, dy);
}

unsigned int DStarPlanner::toIndex(unsigned int x, unsigned int y) const
{
  return y * costmap_->getSizeInCellsX() + x;
}

DStarPlanner::GridNode DStarPlanner::toNode(unsigned int index) const
{
  const unsigned int size_x = costmap_->getSizeInCellsX();
  return GridNode{index % size_x, index / size_x};
}

std::vector<DStarPlanner::GridNode> DStarPlanner::neighbors(const GridNode & node) const
{
  std::vector<GridNode> out;
  out.reserve(8);
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const int nx = static_cast<int>(node.x) + dx;
      const int ny = static_cast<int>(node.y) + dy;
      if (nx < 0 || ny < 0) {
        continue;
      }
      const auto ux = static_cast<unsigned int>(nx);
      const auto uy = static_cast<unsigned int>(ny);
      if (ux >= costmap_->getSizeInCellsX() || uy >= costmap_->getSizeInCellsY()) {
        continue;
      }
      if (isTraversable(ux, uy)) {
        out.push_back(GridNode{ux, uy});
      }
    }
  }
  return out;
}

bool DStarPlanner::hasLineOfSight(const GridNode & from, const GridNode & to) const
{
  int x0 = static_cast<int>(from.x);
  int y0 = static_cast<int>(from.y);
  const int x1 = static_cast<int>(to.x);
  const int y1 = static_cast<int>(to.y);

  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;

  while (true) {
    if (x0 < 0 || y0 < 0) {
      return false;
    }
    const auto ux = static_cast<unsigned int>(x0);
    const auto uy = static_cast<unsigned int>(y0);
    if (ux >= costmap_->getSizeInCellsX() || uy >= costmap_->getSizeInCellsY()) {
      return false;
    }
    if (!isTraversable(ux, uy)) {
      return false;
    }
    const unsigned char cost = costmap_->getCost(ux, uy);
    if (cost != nav2_costmap_2d::NO_INFORMATION && cost > smoothing_cost_threshold_) {
      return false;
    }
    if (x0 == x1 && y0 == y1) {
      return true;
    }
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

std::vector<unsigned int> DStarPlanner::smoothPath(
  const std::vector<unsigned int> & path_indices) const
{
  if (path_indices.size() <= 2) {
    return path_indices;
  }

  std::vector<unsigned int> smoothed;
  smoothed.reserve(path_indices.size());
  smoothed.push_back(path_indices.front());

  std::size_t anchor = 0;
  while (anchor + 1 < path_indices.size()) {
    std::size_t next = anchor + 1;
    const GridNode anchor_node = toNode(path_indices[anchor]);
    for (std::size_t candidate = path_indices.size() - 1; candidate > anchor; --candidate) {
      if (hasLineOfSight(anchor_node, toNode(path_indices[candidate]))) {
        next = candidate;
        break;
      }
    }
    smoothed.push_back(path_indices[next]);
    anchor = next;
  }

  return smoothed;
}

nav_msgs::msg::Path DStarPlanner::makePath(
  const std::vector<unsigned int> & path_indices,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;
  path.poses.reserve(path_indices.size() * 4);

  auto make_pose_at = [&](double x, double y) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
  };

  auto point_at = [&](std::size_t i) {
    if (i == 0) {
      return std::make_pair(start.pose.position.x, start.pose.position.y);
    }
    if (i + 1 == path_indices.size()) {
      return std::make_pair(goal.pose.position.x, goal.pose.position.y);
    }
    const GridNode node = toNode(path_indices[i]);
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(node.x, node.y, wx, wy);
    return std::make_pair(wx, wy);
  };

  if (path_indices.empty()) {
    return path;
  }

  auto [first_x, first_y] = point_at(0);
  auto first_pose = make_pose_at(first_x, first_y);
  first_pose.pose = start.pose;
  path.poses.push_back(first_pose);

  for (std::size_t i = 1; i < path_indices.size(); ++i) {
    const auto [prev_x, prev_y] = point_at(i - 1);
    const auto [next_x, next_y] = point_at(i);
    const double dx = next_x - prev_x;
    const double dy = next_y - prev_y;
    const double distance = std::hypot(dx, dy);
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / max_pose_spacing_)));

    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      auto pose = make_pose_at(prev_x + ratio * dx, prev_y + ratio * dy);
      if (i + 1 == path_indices.size() && step == steps) {
        pose.pose = goal.pose;
      }
      path.poses.push_back(pose);
    }
  }

  if (use_final_approach_orientation_ && path.poses.size() >= 2) {
    for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto & p = path.poses[i].pose.position;
      const auto & q = path.poses[i + 1].pose.position;
      const double yaw = std::atan2(q.y - p.y, q.x - p.x);
      path.poses[i].pose.orientation.z = std::sin(yaw * 0.5);
      path.poses[i].pose.orientation.w = std::cos(yaw * 0.5);
    }
  }

  RCLCPP_DEBUG(logger_, "D* planner produced %zu poses", path.poses.size());
  return path;
}

}  // namespace cpe631_ros2

PLUGINLIB_EXPORT_CLASS(cpe631_ros2::DStarPlanner, nav2_core::GlobalPlanner)
