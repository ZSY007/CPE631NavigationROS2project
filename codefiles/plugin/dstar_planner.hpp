#ifndef CPE631_ROS2__DSTAR_PLANNER_HPP_
#define CPE631_ROS2__DSTAR_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace cpe631_ros2
{

class DStarPlanner : public nav2_core::GlobalPlanner
{
public:
  DStarPlanner() = default;
  ~DStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  using NodeT = nav2_util::LifecycleNode;
  using NodeWeakPtr = rclcpp_lifecycle::LifecycleNode::WeakPtr;

  struct GridNode
  {
    unsigned int x;
    unsigned int y;
  };

  bool worldToMap(
    const geometry_msgs::msg::PoseStamped & pose,
    GridNode & node) const;

  bool findReachableGoal(
    const GridNode & requested_goal,
    GridNode & reachable_goal) const;

  bool isTraversable(unsigned int x, unsigned int y) const;
  double traversalCost(const GridNode & from, const GridNode & to) const;
  double lineTraversalCost(const GridNode & from, const GridNode & to) const;
  double heuristic(const GridNode & a, const GridNode & b) const;
  unsigned int toIndex(unsigned int x, unsigned int y) const;
  GridNode toNode(unsigned int index) const;
  std::vector<GridNode> neighbors(const GridNode & node) const;
  bool hasLineOfSight(const GridNode & from, const GridNode & to) const;
  std::vector<unsigned int> smoothPath(
    const std::vector<unsigned int> & path_indices) const;

  nav_msgs::msg::Path makePath(
    const std::vector<unsigned int> & path_indices,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  NodeWeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("DStarPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  bool allow_unknown_{true};
  bool use_final_approach_orientation_{true};
  bool theta_any_angle_{true};
  bool smooth_path_{true};
  double max_pose_spacing_{0.10};
  double tolerance_{0.75};
  double neutral_cost_{1.0};
  double cost_factor_{3.0};
  unsigned char lethal_cost_threshold_{253};
  unsigned char smoothing_cost_threshold_{253};
  int max_iterations_{2000000};
};

}  // namespace cpe631_ros2

#endif  // CPE631_ROS2__DSTAR_PLANNER_HPP_
