#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include <full_coverage_path_planner/full_coverage_path_planner.hpp>
#include "full_coverage_path_planner/spiral_stc.h"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(FCPP, test_2d_01)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node2D =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("FCPP");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  node2D->declare_parameter("test.smooth_path", true);
  node2D->set_parameter(rclcpp::Parameter("test.smooth_path", true));
  node2D->declare_parameter("test.downsample_costmap", true);
  node2D->set_parameter(rclcpp::Parameter("test.downsample_costmap", true));
  node2D->declare_parameter("test.downsampling_factor", 2);
  node2D->set_parameter(rclcpp::Parameter("test.downsampling_factor", 2));

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal = start;
  auto planner_2d = std::make_unique<full_coverage_path_planner::SpiralSTC>();

  planner_2d->configure(node2D, "test", nullptr, costmap_ros);
  planner_2d->activate();
  try {
    planner_2d->createPlan(start, goal);
  } catch (...) {
  }

  planner_2d->deactivate();
  planner_2d->cleanup();

  planner_2d.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  node2D.reset();
  costmap_ros.reset();
}
