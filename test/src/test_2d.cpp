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

#if 0
TEST(FCPP, test_a_star_2d)
{
  smac_planner::SearchInfo info;
  smac_planner::AStarAlgorithm<smac_planner::Node2D> a_star(smac_planner::MotionModel::MOORE, info);
  int max_iterations = 10000;
  float tolerance = 0.0;
  float some_tolerance = 20.0;
  int it_on_approach = 10;
  int num_it = 0;

  a_star.initialize(false, max_iterations, it_on_approach);
  a_star.setFootprint(nav2_costmap_2d::Footprint(), true);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);
  // island in the middle of lethal cost to cross
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmapA->setCost(i, j, 254);
    }
  }

  // functional case testing
  a_star.createGraph(costmapA->getSizeInCellsX(), costmapA->getSizeInCellsY(), 1, costmapA);
  a_star.setStart(20u, 20u, 0);
  a_star.setGoal(80u, 80u, 0);
  smac_planner::Node2D::CoordinateVector path;
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance));
  EXPECT_EQ(num_it, 556);

  // check path is the right size and collision free
  EXPECT_EQ(path.size(), 81u);
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_EQ(costmapA->getCost(path[i].x, path[i].y), 0);
  }

  // setting non-zero dim 3 for 2D search
  EXPECT_THROW(
    a_star.createGraph(
      costmapA->getSizeInCellsX(), costmapA->getSizeInCellsY(), 10, costmapA), std::runtime_error);
  EXPECT_THROW(a_star.setGoal(0, 0, 10), std::runtime_error);
  EXPECT_THROW(a_star.setStart(0, 0, 10), std::runtime_error);

  path.clear();
  // failure cases with invalid inputs
  smac_planner::AStarAlgorithm<smac_planner::Node2D> a_star_2(
    smac_planner::MotionModel::VON_NEUMANN, info);
  a_star_2.initialize(false, max_iterations, it_on_approach);
  a_star_2.setFootprint(nav2_costmap_2d::Footprint(), true);
  num_it = 0;
  EXPECT_THROW(a_star_2.createPath(path, num_it, tolerance), std::runtime_error);
  a_star_2.createGraph(costmapA->getSizeInCellsX(), costmapA->getSizeInCellsY(), 1, costmapA);
  num_it = 0;
  EXPECT_THROW(a_star_2.createPath(path, num_it, tolerance), std::runtime_error);
  a_star_2.setStart(50, 50, 0);  // invalid
  a_star_2.setGoal(0, 0, 0);  // valid
  num_it = 0;
  EXPECT_THROW(a_star_2.createPath(path, num_it, tolerance), std::runtime_error);
  a_star_2.setStart(0, 0, 0);  // valid
  a_star_2.setGoal(50, 50, 0);  // invalid
  num_it = 0;
  EXPECT_THROW(a_star_2.createPath(path, num_it, tolerance), std::runtime_error);
  num_it = 0;
  // invalid goal but liberal tolerance
  a_star_2.setStart(20, 20, 0);  // valid
  a_star_2.setGoal(50, 50, 0);  // invalid
  EXPECT_TRUE(a_star_2.createPath(path, num_it, some_tolerance));
  EXPECT_EQ(path.size(), 32u);
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_EQ(costmapA->getCost(path[i].x, path[i].y), 0);
  }

  EXPECT_TRUE(a_star_2.getStart() != nullptr);
  EXPECT_TRUE(a_star_2.getGoal() != nullptr);
  EXPECT_EQ(a_star_2.getSizeX(), 100u);
  EXPECT_EQ(a_star_2.getSizeY(), 100u);
  EXPECT_EQ(a_star_2.getSizeDim3(), 1u);
  EXPECT_EQ(a_star_2.getToleranceHeuristic(), 1000.0);
  EXPECT_EQ(a_star_2.getOnApproachMaxIterations(), 10);

  delete costmapA;
}
#endif