cmake_minimum_required(VERSION 3.5)
project(full_coverage_path_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

include_directories(
  include
  test/include
)

set(library_name full_coverage_path_planner)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs  
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
)

add_library(${library_name} SHARED
  src/full_coverage_path_planner.cpp
  src/navfn.cpp
  src/spiral_stc.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}-test test/src/test_spiral_stc.cpp test/src/test_common.cpp test/src/test_2d.cpp src/spiral_stc.cpp src/full_coverage_path_planner.cpp src/navfn.cpp src/common.cpp test/src/util.cpp )

#    catkin_add_gtest(test_spiral_stc test/src/test_spiral_stc.cpp test/src/util.cpp src/spiral_stc.cpp src/common.cpp src/${PROJECT_NAME}.cpp)
#    add_dependencies(test_spiral_stc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#    target_link_libraries(test_spiral_stc ${catkin_LIBRARIES})

    find_package(OpenCV)
    include_directories(${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(${PROJECT_NAME}-test rclcpp nav2_util rclcpp_lifecycle nav2_costmap_2d)
    target_link_libraries(${PROJECT_NAME}-test ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})


    #add_rostest(test/${PROJECT_NAME}/test_${PROJECT_NAME}.test)
    
    ament_add_test(system_test_full_coverage_path_planner
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/full_coverage_path_planner/test_full_coverage_path_planner_launch.py"
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test/full_coverage_path_planner"
      TIMEOUT 180
      ENV
        TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test
        TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map_circular.yaml
        TEST_WORLD=${PROJECT_SOURCE_DIR}/worlds/turtlebot3_ros2_demo.world
        GAZEBO_MODEL_PATH=${PROJECT_SOURCE_DIR}/models
        BT_NAVIGATOR_XML=navigate_w_replanning_and_recovery.xml
        ASTAR=False
    )

endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
