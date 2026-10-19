cmake_minimum_required(VERSION 3.14)
project(slithers_ros2)

# ---------------------------------------------------------------------------
# C++ standard
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies
#   - slithers : the core SLITHERS library (Eigen comes with it)
#   - ROS 2    : rclcpp + message packages used by the node
# ---------------------------------------------------------------------------
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(slithers      REQUIRED)

# ---------------------------------------------------------------------------
# Executable: slithers_node
# ---------------------------------------------------------------------------
add_executable(slithers_node
  src/slithers_node.cpp
)

ament_target_dependencies(slithers_node
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(slithers_node
  slithers::slithers
)

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------
# Install the node executable into lib/<project> so ros2 run can find it.
install(TARGETS slithers_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files so ros2 launch can find them.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
ament_package()
