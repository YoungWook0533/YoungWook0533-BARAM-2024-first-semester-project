cmake_minimum_required(VERSION 3.8)
project(controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add executable
add_executable(joint_traj_action joint_traj_action/joint_traj_action.cpp)
add_executable(iiwa_ik joint_traj_action/iiwa_ik.cpp)
add_executable(iiwa_fk joint_traj_action/iiwa_fk.cpp)

# Include directoriess
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(joint_traj_action
  rclcpp
  rclcpp_action
  std_msgs
  trajectory_msgs
  control_msgs
)
ament_target_dependencies(iiwa_ik
  rclcpp
  std_msgs
  sensor_msgs
  Eigen3
)
ament_target_dependencies(iiwa_fk
  rclcpp
  std_msgs
  Eigen3
)

# Link Eigen library
target_link_libraries(iiwa_ik
  Eigen3::Eigen
)
target_link_libraries(iiwa_fk
  Eigen3::Eigen
)

# Install executable
install(TARGETS joint_traj_action
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS iiwa_ik
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS iiwa_fk
  DESTINATION lib/${PROJECT_NAME})    

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
