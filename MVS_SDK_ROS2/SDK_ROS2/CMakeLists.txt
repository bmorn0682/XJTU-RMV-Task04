cmake_minimum_required(VERSION 3.8)
project(sdk_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(sdk_ros2_node src/sdk_ros2_node.cpp)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(sensor_msgs REQUIRED)


#ensure the include and lib path of MVS SDK is correct
set(MVS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(MVS_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/64)
set(LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/launch)
set(RVIZ_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
link_directories(${MVS_LIB_DIR})

find_library(MVS_LIB MvCameraControl PATHS ${MVS_LIB_DIR} NO_DEFAULT_PATH)

target_link_libraries(sdk_ros2_node
  ${MVS_LIB}
  pthread
)

target_include_directories(sdk_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MVS_INCLUDE_DIR})

install(
  TARGETS sdk_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install MVS SDK libraries
install(DIRECTORY ${MVS_LIB_DIR}/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
  PATTERN "ThirdParty" EXCLUDE)

# Install MVS SDK config and cti files
install(DIRECTORY ${MVS_LIB_DIR}/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.ini"
  PATTERN "*.cti"
  PATTERN "ThirdParty" EXCLUDE)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.launch.py"
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.rviz"
)

ament_target_dependencies(
  sdk_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
)

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
