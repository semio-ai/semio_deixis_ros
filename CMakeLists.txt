cmake_minimum_required(VERSION 2.8.3)
project(semio_deixis_ros)

# find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  semio_msgs_ros
  semio_humanoid_ros
)

# declare catkin package
catkin_package(
)

# add include directores of dependencies to header search path
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# build nodes
add_subdirectory(nodes)
