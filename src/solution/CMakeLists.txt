cmake_minimum_required(VERSION 3.0.2)
project(solution)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(catkin REQUIRED COMPONENTS
geometry_msgs
nav_msgs
sensor_msgs
octomap_msgs
octomap_ros
roscpp
rospy
rosbag
std_msgs
tf
eigen_conversions
)
find_package(Eigen3)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)


# catkin_install_python(solution scripts/Crawling.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# 头文件路径
add_executable(to_three
  src/to_three.cpp
)
target_link_libraries(to_three
${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
add_executable(handle_pc
  src/handle_pc.cpp
)
target_link_libraries(handle_pc
${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
add_executable(update_map
  src/update_map.cpp
)
target_link_libraries(update_map
${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(roat
  src/roat.cpp
)
target_link_libraries(roat
${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
)

# add_executable(pub_test
#   src/pub_test.cpp
# )
# target_link_libraries(pub_test
# ${catkin_LIBRARIES}
# )

add_executable(odom_pub
  src/odom_pub.cpp
)
target_link_libraries(odom_pub
${catkin_LIBRARIES} ${EIGEN3_LIBRARIES}
)
add_executable(mk_cyc
  src/mk_cyc.cpp
)
target_link_libraries(mk_cyc
${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES}
)