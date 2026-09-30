cmake_minimum_required(VERSION 3.8)
project(vio_slam_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 1. FIND ROS2 & STANDARD PACKAGES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# --- 2. FIND EPOXY, OPENGL & PANGOLIN ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(EPOXY REQUIRED epoxy)
find_package(OpenGL REQUIRED)
find_package(Pangolin REQUIRED) 

# --- 3. SET ROOT PATH FOR ORB_SLAM3 ---
set(ORB_SLAM3_ROOT $ENV{HOME}/vio_slam/ORB_SLAM3)

# --- 4. INCLUDE DIRECTORIES ---
include_directories(
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS} 
  ${EPOXY_INCLUDE_DIRS}
)

# --- 5. ADD EXECUTABLE ---
add_executable(vio_node src/vio_node.cpp)

# --- 6. LINK LIBRARIES ---
target_link_libraries(vio_node
  ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS}
  ${EPOXY_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${Pangolin_LIBRARIES} 
  -lboost_serialization
  -lcrypto
)

# --- 7. AMENT DEPENDENCIES ---
ament_target_dependencies(vio_node
  rclcpp
  geometry_msgs
  sensor_msgs
  OpenCV
)

# --- 8. INSTALL & PACKAGE ---
install(TARGETS
  vio_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()