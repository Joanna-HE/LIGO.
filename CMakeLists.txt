cmake_minimum_required(VERSION 3.10)
project(ligo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")

# Output files (Log/, PCD/, Data/) are resolved relative to the package source directory.
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  roscpp
  rospy
  pcl_ros
  pcl_conversions
  tf
  eigen_conversions
  message_generation
  livox_ros_driver
  gnss_comm
)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres 2.1 REQUIRED)
find_package(Sophus REQUIRED)

add_message_files(
  FILES
  LocalSensorExternalTrigger.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs std_msgs roscpp rospy message_runtime
                 livox_ros_driver gnss_comm
  DEPENDS EIGEN3 PCL
)

add_executable(ligo_mapping
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp
  src/GNSS_Processing_fg.cpp
  src/GNSS_Initialization.cpp
  src/GNSS_Assignment.cpp
  src/NMEA_Processing_fg.cpp
  src/NMEA_Assignment.cpp
  include/Urbannav_process/handler.cpp
)

target_include_directories(ligo_mapping PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

target_compile_definitions(ligo_mapping PRIVATE ${PCL_DEFINITIONS})

# Make sure the generated message headers (ligo/LocalSensorExternalTrigger.h) exist before compiling.
add_dependencies(ligo_mapping ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(ligo_mapping
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam
  Sophus::Sophus
  OpenMP::OpenMP_CXX
)

install(TARGETS ligo_mapping
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
