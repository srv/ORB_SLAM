cmake_minimum_required(VERSION 2.8.3)
project(orb_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
  pcl_ros
  pcl_conversions
  g2o
  message_generation
)

ADD_DEFINITIONS("-std=c++11")

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#find_package(Cholmod REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap 1.6 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ORBState.msg
)

add_service_files(
  FILES
  SaveOctomap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs 
  DEPENDS message_runtime std_msgs 
)

message(*************************)
message(${catkin_INCLUDE_DIRS})
message(*************************)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 ")

add_subdirectory(Thirdparty/DBoW2)

add_executable(${PROJECT_NAME}
  src/main.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FramePublisher.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapPublisher.cc
  src/OctoMapPublisher.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/StatePublisher.cc
  src/Initializer.cc
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  ${OCTOMAP_LIBRARIES}
#  cholmod
  DBoW2
#  g2o_core
#  g2o_solver_cholmod
#  g2o_solver_dense
#  g2o_stuff
#  g2o_types_sba
#  g2o_types_sim3
#  g2o_types_slam3d
)
