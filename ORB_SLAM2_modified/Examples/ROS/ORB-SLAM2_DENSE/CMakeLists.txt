cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(octomap_DIR "/opt/ros/indigo/share/octomap")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")


# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)


find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
)
				
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS}/opt/ros/indigo/include
)

#### MY ####
include_directories(
"~/Project/orb-slam_dese/RGBD_ORB_SLAM2_RT/orb_slam_rgbd/g2o_with_orbslam2/"
"~/Project/orb-slam_dese/RGBD_ORB_SLAM2_RT/orb_slam_rgbd/g2o_with_orbslam2/build"
)
#### MY ####

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

#### MY ####
link_directories("~/Project/orb-slam_dese/RGBD_ORB_SLAM2_RT/orb_slam_rgbd/g2o_with_orbslam2/lib" )
#### MY ####


set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sim3 g2o_types_sba
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
${PCL_LIBRARIES}
${OCTOMAP_LIBRARIES}
)

# Node for monocular camera
rosbuild_add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

# Node for octomap
rosbuild_add_executable(orb_octomap
src/octomap.cc
)

target_link_libraries(orb_octomap
${LIBS}
${catkin_INCLUDE_DIRS}
)
# Node for visual_octomap
rosbuild_add_executable(visual_octomap
src/visual_octomap.cc
)

target_link_libraries(visual_octomap
${LIBS}
${catkin_INCLUDE_DIRS}
)
