cmake_minimum_required(VERSION 3.8)
project(ros2_orb_slam3)

# Make sure to set this path before building the  
# set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/") # Must be set to match your installation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Must use C++17 to make it compatible with rclcpp
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if(NOT CMAKE_BUILD_TYPE)        # If no build type is set, set one
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build." FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Werror")
endif()

# if(COMPILER_SUPPORTS_CXX17)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
#   add_definitions(-DCOMPILEDWITHC17)
#   message(STATUS "Using flag -std=c++17.")
# else()
#   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
# endif()

# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules) # REDUNDANT?

set(project_path ${CMAKE_CURRENT_SOURCE_DIR})
set(ENV{PROJECT_PATH} "${project_path}")
add_definitions(-DPROJECT_PATH="${project_path}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(octomap_server REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED) # Matched with Sophus
find_package(Pangolin REQUIRED)
find_package(image_transport REQUIRED)


# Header file locations [C++ node]
include_directories(include) # Add .hpp, .h files from include/ros2_orb_slam3
include_directories(${EIGEN3_INCLUDE_DIRS}) # Include headers for eigen3
include_directories(${Pangolin_INCLUDE_DIRS}) # include headers for pangolin
include_directories(${OpenCV_INCLUDE_DIRS}) # include headers for pangolin


# ORB-SLAM3 includes [VSLAM library]
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/orb_slam3
  ${PROJECT_SOURCE_DIR}/orb_slam3/include
  ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
  ${ament_INCLUDE_DIRS}
)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  octomap_server
  nav_msgs
  apriltag
  apriltag_ros
  apriltag_msgs
  cv_bridge
  image_transport
  OpenCV
  Eigen3
  Pangolin
)

#* ORB SLAM3 as a shared library
# Whenever you add a new .h, .hpp or .cc, .cpp file in the ros2_orb_slam3/orb_slam3/include and os2_orb_slam3/orb_slam3/src directories, make sure to add them here as shown below
add_library(orb_slam3_lib SHARED
  orb_slam3/src/System.cc
  orb_slam3/src/Tracking.cc
  orb_slam3/src/LocalMapping.cc
  orb_slam3/src/LoopClosing.cc
  orb_slam3/src/ORBextractor.cc
  orb_slam3/src/ORBmatcher.cc
  orb_slam3/src/FrameDrawer.cc
  orb_slam3/src/Converter.cc
  orb_slam3/src/MapPoint.cc
  orb_slam3/src/KeyFrame.cc
  orb_slam3/src/Atlas.cc
  orb_slam3/src/Map.cc
  orb_slam3/src/MapDrawer.cc
  orb_slam3/src/Optimizer.cc
  orb_slam3/src/Frame.cc
  orb_slam3/src/KeyFrameDatabase.cc
  orb_slam3/src/Sim3Solver.cc
  orb_slam3/src/Viewer.cc
  orb_slam3/src/ImuTypes.cc
  orb_slam3/src/G2oTypes.cc
  orb_slam3/src/CameraModels/Pinhole.cpp
  orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3/src/OptimizableTypes.cpp
  orb_slam3/src/MLPnPsolver.cpp
  orb_slam3/src/GeometricTools.cc
  orb_slam3/src/TwoViewReconstruction.cc
  orb_slam3/src/Config.cc
  orb_slam3/src/Settings.cc
  orb_slam3/include/System.h
  orb_slam3/include/Tracking.h
  orb_slam3/include/LocalMapping.h
  orb_slam3/include/LoopClosing.h
  orb_slam3/include/ORBextractor.h
  orb_slam3/include/ORBmatcher.h
  orb_slam3/include/FrameDrawer.h
  orb_slam3/include/Converter.h
  orb_slam3/include/MapPoint.h
  orb_slam3/include/KeyFrame.h
  orb_slam3/include/Atlas.h
  orb_slam3/include/Map.h
  orb_slam3/include/MapDrawer.h
  orb_slam3/include/Optimizer.h
  orb_slam3/include/Frame.h
  orb_slam3/include/KeyFrameDatabase.h
  orb_slam3/include/Sim3Solver.h
  orb_slam3/include/Viewer.h
  orb_slam3/include/ImuTypes.h
  orb_slam3/include/G2oTypes.h
  orb_slam3/include/CameraModels/GeometricCamera.h
  orb_slam3/include/CameraModels/Pinhole.h
  orb_slam3/include/CameraModels/KannalaBrandt8.h
  orb_slam3/include/OptimizableTypes.h
  orb_slam3/include/MLPnPsolver.h
  orb_slam3/include/GeometricTools.h
  orb_slam3/include/TwoViewReconstruction.h
  orb_slam3/include/SerializationUtils.h
  orb_slam3/include/Config.h
  orb_slam3/include/Settings.h
)

set_target_properties(orb_slam3_lib PROPERTIES VERSION "${orb_slam3_lib_VERSION}") # TODO need to findout why this is required

ament_target_dependencies(orb_slam3_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Link libraries
target_link_libraries(orb_slam3_lib
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto 
)


# Find the .so files provided in ros2_orb_slam3/orb_slam3/Thirdparty projects
set(DBoW2_PATH "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so")
set(g2o_PATH "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so")

# Install .so files to the lib directory in the install space
install(FILES ${DBoW2_PATH} DESTINATION lib)
install(FILES ${g2o_PATH} DESTINATION lib)


# Add executable
add_executable(example_mono_node_cpp
  src/mono_example.cpp
  src/common.cpp
)

add_executable(mono_node_cpp
  src/mono_realsense.cpp
)

add_executable(imu_mono_node_cpp
  src/imu_mono_realsense.cpp
)

add_executable(capture_video
    src/capture_realsense_video.cpp
)

add_executable(visualize_point_cloud
    src/visualize_point_cloud.cpp
)

add_executable(apriltag_node
    src/apriltag_node.cpp
)

ament_target_dependencies(example_mono_node_cpp
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(mono_node_cpp
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(imu_mono_node_cpp
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(capture_video
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(visualize_point_cloud
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(apriltag_node
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(mono_node_cpp PUBLIC orb_slam3_lib) # Link a node with the internal shared library
target_link_libraries(imu_mono_node_cpp PUBLIC orb_slam3_lib) # Link a node with the internal shared library
target_link_libraries(example_mono_node_cpp PUBLIC orb_slam3_lib) # Link a node with the internal shared library
target_link_libraries(apriltag_node PUBLIC apriltag::apriltag)

# Install all the header files in package/package/include
install (DIRECTORY include/ 
        DESTINATION include
)

# install bag files
install(
  DIRECTORY bags/
  DESTINATION share/${PROJECT_NAME}/bags
)

# install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# install video files
install(
  DIRECTORY videos/
  DESTINATION share/${PROJECT_NAME}/videos
)

# install EuRoC dataset
install(
  DIRECTORY TEST_DATASET/
  DESTINATION share/${PROJECT_NAME}/TEST_DATASET)

# install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install our node and library
install(TARGETS apriltag_node visualize_point_cloud capture_video mono_node_cpp imu_mono_node_cpp example_mono_node_cpp orb_slam3_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME} ${CMAKE_INSTALL_INCLUDEDIR}
)

# Tell downstream packages where to find the headers
ament_export_include_directories(include)

# Help downstream packages to find transitive dependencies
ament_export_dependencies(
  orb_slam3_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Python node
# Install Python modules, submodules
ament_python_install_package(${PROJECT_NAME}) # Install the modules in ros2_orb_slam3/ros2_orb_slam3 folder

# Install Python executibles
install(PROGRAMS
  scripts/mono_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
