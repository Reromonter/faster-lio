cmake_minimum_required(VERSION 3.5)
project(faster_lio)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif ()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)

# Upstream compiles Release with -g -ggdb. The Eigen/ikfom templates in
# laser_mapping.cc then emit ~150MB of debug info, and linking the resulting
# archive takes tens of minutes (much worse under qemu for the arm64 images).
# Off by default; turn on when you actually need to debug the estimator.
option(FASTER_LIO_DEBUG_SYMBOLS "Emit debug symbols in Release builds" OFF)
if (FASTER_LIO_DEBUG_SYMBOLS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -ggdb")
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb ${CMAKE_CXX_FLAGS}")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# options
option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)

if (WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else ()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters)
find_package(yaml-cpp REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)

set(ROS2_DEPS
        rclcpp
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        tf2_ros
        pcl_conversions
        livox_ros_driver
)

add_library(${PROJECT_NAME}
        src/laser_mapping.cc
        src/pointcloud_preprocess.cc
        src/options.cc
        src/utils.cc
)
ament_target_dependencies(${PROJECT_NAME} ${ROS2_DEPS})
target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        tbb
        glog
        yaml-cpp
)

add_executable(run_mapping_online app/run_mapping_online.cc)
target_link_libraries(run_mapping_online ${PROJECT_NAME} gflags)
ament_target_dependencies(run_mapping_online ${ROS2_DEPS})

add_executable(run_mapping_offline app/run_mapping_offline.cc)
target_link_libraries(run_mapping_offline ${PROJECT_NAME} gflags)
ament_target_dependencies(run_mapping_offline ${ROS2_DEPS} rosbag2_cpp)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS run_mapping_online run_mapping_offline
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
