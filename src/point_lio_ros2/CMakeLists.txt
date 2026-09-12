cmake_minimum_required(VERSION 3.5)
project(point_lio)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )

# Compiler settings for performance and threading
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_compile_options(-O3 -pthread -fexceptions)

# Processor count and definitions
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processor number:  ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message(STATUS "core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message(STATUS "core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(livox_ros_driver2 REQUIRED)

find_package(Eigen3 REQUIRED)


# OpenMP
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

# ─── LOCAL PATCH P10(见 docs/PATCHES.md)─────────────────────────────────
# 本机 ROS（Lyrical，ament_cmake 2.8.8）已经**删除** ament_target_dependencies 宏
# （上游 point_lio 写于还在用它的年代），CMake 4 也删了 FindPythonLibs（CMP0148）。
# 两处都在下面就地补齐，下游的依赖列表/语义不变。

# 1) ament_target_dependencies 的等价实现：
#    有 <dep>::<dep> imported target 就用它（rclcpp / tf2_ros / pcl_conversions…），
#    否则退回 <dep>_INCLUDE_DIRS / _LIBRARIES / _DEFINITIONS 老变量
#    （消息包 geometry_msgs / nav_msgs / sensor_msgs / visualization_msgs、
#     以及 pcl_ros 现在仍然导出这些变量）。
#    为什么不直接写成一堆 target_link_libraries：那样要逐包查目标名
#    （pcl_ros 根本没有 pcl_ros::pcl_ros 这个聚合目标），改一个包就得改一处，
#    而这个宏能跟上未来包导出方式的变化。
macro(ament_target_dependencies target)
  foreach(_dep ${ARGN})
    if(TARGET ${_dep}::${_dep})
      target_link_libraries(${target} ${_dep}::${_dep})
    else()
      if(DEFINED ${_dep}_INCLUDE_DIRS AND NOT "${${_dep}_INCLUDE_DIRS}" STREQUAL "")
        target_include_directories(${target} PUBLIC ${${_dep}_INCLUDE_DIRS})
      endif()
      if(DEFINED ${_dep}_LIBRARIES AND NOT "${${_dep}_LIBRARIES}" STREQUAL "")
        target_link_libraries(${target} ${${_dep}_LIBRARIES})
      endif()
      if(DEFINED ${_dep}_DEFINITIONS AND NOT "${${_dep}_DEFINITIONS}" STREQUAL "")
        target_compile_definitions(${target} PUBLIC ${${_dep}_DEFINITIONS})
      endif()
    endif()
  endforeach()
endmacro()

# 2) FindPythonLibs（CMake 4 已删）→ Python3；沿用老变量名，
#    下面的 include_directories/target_link_libraries 都不用改。
find_package(Python3 COMPONENTS Development REQUIRED)
set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${PYTHON_INCLUDE_DIRS}
)

# Declare a ROS2 executable
add_executable(pointlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/parameters.cpp src/preprocess.cpp src/Estimator.cpp)
ament_target_dependencies(pointlio_mapping
        rclcpp
        rclpy
        geometry_msgs
        nav_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        tf2_ros
        visualization_msgs
        # livox_ros_driver2
)
target_link_libraries(pointlio_mapping ${PYTHON_LIBRARIES})
target_include_directories(pointlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

# Install the executable
install(TARGETS
        pointlio_mapping
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY config launch rviz_cfg
        DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  visualization_msgs
  # livox_ros_driver2
  Eigen3
)

ament_package()
