cmake_minimum_required(VERSION 3.8)
project(mid360_bringup)

find_package(ament_cmake REQUIRED)

# 纯 launch / config / rviz / 脚本包,没有编译目标
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/cleanup.sh
  scripts/preflight.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()