cmake_minimum_required(VERSION 2.8.3)
project(camera_info_plugins)

find_package(catkin REQUIRED COMPONENTS
  rviz
  roscpp
  tf
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/camera_info_panel.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

