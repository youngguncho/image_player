cmake_minimum_required (VERSION 2.8.12)

#set (CMAKE_PREFIX_PATH /opt/Qt5.6.1/5.6/gcc_64/lib/cmake)

project (image_player)

add_definitions(-std=c++11)

# Find includes in corresponding build directories
# Adds in each directory ${CMAKE_CURRENT_SOURCE_DIR} and ${CMAKE_CURRENT_BINARY_DIR}
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
# Instruct CMake to run moc automatically when needed.


# Find the Qt libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
)

set(CMAKE_AUTOMOC ON)

find_package (Qt5Widgets)
find_package (Qt5Gui)
find_package (OpenCV REQUIRED)

set (SRC
  src/main.cpp
  src/mainwindow.cpp src/mainwindow.h
  src/rosthread.cpp src/rosthread.h
  src/cvqtUtils.cpp src/cvqtUtils.h
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge dynamic_reconfigure
    DEPENDS OpenCV
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${SRC_DIR}
)

# add UI file
qt5_wrap_ui (image_player_ui_moc ${SRC_DIR}/mainwindow.ui)

add_executable (image_player ${SRC} ${image_player_ui_moc})
target_link_libraries (image_player
    Qt5::Widgets
    Qt5::Gui
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
