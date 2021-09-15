cmake_minimum_required(VERSION 3.0.2)
project(ompl_2d_rviz_visualizer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
  eigen_conversions
  roscpp
  rviz_visual_tools
  rviz
)

find_package(ompl REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt4_wrap_cpp(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    ${PROJECT_NAME}_nodelet
    ${PROJECT_NAME}_panel
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include 
  ${OMPL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt_wrap_cpp(MOC_FILES
  src/rviz_control_panel.h
)

# visualizer nodelet
add_library(${PROJECT_NAME}_nodelet
  src/visualizer_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

# control panel
add_library(${PROJECT_NAME}_panel
  src/rviz_control_panel.cpp
  ${MOC_FILES}
)
target_link_libraries(${PROJECT_NAME}_panel 
  ${rviz_DEFAULT_PLUGIN_LIBRARIES} 
  ${QT_LIBRARIES} 
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES 
  nodelet_plugins.xml
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)