##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(application)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker

find_package(catkin REQUIRED COMPONENTS qt_build roscpp std_msgs geometry_msgs nav_msgs)
find_package(OpenCV REQUIRED)
include_directories(
    include
      ${OpenCV_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      )
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 

#add_message_files(
#    FILES
#    StartLocation.msg
#    TargetLocation.msg
#    MapGrid.msg
#)

#set(CMAKE_PREFIX_PATH "/opt/ros/melodic/share")


##############################################################################
# Qt Environment
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)
# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/application/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5PrintSupport_INCLUDE_DIRS}
    )

catkin_package()

add_definitions( -std=c++11 -fPIC)

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

set( QT_SOURCES
    src/main.cpp
    src/qnode.cpp
    src/guiapplication.cpp
    src/ShopType.cpp

    include/application/qnode.hpp
    include/application/guiapplication.hpp
    include/application/ShopType.hpp
    )

##############################################################################
# Binaries
##############################################################################￣

add_executable(application ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(
    application 
    ${QT_LIBRARIES} 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES application
#    CATKIN_DEPENDS
#    geometry_msgs
#    nav_msgs
    )


install(TARGETS application RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

