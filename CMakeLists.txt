cmake_minimum_required(VERSION 3.3.0)

message("Building ImFusion Robotic Vessel")
project(3d_livesegm_compounding)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

include(cmake/Macros.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ROS_DISTRO "noetic" CACHE STRING "ROS distribution name (e.g. noetic)")
set(ROS_WORKSPACE_ROOT "/opt/ros/${ROS_DISTRO}" CACHE STRING "Root of the ROS workspace to be sourced (e.g. /opt/ros/noetic)")

message(STATUS "> Configuring ${PROJECT_NAME} for ROS ${ROS_DISTRO}")
if (EXISTS ${ROS_WORKSPACE_ROOT})
    message(STATUS "> Sourcing ROS workspace at ${ROS_WORKSPACE_ROOT}")
else()
    message(FATAL_ERROR "Could not find a ROS workspace root at ${ROS_WORKSPACE_ROOT}")
endif()


list(APPEND CMAKE_PREFIX_PATH ${ROS_WORKSPACE_ROOT})

find_package(ImFusionLib COMPONENTS ImFusionRobotics ImFusionROS ImFusionUS ImFusionLiveUS CephasonicsPlugin REQUIRED)

find_package(ImFusionLib REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        iiwa_msgs
        iiwa_ros
        )
find_package(OpenIGTLink REQUIRED)

find_package(OpenCV 4 REQUIRED) # locate OpenCV in system #JZL
#include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "This is OpenCV INCLUDE DIRS" ${OpenCV_INCLUDE_DIRS})
message(STATUS "This is OpenCV LIBS DIRS" ${OpenCV_LIBS})

#find_package(Torch REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        iiwa_msgs
        iiwa_ros
        DEPENDS OpenIGTLink
        CFG_EXTRAS definitions.cmake
)


include("cmake/definitions.cmake")
file(GLOB PLUGIN_HEADERS "include/*.h" "include/3d_livesegm_compounding/*.h")

set(PLUGIN_SOURCES
        src/sweep_rec_and_comp.cpp
        src/plugin.cpp
        src/factory.cpp
        src/algorithm.cpp
        src/controller.cpp
        src/controller.ui
        src/CustomROSTopicTrackingStream.cpp
        )

add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCES} ${PLUGIN_FORMS} ${PLUGIN_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
        include
        ${IMFUSION_SDK_INCLUDE}
        ${catkin_INCLUDE_DIRS}
        ${OpenIGTLink_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${Qt5Widgets_INCLUDE_DIRS}
        )
link_directories(${CATKIN_DEVEL_PREFIX}/lib)
target_link_libraries(${PROJECT_NAME} PRIVATE
        ImFusionLib
        ImFusionStream
        ImFusionUS
        ImFusionLiveUS
        OpenIGTLink
        ImFusionROS
        ImFusionRobotics
        CephasonicsPlugin
        ${OpenCV_LIBS}
        "${TORCH_LIBRARIES}"
        ${catkin_LIBRARIES}
        Qt5::Widgets
        )

# Define output target directories
#imfusion_set_common_target_properties()

set(OldPythonExecutable ${PYTHON_EXECUTABLE})
set(PYTHON_EXECUTABLE /usr/bin/python2)
include(cmake/Macros.cmake)
imfusion_ros_add_catkin_dependencies(${PROJECT_NAME} COMPONENTS tf2_ros tf2_eigen cv_bridge)
set(PYTHON_EXECUTABLE ${OldPythonExecutable})

if (UNIX)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        target_link_libraries(${PROJECT_NAME} PRIVATE "-Wl,--allow-multiple-definition")
        #        target_link_options(${PROJECT_NAME} PRIVATE
        #                "-Wl,--disable-new-dtags"		# Always use rpath instead of runpath, since it is used recursively. See http://blog.qt.io/blog/2011/10/28/rpath-and-runpath for an explanation.
        #                )
    endif()

    # set the linker rpath so that libraries are found even if not install in system location (e.g. /usr)
    # see https://cmake.org/Wiki/CMake_RPATH_handling for more information
    set(RPATH "${RPATH}:$ORIGIN:${ROS_WORKSPACE_ROOT}/lib")
    if (NOT "${RPATH}" STREQUAL "")
        set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${RPATH}")
    endif()
endif()

###### INSTALLATION
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
        EXPORT "${PROJECT_NAME}Targets"
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
