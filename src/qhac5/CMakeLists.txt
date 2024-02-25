##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0072 NEW)

project(qhac5)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble/")
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

include(EnableSanitizers)

##############################################################################
# ROS2
##############################################################################
list(APPEND ROS_DISTROS "foxy" "humble")
if(DEFINED ENV{ROS2_DISTRO})
    set(ROS_DISTRO_ENV $ENV{ROS2_DISTRO})
    if (ROS_DISTRO_ENV IN_LIST ROS_DISTROS)
        add_definitions(-DROS_DEFAULT_API=1)
    endif()
else()
    set(ROS_DISTRO_ENV $ENV{ROS_DISTRO})
    if (ROS_DISTRO_ENV IN_LIST ROS_DISTROS)
        add_definitions(-DROS_DEFAULT_API=1)
    endif()
endif()


find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

find_package(px4_msgs REQUIRED)
#find_package(agent_msg REQUIRED)

find_package(PythonInterp 3 REQUIRED)
find_package(OpenCV REQUIRED)

#get_filename_component(PX4_MSGS_DIR "../" ABSOLUTE BASE_DIR ${px4_msgs_DIR})
#set(MSGS_DIR "${PX4_MSGS_DIR}/msg" CACHE INTERNAL "MSGS_DIR")
#include(GenerateMicroRTPSAgent)

include_directories(include ${rclcpp_INCLUDE_DIRS})

#function(custom_executable subfolder target)
#  add_executable(${target} src/${subfolder}/${target}.cpp)
#  ament_target_dependencies(${target}
#    "rclcpp"
#    "px4_msgs")
#  install(TARGETS ${target}
#  DESTINATION lib/${PROJECT_NAME})
#endfunction()

##############################################################################

#set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fpermissive")

#find_package(Boost REQUIRED COMPONENTS system)
include_directories(
        #    ${catkin_INCLUDE_DIRS}
        src/Main
        src/UI
        src/Agent
        src/Manager
        src/Control
        src/Comm
        src/Scenario
        src/Util
        src/Agent/Pixhawk
        src/Agent/Pixhawk/MModel
        src/Agent/Pixhawk/BModel
        src/Agent/Pixhawk/CModel
        /usr/include/eigen3
        ThirdPart/Mavlink_v2.0
        ${OpenCV_INCLUDE_DIRS}
)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Multimedia Network PrintSupport WebKit WebKitWidgets Positioning)
find_package(OpenGL)
find_package(GLUT)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)

#catkin_package()

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.qrc)
file(GLOB QT_OTHERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.html)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
set(QT_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Main/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/mainwindow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Scenario/scenario.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Scenario/scenarioevent.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Util/sleeper.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Util/logger.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/qcustomplot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Util/mymath.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Util/hungarian.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Manager/manager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/agent.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Scenario/cspline.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Comm/udpcomm.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Comm/comm.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/filemanager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/mavlinkdata.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/rosdata.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/paramcheckworker.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/spinworker.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/MModel/mmodelagent.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/MModel/mmodelcmdsender.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/MModel/mmodelcomm.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/BModel/bmodelagent.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/BModel/bmodelcmdsender.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/CModel/cmodelagent.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/CModel/cmodelcmdsender.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/paramdialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/calibdialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/monitoringdialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/emscenariodialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/controldialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/sendscdialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/qhac_mapview.cpp
)


file(GLOB_RECURSE QT_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.h)
set(QT_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Main/customconfig.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Scenario/scenario.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Scenario/scenarioevent.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Util/logger.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Util/sleeper.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/mainwindow.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/qcustomplot.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Util/mymath.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Util/hungarian.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Manager/manager.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/agent.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Scenario/cspline.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Comm/udpcomm.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Comm/comm.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/filemanager.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/mavlinkdata.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/rosdata.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/paramcheckworker.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/spinworker.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/MModel/mmodelagent.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/MModel/mmodelcmdsender.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/MModel/mmodelcomm.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/BModel/bmodelagent.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/BModel/bmodelcmdsender.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/CModel/cmodelagent.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Agent/Pixhawk/CModel/cmodelcmdsender.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/paramdialog.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/calibdialog.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/monitoringdialog.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/emscenariodialog.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/controldialog.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/sendscdialog.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UI/qhac_mapview.h
)

QT5_WRAP_CPP(QT_MOC_HPP ${QT_HEADERS})

##############################################################################
# Binaries
##############################################################################

add_executable(qhac5 ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_HEADERS} ${QT_MOC_HPP} ${QT_OTHERS})
qt5_use_modules(qhac5 Multimedia Core Gui Widgets Network PrintSupport WebKit WebKitWidgets Positioning)
target_link_libraries(qhac5 ${QT_LIBRARIES} ${OPENGL_gl_LIBRARY} ${GLUT_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS qhac5
        DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(qhac5
        "rclcpp"
        "px4_msgs"
        "std_msgs"
        "sensor_msgs"
        #    "agent_msg"
        "OpenCV")
target_link_libraries(qhac5)

ament_export_dependencies(rclcpp rosidl_default_runtime)
ament_export_include_directories(include ${px4_msgs_INCLUDE_DIRS})
ament_export_libraries()


ament_package()
