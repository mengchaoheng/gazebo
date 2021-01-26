# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)


find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} 
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_subdirectory(msgs)
#target_link_libraries(ductedfan_msgs ${GAZEBO_libraries})

add_library(DFGUIPlugin SHARED ${headers_MOC}
  DFGUIPlugin.cc)
target_link_libraries(DFGUIPlugin ductedfan_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
add_dependencies(DFGUIPlugin ductedfan_msgs)
    
    
 #  gui_example_spawn_widget
add_library(DFPlugin SHARED DFPlugin.cc)
target_link_libraries(DFPlugin ductedfan_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(DFPlugin ductedfan_msgs)


add_library(LiftDragPlugin SHARED LiftDragPlugin.cc)
target_link_libraries(LiftDragPlugin ${GAZEBO_LIBRARIES})

