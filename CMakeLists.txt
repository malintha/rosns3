cmake_minimum_required(VERSION 3.0.2)

macro(use_cxx11)
    if (CMAKE_VERSION VERSION_LESS "3.1")
        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
        endif ()
    else ()
        set(CMAKE_CXX_STANDARD 11)
    endif ()
endmacro(use_cxx11)

project(rosns3_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  simulator_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


include_directories(
  include/${PROJECT_NAME}
  include/messages
  ${catkin_INCLUDE_DIRS}
  )
 
add_executable(${PROJECT_NAME}     
src/rosns3_client.cpp
src/utils.cpp
src/client.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}     
)  

