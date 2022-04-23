# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rosns3_client: 1 messages, 0 services")

set(MSG_I_FLAGS "-Irosns3_client:/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rosns3_client_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg" NAME_WE)
add_custom_target(_rosns3_client_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rosns3_client" "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rosns3_client
  "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosns3_client
)

### Generating Services

### Generating Module File
_generate_module_cpp(rosns3_client
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosns3_client
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rosns3_client_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rosns3_client_generate_messages rosns3_client_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg" NAME_WE)
add_dependencies(rosns3_client_generate_messages_cpp _rosns3_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosns3_client_gencpp)
add_dependencies(rosns3_client_gencpp rosns3_client_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosns3_client_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rosns3_client
  "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosns3_client
)

### Generating Services

### Generating Module File
_generate_module_eus(rosns3_client
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosns3_client
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rosns3_client_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rosns3_client_generate_messages rosns3_client_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg" NAME_WE)
add_dependencies(rosns3_client_generate_messages_eus _rosns3_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosns3_client_geneus)
add_dependencies(rosns3_client_geneus rosns3_client_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosns3_client_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rosns3_client
  "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosns3_client
)

### Generating Services

### Generating Module File
_generate_module_lisp(rosns3_client
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosns3_client
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rosns3_client_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rosns3_client_generate_messages rosns3_client_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg" NAME_WE)
add_dependencies(rosns3_client_generate_messages_lisp _rosns3_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosns3_client_genlisp)
add_dependencies(rosns3_client_genlisp rosns3_client_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosns3_client_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rosns3_client
  "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosns3_client
)

### Generating Services

### Generating Module File
_generate_module_nodejs(rosns3_client
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosns3_client
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rosns3_client_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rosns3_client_generate_messages rosns3_client_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg" NAME_WE)
add_dependencies(rosns3_client_generate_messages_nodejs _rosns3_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosns3_client_gennodejs)
add_dependencies(rosns3_client_gennodejs rosns3_client_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosns3_client_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rosns3_client
  "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosns3_client
)

### Generating Services

### Generating Module File
_generate_module_py(rosns3_client
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosns3_client
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rosns3_client_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rosns3_client_generate_messages rosns3_client_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/malintha/rosns3_ws/src/rosns3_client/rosns3_client/msg/Waypoint.msg" NAME_WE)
add_dependencies(rosns3_client_generate_messages_py _rosns3_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosns3_client_genpy)
add_dependencies(rosns3_client_genpy rosns3_client_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosns3_client_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosns3_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosns3_client
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rosns3_client_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rosns3_client_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosns3_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosns3_client
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rosns3_client_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(rosns3_client_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosns3_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosns3_client
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rosns3_client_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(rosns3_client_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosns3_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosns3_client
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rosns3_client_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(rosns3_client_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosns3_client)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosns3_client\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosns3_client
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rosns3_client_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rosns3_client_generate_messages_py geometry_msgs_generate_messages_py)
endif()
