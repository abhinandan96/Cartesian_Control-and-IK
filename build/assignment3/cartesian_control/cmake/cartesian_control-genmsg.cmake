# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cartesian_control: 1 messages, 0 services")

set(MSG_I_FLAGS "-Icartesian_control:/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cartesian_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg" NAME_WE)
add_custom_target(_cartesian_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cartesian_control" "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg" "geometry_msgs/Quaternion:geometry_msgs/Transform:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cartesian_control
  "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cartesian_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(cartesian_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cartesian_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cartesian_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cartesian_control_generate_messages cartesian_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg" NAME_WE)
add_dependencies(cartesian_control_generate_messages_cpp _cartesian_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cartesian_control_gencpp)
add_dependencies(cartesian_control_gencpp cartesian_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cartesian_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(cartesian_control
  "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cartesian_control
)

### Generating Services

### Generating Module File
_generate_module_eus(cartesian_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cartesian_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cartesian_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cartesian_control_generate_messages cartesian_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg" NAME_WE)
add_dependencies(cartesian_control_generate_messages_eus _cartesian_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cartesian_control_geneus)
add_dependencies(cartesian_control_geneus cartesian_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cartesian_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cartesian_control
  "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cartesian_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(cartesian_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cartesian_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cartesian_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cartesian_control_generate_messages cartesian_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg" NAME_WE)
add_dependencies(cartesian_control_generate_messages_lisp _cartesian_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cartesian_control_genlisp)
add_dependencies(cartesian_control_genlisp cartesian_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cartesian_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(cartesian_control
  "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cartesian_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(cartesian_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cartesian_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cartesian_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cartesian_control_generate_messages cartesian_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg" NAME_WE)
add_dependencies(cartesian_control_generate_messages_nodejs _cartesian_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cartesian_control_gennodejs)
add_dependencies(cartesian_control_gennodejs cartesian_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cartesian_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cartesian_control
  "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cartesian_control
)

### Generating Services

### Generating Module File
_generate_module_py(cartesian_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cartesian_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cartesian_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cartesian_control_generate_messages cartesian_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg/CartesianCommand.msg" NAME_WE)
add_dependencies(cartesian_control_generate_messages_py _cartesian_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cartesian_control_genpy)
add_dependencies(cartesian_control_genpy cartesian_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cartesian_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cartesian_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cartesian_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(cartesian_control_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(cartesian_control_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(cartesian_control_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cartesian_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cartesian_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(cartesian_control_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(cartesian_control_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(cartesian_control_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cartesian_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cartesian_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(cartesian_control_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(cartesian_control_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(cartesian_control_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cartesian_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cartesian_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(cartesian_control_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(cartesian_control_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(cartesian_control_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cartesian_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cartesian_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cartesian_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(cartesian_control_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(cartesian_control_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(cartesian_control_generate_messages_py geometry_msgs_generate_messages_py)
endif()
