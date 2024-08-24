# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "block_localization: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iblock_localization:/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(block_localization_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg" NAME_WE)
add_custom_target(_block_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_localization" "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg" "sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Header"
)

get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv" NAME_WE)
add_custom_target(_block_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_localization" "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(block_localization
  "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_localization
)

### Generating Services
_generate_srv_cpp(block_localization
  "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_localization
)

### Generating Module File
_generate_module_cpp(block_localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(block_localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(block_localization_generate_messages block_localization_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg" NAME_WE)
add_dependencies(block_localization_generate_messages_cpp _block_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv" NAME_WE)
add_dependencies(block_localization_generate_messages_cpp _block_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_localization_gencpp)
add_dependencies(block_localization_gencpp block_localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_localization_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(block_localization
  "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_localization
)

### Generating Services
_generate_srv_eus(block_localization
  "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_localization
)

### Generating Module File
_generate_module_eus(block_localization
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_localization
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(block_localization_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(block_localization_generate_messages block_localization_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg" NAME_WE)
add_dependencies(block_localization_generate_messages_eus _block_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv" NAME_WE)
add_dependencies(block_localization_generate_messages_eus _block_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_localization_geneus)
add_dependencies(block_localization_geneus block_localization_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_localization_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(block_localization
  "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_localization
)

### Generating Services
_generate_srv_lisp(block_localization
  "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_localization
)

### Generating Module File
_generate_module_lisp(block_localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(block_localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(block_localization_generate_messages block_localization_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg" NAME_WE)
add_dependencies(block_localization_generate_messages_lisp _block_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv" NAME_WE)
add_dependencies(block_localization_generate_messages_lisp _block_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_localization_genlisp)
add_dependencies(block_localization_genlisp block_localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_localization_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(block_localization
  "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_localization
)

### Generating Services
_generate_srv_nodejs(block_localization
  "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_localization
)

### Generating Module File
_generate_module_nodejs(block_localization
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_localization
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(block_localization_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(block_localization_generate_messages block_localization_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg" NAME_WE)
add_dependencies(block_localization_generate_messages_nodejs _block_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv" NAME_WE)
add_dependencies(block_localization_generate_messages_nodejs _block_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_localization_gennodejs)
add_dependencies(block_localization_gennodejs block_localization_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_localization_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(block_localization
  "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_localization
)

### Generating Services
_generate_srv_py(block_localization
  "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_localization
)

### Generating Module File
_generate_module_py(block_localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(block_localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(block_localization_generate_messages block_localization_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/msg/cloud_info.msg" NAME_WE)
add_dependencies(block_localization_generate_messages_py _block_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yixin/icra_ws/src/Block-Map-Based-Localization/srv/queryMap.srv" NAME_WE)
add_dependencies(block_localization_generate_messages_py _block_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_localization_genpy)
add_dependencies(block_localization_genpy block_localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_localization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(block_localization_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(block_localization_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(block_localization_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(block_localization_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_localization
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(block_localization_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(block_localization_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(block_localization_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(block_localization_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(block_localization_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(block_localization_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(block_localization_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(block_localization_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_localization
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(block_localization_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(block_localization_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(block_localization_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(block_localization_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(block_localization_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(block_localization_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(block_localization_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(block_localization_generate_messages_py sensor_msgs_generate_messages_py)
endif()
