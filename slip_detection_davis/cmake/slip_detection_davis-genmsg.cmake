# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "slip_detection_davis: 3 messages, 1 services")

set(MSG_I_FLAGS "-Islip_detection_davis:/home/user/catkin_ws/src/slip_detection_davis/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Ibaxter_core_msgs:/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(slip_detection_davis_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg" NAME_WE)
add_custom_target(_slip_detection_davis_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "slip_detection_davis" "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg" "geometry_msgs/Vector3:geometry_msgs/WrenchStamped:std_msgs/Header:baxter_core_msgs/EndEffectorState:geometry_msgs/Wrench"
)

get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv" NAME_WE)
add_custom_target(_slip_detection_davis_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "slip_detection_davis" "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv" ""
)

get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg" NAME_WE)
add_custom_target(_slip_detection_davis_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "slip_detection_davis" "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg" "geometry_msgs/Vector3:geometry_msgs/WrenchStamped:std_msgs/Header:baxter_core_msgs/EndEffectorState:geometry_msgs/Wrench"
)

get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg" NAME_WE)
add_custom_target(_slip_detection_davis_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "slip_detection_davis" "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg" "std_msgs/MultiArrayDimension:std_msgs/Header:std_msgs/MultiArrayLayout:geometry_msgs/WrenchStamped:geometry_msgs/Vector3:geometry_msgs/Wrench:std_msgs/Float64MultiArray:baxter_core_msgs/EndEffectorState"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slip_detection_davis
)
_generate_msg_cpp(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slip_detection_davis
)
_generate_msg_cpp(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slip_detection_davis
)

### Generating Services
_generate_srv_cpp(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slip_detection_davis
)

### Generating Module File
_generate_module_cpp(slip_detection_davis
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slip_detection_davis
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(slip_detection_davis_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(slip_detection_davis_generate_messages slip_detection_davis_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_cpp _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_cpp _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_cpp _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_cpp _slip_detection_davis_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(slip_detection_davis_gencpp)
add_dependencies(slip_detection_davis_gencpp slip_detection_davis_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS slip_detection_davis_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/slip_detection_davis
)
_generate_msg_eus(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/slip_detection_davis
)
_generate_msg_eus(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/slip_detection_davis
)

### Generating Services
_generate_srv_eus(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/slip_detection_davis
)

### Generating Module File
_generate_module_eus(slip_detection_davis
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/slip_detection_davis
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(slip_detection_davis_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(slip_detection_davis_generate_messages slip_detection_davis_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_eus _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_eus _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_eus _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_eus _slip_detection_davis_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(slip_detection_davis_geneus)
add_dependencies(slip_detection_davis_geneus slip_detection_davis_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS slip_detection_davis_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slip_detection_davis
)
_generate_msg_lisp(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slip_detection_davis
)
_generate_msg_lisp(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slip_detection_davis
)

### Generating Services
_generate_srv_lisp(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slip_detection_davis
)

### Generating Module File
_generate_module_lisp(slip_detection_davis
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slip_detection_davis
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(slip_detection_davis_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(slip_detection_davis_generate_messages slip_detection_davis_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_lisp _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_lisp _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_lisp _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_lisp _slip_detection_davis_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(slip_detection_davis_genlisp)
add_dependencies(slip_detection_davis_genlisp slip_detection_davis_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS slip_detection_davis_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/slip_detection_davis
)
_generate_msg_nodejs(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/slip_detection_davis
)
_generate_msg_nodejs(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/slip_detection_davis
)

### Generating Services
_generate_srv_nodejs(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/slip_detection_davis
)

### Generating Module File
_generate_module_nodejs(slip_detection_davis
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/slip_detection_davis
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(slip_detection_davis_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(slip_detection_davis_generate_messages slip_detection_davis_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_nodejs _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_nodejs _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_nodejs _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_nodejs _slip_detection_davis_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(slip_detection_davis_gennodejs)
add_dependencies(slip_detection_davis_gennodejs slip_detection_davis_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS slip_detection_davis_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slip_detection_davis
)
_generate_msg_py(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slip_detection_davis
)
_generate_msg_py(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/home/user/catkin_ws/src/baxter_common/baxter_core_msgs/msg/EndEffectorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slip_detection_davis
)

### Generating Services
_generate_srv_py(slip_detection_davis
  "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slip_detection_davis
)

### Generating Module File
_generate_module_py(slip_detection_davis
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slip_detection_davis
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(slip_detection_davis_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(slip_detection_davis_generate_messages slip_detection_davis_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final3.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_py _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/srv/object_test.srv" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_py _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/SlipDetectionDataComplete_analysis_final4.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_py _slip_detection_davis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/catkin_ws/src/slip_detection_davis/msg/CompressiveForceGrasp_stable_Final.msg" NAME_WE)
add_dependencies(slip_detection_davis_generate_messages_py _slip_detection_davis_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(slip_detection_davis_genpy)
add_dependencies(slip_detection_davis_genpy slip_detection_davis_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS slip_detection_davis_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slip_detection_davis)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slip_detection_davis
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(slip_detection_davis_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(slip_detection_davis_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(slip_detection_davis_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET baxter_core_msgs_generate_messages_cpp)
  add_dependencies(slip_detection_davis_generate_messages_cpp baxter_core_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/slip_detection_davis)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/slip_detection_davis
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(slip_detection_davis_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(slip_detection_davis_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(slip_detection_davis_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET baxter_core_msgs_generate_messages_eus)
  add_dependencies(slip_detection_davis_generate_messages_eus baxter_core_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slip_detection_davis)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slip_detection_davis
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(slip_detection_davis_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(slip_detection_davis_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(slip_detection_davis_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET baxter_core_msgs_generate_messages_lisp)
  add_dependencies(slip_detection_davis_generate_messages_lisp baxter_core_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/slip_detection_davis)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/slip_detection_davis
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(slip_detection_davis_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(slip_detection_davis_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(slip_detection_davis_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET baxter_core_msgs_generate_messages_nodejs)
  add_dependencies(slip_detection_davis_generate_messages_nodejs baxter_core_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slip_detection_davis)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slip_detection_davis\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slip_detection_davis
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(slip_detection_davis_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(slip_detection_davis_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(slip_detection_davis_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET baxter_core_msgs_generate_messages_py)
  add_dependencies(slip_detection_davis_generate_messages_py baxter_core_msgs_generate_messages_py)
endif()
