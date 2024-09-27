# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hademo: 5 messages, 0 services")

set(MSG_I_FLAGS "-Ihademo:/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hademo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg" NAME_WE)
add_custom_target(_hademo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hademo" "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg" "std_msgs/MultiArrayLayout:std_msgs/Float64MultiArray:std_msgs/MultiArrayDimension:hademo/Func_and_Args:hademo/Args"
)

get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg" NAME_WE)
add_custom_target(_hademo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hademo" "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg" "std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray:std_msgs/MultiArrayLayout"
)

get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg" NAME_WE)
add_custom_target(_hademo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hademo" "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg" "std_msgs/MultiArrayDimension:std_msgs/MultiArrayLayout:std_msgs/Float64MultiArray:hademo/Args"
)

get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg" NAME_WE)
add_custom_target(_hademo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hademo" "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg" "hademo/ResultInfo"
)

get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg" NAME_WE)
add_custom_target(_hademo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hademo" "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hademo
)
_generate_msg_cpp(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hademo
)
_generate_msg_cpp(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hademo
)
_generate_msg_cpp(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg"
  "${MSG_I_FLAGS}"
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hademo
)
_generate_msg_cpp(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hademo
)

### Generating Services

### Generating Module File
_generate_module_cpp(hademo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hademo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hademo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hademo_generate_messages hademo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg" NAME_WE)
add_dependencies(hademo_generate_messages_cpp _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg" NAME_WE)
add_dependencies(hademo_generate_messages_cpp _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg" NAME_WE)
add_dependencies(hademo_generate_messages_cpp _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg" NAME_WE)
add_dependencies(hademo_generate_messages_cpp _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg" NAME_WE)
add_dependencies(hademo_generate_messages_cpp _hademo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hademo_gencpp)
add_dependencies(hademo_gencpp hademo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hademo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hademo
)
_generate_msg_eus(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hademo
)
_generate_msg_eus(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hademo
)
_generate_msg_eus(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg"
  "${MSG_I_FLAGS}"
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hademo
)
_generate_msg_eus(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hademo
)

### Generating Services

### Generating Module File
_generate_module_eus(hademo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hademo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hademo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hademo_generate_messages hademo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg" NAME_WE)
add_dependencies(hademo_generate_messages_eus _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg" NAME_WE)
add_dependencies(hademo_generate_messages_eus _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg" NAME_WE)
add_dependencies(hademo_generate_messages_eus _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg" NAME_WE)
add_dependencies(hademo_generate_messages_eus _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg" NAME_WE)
add_dependencies(hademo_generate_messages_eus _hademo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hademo_geneus)
add_dependencies(hademo_geneus hademo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hademo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hademo
)
_generate_msg_lisp(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hademo
)
_generate_msg_lisp(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hademo
)
_generate_msg_lisp(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg"
  "${MSG_I_FLAGS}"
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hademo
)
_generate_msg_lisp(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hademo
)

### Generating Services

### Generating Module File
_generate_module_lisp(hademo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hademo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hademo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hademo_generate_messages hademo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg" NAME_WE)
add_dependencies(hademo_generate_messages_lisp _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg" NAME_WE)
add_dependencies(hademo_generate_messages_lisp _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg" NAME_WE)
add_dependencies(hademo_generate_messages_lisp _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg" NAME_WE)
add_dependencies(hademo_generate_messages_lisp _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg" NAME_WE)
add_dependencies(hademo_generate_messages_lisp _hademo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hademo_genlisp)
add_dependencies(hademo_genlisp hademo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hademo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hademo
)
_generate_msg_nodejs(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hademo
)
_generate_msg_nodejs(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hademo
)
_generate_msg_nodejs(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg"
  "${MSG_I_FLAGS}"
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hademo
)
_generate_msg_nodejs(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hademo
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hademo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hademo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hademo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hademo_generate_messages hademo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg" NAME_WE)
add_dependencies(hademo_generate_messages_nodejs _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg" NAME_WE)
add_dependencies(hademo_generate_messages_nodejs _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg" NAME_WE)
add_dependencies(hademo_generate_messages_nodejs _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg" NAME_WE)
add_dependencies(hademo_generate_messages_nodejs _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg" NAME_WE)
add_dependencies(hademo_generate_messages_nodejs _hademo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hademo_gennodejs)
add_dependencies(hademo_gennodejs hademo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hademo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hademo
)
_generate_msg_py(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hademo
)
_generate_msg_py(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hademo
)
_generate_msg_py(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg"
  "${MSG_I_FLAGS}"
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hademo
)
_generate_msg_py(hademo
  "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hademo
)

### Generating Services

### Generating Module File
_generate_module_py(hademo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hademo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hademo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hademo_generate_messages hademo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg" NAME_WE)
add_dependencies(hademo_generate_messages_py _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg" NAME_WE)
add_dependencies(hademo_generate_messages_py _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg" NAME_WE)
add_dependencies(hademo_generate_messages_py _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg" NAME_WE)
add_dependencies(hademo_generate_messages_py _hademo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg" NAME_WE)
add_dependencies(hademo_generate_messages_py _hademo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hademo_genpy)
add_dependencies(hademo_genpy hademo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hademo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hademo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hademo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hademo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hademo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hademo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hademo_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hademo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hademo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hademo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hademo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hademo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hademo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hademo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hademo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hademo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hademo_generate_messages_py std_msgs_generate_messages_py)
endif()
