# Install script for directory: /home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hademo/msg" TYPE FILE FILES
    "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Action.msg"
    "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Args.msg"
    "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Func_and_Args.msg"
    "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/Result.msg"
    "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/msg/ResultInfo.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hademo/cmake" TYPE FILE FILES "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/build/hademo/catkin_generated/installspace/hademo-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/devel/include/hademo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/devel/share/roseus/ros/hademo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/devel/share/common-lisp/ros/hademo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/devel/share/gennodejs/ros/hademo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/devel/lib/python3/dist-packages/hademo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/devel/lib/python3/dist-packages/hademo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/build/hademo/catkin_generated/installspace/hademo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hademo/cmake" TYPE FILE FILES "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/build/hademo/catkin_generated/installspace/hademo-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hademo/cmake" TYPE FILE FILES
    "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/build/hademo/catkin_generated/installspace/hademoConfig.cmake"
    "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/build/hademo/catkin_generated/installspace/hademoConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hademo" TYPE FILE FILES "/home/pjlab/workspace/OmniGibson/Benchmark/ros_hademo_ws/src/hademo/package.xml")
endif()

