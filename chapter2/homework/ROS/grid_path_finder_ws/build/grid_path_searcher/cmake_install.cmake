# Install script for directory: /home/sunnypaul/Project/github/Learn_Motion_Planning/chapter2/homework/ROS/grid_path_finder_ws/src/grid_path_searcher

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sunnypaul/Project/github/Learn_Motion_Planning/chapter2/homework/ROS/grid_path_finder_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/grid_path_searcher" TYPE FILE FILES "/home/sunnypaul/Project/github/Learn_Motion_Planning/chapter2/homework/ROS/grid_path_finder_ws/devel/include/grid_path_searcher/dy_configConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/grid_path_searcher" TYPE FILE FILES "/home/sunnypaul/Project/github/Learn_Motion_Planning/chapter2/homework/ROS/grid_path_finder_ws/devel/lib/python2.7/dist-packages/grid_path_searcher/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/sunnypaul/Project/github/Learn_Motion_Planning/chapter2/homework/ROS/grid_path_finder_ws/devel/lib/python2.7/dist-packages/grid_path_searcher/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/grid_path_searcher" TYPE DIRECTORY FILES "/home/sunnypaul/Project/github/Learn_Motion_Planning/chapter2/homework/ROS/grid_path_finder_ws/devel/lib/python2.7/dist-packages/grid_path_searcher/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sunnypaul/Project/github/Learn_Motion_Planning/chapter2/homework/ROS/grid_path_finder_ws/build/grid_path_searcher/catkin_generated/installspace/grid_path_searcher.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/grid_path_searcher/cmake" TYPE FILE FILES
    "/home/sunnypaul/Project/github/Learn_Motion_Planning/chapter2/homework/ROS/grid_path_finder_ws/build/grid_path_searcher/catkin_generated/installspace/grid_path_searcherConfig.cmake"
    "/home/sunnypaul/Project/github/Learn_Motion_Planning/chapter2/homework/ROS/grid_path_finder_ws/build/grid_path_searcher/catkin_generated/installspace/grid_path_searcherConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/grid_path_searcher" TYPE FILE FILES "/home/sunnypaul/Project/github/Learn_Motion_Planning/chapter2/homework/ROS/grid_path_finder_ws/src/grid_path_searcher/package.xml")
endif()

