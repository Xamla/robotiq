cmake_minimum_required(VERSION 2.8.3)
project(robotiq_ethercat)

find_package(catkin REQUIRED COMPONENTS
  soem
  roscpp
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES robotiq_ethercat
   CATKIN_DEPENDS soem roscpp
)

# C++11 Needed
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ## The following work around allows SOEM headers to include other SOEM headers.
  ## SOEM headers assume all headers are installed in a flat directory structure
  ## See https://github.com/smits/soem/issues/4 for more information.
  ${soem_INCLUDE_DIRS}/soem
)

add_library(robotiq_ethercat
 include/robotiq_ethercat/ethercat_manager.h
 src/ethercat_manager.cpp
)

install(TARGETS robotiq_ethercat
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
