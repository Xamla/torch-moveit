CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

PROJECT(torch-moveit)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

FIND_PACKAGE(Torch REQUIRED)
find_package(Boost 1.47.0 REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  tf
   tf_conversions
  roscpp
)

SET(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

SET(src
  "${SOURCE_DIR}/movegroup.cpp"
  "${SOURCE_DIR}/plan.cpp"
  "${SOURCE_DIR}/robotstate.cpp"
  "${SOURCE_DIR}/collision_object.cpp"
  "${SOURCE_DIR}/planning_scene_interface.cpp"
  "${SOURCE_DIR}/robot_model_loader.cpp"
  "${SOURCE_DIR}/robot_model.cpp"
  "${SOURCE_DIR}/planning_scene.cpp"
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Torch_INSTALL_INCLUDE}
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
  ${Torch_INSTALL_LIB}
  ${Boost_LIBRARY_DIRS}
)

FILE(GLOB luasrc *.lua)

ADD_LIBRARY(moveit MODULE ${src})

target_link_libraries(moveit TH ${catkin_LIBRARIES})

INSTALL(TARGETS moveit LIBRARY DESTINATION ${Torch_INSTALL_LUA_CPATH_SUBDIR})
#INSTALL(DIRECTORY "doc" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/moveit")
INSTALL(FILES "README.md" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/moveit")
INSTALL(FILES ${luasrc} DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/moveit")
