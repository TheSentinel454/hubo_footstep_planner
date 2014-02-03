cmake_minimum_required(VERSION 2.8)
project(FootstepPlanner)

find_package(OpenSceneGraph)

include_directories(${OpenSceneGraph_INCLUDE_DIRS})
# Include the Eigen folder
include_directories(/usr/include/eigen3)
link_libraries(osg osgViewer OpenThreads fcl boost_system)

file(GLOB SRC_LIST "src/*.cpp" "include/*.h")
list(SORT SRC_LIST)

add_executable(${PROJECT_NAME} ${SRC_LIST})
