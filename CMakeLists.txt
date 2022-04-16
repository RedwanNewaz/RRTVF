cmake_minimum_required(VERSION 3.20)
project(VFRRT)

set(CMAKE_CXX_STANDARD 17)
include(cmake/CPM.cmake)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(SOURCE_FILES
        src/dataset_parser.cpp src/dataset_parser.h src/ompl_planner.cpp src/ompl_planner.h src/polygonal_obstacles.cpp src/polygonal_obstacles.h src/param_parser.cpp src/param_parser.h
        )
find_package(OMPL REQUIRED)
include_directories(
        ${ompl_INCLUDE_DIRS}
        /usr/local/include
        include
)

option(BUILD_GUI "Build the gui library" ON)
option(BUILD_LITE "Build the gui library" ON)



if(BUILD_GUI)
    add_subdirectory(gui)
endif(BUILD_GUI)

if(BUILD_LITE)
    add_subdirectory(lite)
endif(BUILD_LITE)
