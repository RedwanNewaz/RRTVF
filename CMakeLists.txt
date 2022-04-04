cmake_minimum_required(VERSION 3.21)
project(VFRRT)

set(CMAKE_CXX_STANDARD 17)
include(cmake/CPM.cmake)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OMPL REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

link_directories(
        ${OMPLAPP_LIBRARY_DIRS}
        ${OMPL_LIBRARY_DIRS}
        /usr/local/lib)

include_directories(
        ${OMPLAPP_INCLUDE_DIR}
        ${ompl_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        /usr/include/python3.8
        include
        /usr/local/include)

CPMAddPackage("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")
add_executable(VFRRT main.cpp src/dataset_parser.cpp src/dataset_parser.h src/ompl_planner.cpp src/ompl_planner.h src/polygonal_obstacles.cpp src/polygonal_obstacles.h src/param_parser.cpp src/param_parser.h)
target_link_libraries(${PROJECT_NAME}
        ${OMPL_LIBRARY}
        ${OMPLAPP_LIBRARY}
        ${OMPLAPPBASE_LIBRARY}
        netcdf-cxx4
        -lpython3.8
        Eigen3::Eigen
        yaml-cpp
        )