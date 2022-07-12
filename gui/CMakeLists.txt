
link_directories(
#        ${OMPLAPP_LIBRARY_DIRS}
#        ${OMPL_LIBRARY_DIRS}
        /usr/local/lib)

include_directories(
        ${OMPLAPP_INCLUDE_DIR}
        ${ompl_INCLUDE_DIRS}
#        ${OpenCV_INCLUDE_DIRS}
        /usr/include/python3.8
        include
        /usr/local/include)

CPMAddPackage("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")
add_executable(${PROJECT_NAME}_gui main.cpp ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_gui
        ${OMPL_LIBRARY}
#        ${OMPLAPP_LIBRARY}
#        ${OMPLAPPBASE_LIBRARY}
        netcdf-cxx4
        -lpython3.8
        Eigen3::Eigen
        yaml-cpp
        )