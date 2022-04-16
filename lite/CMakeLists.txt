CPMAddPackage("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")
CPMAddPackage("gh:Unidata/netcdf-cxx4#master@master")




add_executable(${PROJECT_NAME}_lite  main.cpp ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_lite
        netcdf-cxx4
        yaml-cpp
        Eigen3::Eigen
        ${OMPL_LIBRARY}
        )

