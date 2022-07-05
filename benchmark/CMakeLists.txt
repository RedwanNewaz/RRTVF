
CPMAddPackage(
        NAME benchmark
        GITHUB_REPOSITORY google/benchmark
        VERSION 1.5.2
        OPTIONS "BENCHMARK_ENABLE_TESTING Off"
)
CPMAddPackage("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")
CPMAddPackage("gh:Unidata/netcdf-cxx4#master")


add_executable(${PROJECT_NAME}_benchmark  main.cpp ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_benchmark
        netcdf-cxx4
        yaml-cpp
        Eigen3::Eigen
        ${OMPL_LIBRARY}
        benchmark
        )