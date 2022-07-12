//
// Created by redwan on 7/4/22.
//
#include <iostream>
#include <cassert>
#include <fstream>
#include "../src/ompl_planner.h"
#include <benchmark/benchmark.h>
#include <Eigen/Core>
using namespace std;

void ompl::msg::setLogLevel(LogLevel level) {
    level = LOG_NONE;
}

double pathLength(const ompl_planner::PATH & path)
{
    double length = 0;
    for (int i = 1; i < path.first.size(); ++i) {
        Eigen::Vector2d x1(path.first[i-1], path.second[i-1]);
        Eigen::Vector2d x2(path.first[i], path.second[i]);
        length += (x1 - x2).norm();
    }
    return length;
}


double angleDiff(const ompl_planner::PATH & path, DatasetPtr dataset, int depth)
{
    auto U = dataset->retrieve_data(depth, gUU);
    auto V = dataset->retrieve_data(depth, gVV);

    double total = 0;
    for (int i = 1; i < path.first.size(); ++i) {
        Eigen::Vector2d x1(path.first[i-1], path.second[i-1]);
        Eigen::Vector2d x2(path.first[i], path.second[i]);
        Eigen::Vector2d deltax = x2 - x1;
        double pathAngle = atan2(deltax(1), deltax(0));

        int i0 = path.second[i-1];
        int j0 = path.first[i-1];
        int i1 = path.second[i];
        int j1 = path.first[i];
        Eigen::Vector2d y1(U[i0][j0], V[i0][j0] );
        Eigen::Vector2d y2(U[i1][j1], V[i1][j1]);
        Eigen::Vector2d deltay = y2 - y1;
        double forceAngle = atan2(deltay(1), deltay(0));
        total += fmod(abs(pathAngle - forceAngle), 2 * M_PI);


    }
    return total;
}

static void BM_VanillaVFRRT(benchmark::State& state)
{
    auto config = "../../resources/config.yaml";
    auto params = make_shared<param_parser>(config);
    auto datasetPtr = make_shared<dataset_parser>(params->get_ptr());

    datasetPtr->open(params->nc_file());
    ompl_planner planner(params->get_start_loc(), params->get_goal_loca());
    auto obstacles = make_shared<polygonal_obstacles>(params->get_ptr());
    planner.setup(obstacles->get_ptr());
    std::stringstream ss;
    for (auto _ : state)
    {
        auto res = planner.get_solution(datasetPtr->get_ptr(),  params->get_depth_index());
        state.counters["pathLength"] = pathLength(res);
        state.counters["angleDiff"] = angleDiff(res, datasetPtr, params->get_depth_index());
    }

}

static void BM_DeepVFRRT(benchmark::State& state)
{
    auto config = "../../resources/pconfig.yaml";
    auto params = make_shared<param_parser>(config);
    auto datasetPtr = make_shared<dataset_parser>(params->get_ptr());
    datasetPtr->open(params->nc_file());

    ompl_planner planner(params->get_start_loc(), params->get_goal_loca());
    auto obstacles = make_shared<polygonal_obstacles>(params->get_ptr());
    planner.setup(obstacles->get_ptr());
    for (auto _ : state)
    {
        auto res = planner.get_solution(datasetPtr->get_ptr(),  params->get_depth_index());
        state.counters["pathLength"] = pathLength(res);
        state.counters["angleDiff"] = angleDiff(res, datasetPtr, params->get_depth_index());

    }


}
const int repeat = 10;
BENCHMARK(BM_VanillaVFRRT)->UseRealTime()->Repetitions(repeat)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_DeepVFRRT)->UseRealTime()->Repetitions(repeat)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();

