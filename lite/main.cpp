//
// Created by redwan on 4/16/22.
//

#include <iostream>
#include <cassert>
#include <fstream>
#include "src/ompl_planner.h"
using namespace std;

template<typename T>
void save_results(const T& res)
{
    size_t N = res.first.size();
    ofstream pathFile;
    pathFile.open ("path.txt");
    cout << "Writing path to the path.txt file.\n";


    for (int i = 0; i < N; ++i) {
        pathFile << res.first[i] << "," << res.second[i] << "\n";
    }

    pathFile.close();
}


int main(int argc, char *argv[])
{
    assert(argc > 1 && "filepath not found in args");
    cout << "VFRRT lite version  "<< endl;
    auto params = make_shared<param_parser>(argv[1]);
    auto datasetPtr = make_shared<dataset_parser>(params->get_ptr());

//    configure experiment
    cout << *params << endl;

    ompl_planner planner(params->get_start_loc(), params->get_goal_loca());
    auto obstacles = make_shared<polygonal_obstacles>(params->get_ptr());
    planner.setup(obstacles->get_ptr());
    auto res = planner.get_solution(datasetPtr->get_ptr(),  params->get_depth_index());
    save_results(res);


    return 0;
}