#include <iostream>
#include "matplotlibcpp.h"
#include "src/ompl_planner.h"
#include "cassert"

using namespace std;
namespace plt = matplotlibcpp;


DatasetPtr get_roms_data_vf(ParamPtr params)
{
    auto dataset = make_shared<dataset_parser>(params->get_ptr());
    int depth = params->get_depth_index();

    if (dataset->open())
    {
        std::vector<float> x, y, u, v;
        auto resU = dataset->retrieve_data(depth, UU);
        auto resV = dataset->retrieve_data(depth, VV);
        //      TODO update x tick and y tick value with lat and lon
        for (int i = 0; i < resU.size(); ++i) {
            for (int j = 0; j < resU[i].size(); ++j) {
                x.push_back(i);
                u.push_back(resU[j][i]);
                y.push_back(j);
                v.push_back(resV[j][i]);
            }
        }
        plt::quiver(x, y, u, v);

    }
    return dataset->get_ptr();
}



int main(int argc, char * argv[]) {

    assert(argc > 1 && "filepath not found in args");

    auto params = make_shared<param_parser>("../resources/config.yaml");

    cout << *params << endl;

    int depth = params->get_depth_index();

    auto datasetPtr = get_roms_data_vf(params);

    auto obstacles = make_shared<polygonal_obstacles>(params->get_ptr());

    for(const auto& ob:obstacles->get_obstacles())
        plt::plot(ob.first, ob.second);

    ompl_planner planner(params->get_start_loc(), params->get_goal_loca());


    planner.setup(obstacles->get_ptr());
    auto res = planner.get_solution(datasetPtr->get_ptr(), depth);
    plt::plot(res.first, res.second);
    // show confidence
    vector<float>upper, lower;
    float ci = 1.96;
    for (float i : res.second) {
        upper.push_back(i + ci);
        lower.push_back(i - ci);

    }

    std::map<string, string> keywords;
    keywords["alpha"] = "0.4";
    keywords["color"] = "grey";
    keywords["hatch"] = "-";

    plt::fill_between(res.first, upper, lower,  keywords);
    plt::show();

    return 0;
}