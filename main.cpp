#include <iostream>

#include "matplotlibcpp.h"
#include "src/ompl_planner.h"
#include "cassert"

using namespace std;
namespace plt = matplotlibcpp;


DatasetPtr get_roms_data_vf(const char *filepath, int depth)
{
    auto dataset = make_shared<dataset_parser>(filepath);

    if (dataset->open())
    {
        std::vector<float> x, y, u, v;
        auto resU = dataset->retrieve_data(depth, UU);
        auto resV = dataset->retrieve_data(depth, VV);

        //      TODO update x tick and y tick value with lat and lon
        for (int i = 0; i < resU.size(); ++i) {
            for (int j = 0; j < resU[i].size(); ++j) {
                //                cout << res[i][j] << " ";
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
    int depth = atoi(argv[2]);

    auto datasetPtr = get_roms_data_vf(argv[1], depth);

    vector<float> obX{10, 15, 15, 10, 10}, obY{2, 2, 15, 15, 2};
    auto obstacles = make_shared<polygonal_obstacles>();
    obstacles->append(obX, obY);

    for(const auto& ob:obstacles->get_obstacles())
        plt::plot(ob.first, ob.second);

    ompl_planner planner({0, 1}, {20, 15});
//    ompl_planner planner({20, 15}, {0, 1});

    planner.setup(obstacles->get_ptr());
    auto res = planner.get_solution(datasetPtr, depth);
    plt::plot(res.first, res.second);
    plt::show();

    return 0;
}