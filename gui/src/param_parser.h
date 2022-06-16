//
// Created by redwan on 4/4/22.
//

#ifndef VFRRT_PARAM_PARSER_H
#define VFRRT_PARAM_PARSER_H

#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

using namespace std;
using namespace YAML;

class param_parser;
typedef shared_ptr<param_parser> ParamPtr;


class param_parser: public enable_shared_from_this<param_parser>{
    using OBS = pair<vector<float>, vector<float>>;
public:
    explicit param_parser(const char* filename);
    ParamPtr get_ptr();
    vector<OBS>get_obstacle_list();
    double get_solve_time();
    double get_exploration_const();
    double get_initial_lambda();
    int get_update_freq();
    vector<float>get_start_loc();
    vector<float>get_goal_loca();
    string get_dataset_path();
    int get_depth_index();
    int get_min_index();
    int get_max_index();
    string get_uu_file();
    string get_vv_file();
    friend ostream &operator<<(ostream &os, const param_parser &parser);

private:
    Node config_;
};


#endif //VFRRT_PARAM_PARSER_H
