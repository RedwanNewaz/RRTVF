//
// Created by redwan on 4/4/22.
//

#include "param_parser.h"
#include <map>

param_parser::param_parser(const char *filename) {
    config_ = YAML::LoadFile(filename);

}

ParamPtr param_parser::get_ptr() {
    return shared_from_this();
}

vector<param_parser::OBS> param_parser::get_obstacle_list() {
    vector<OBS>result;
    auto obstacles = config_["Obstacles"].as<map<string, map<string, vector<float>>>>();
    for(auto & obstacle : obstacles)
    {
        auto x = obstacle.second["x"];
        auto y = obstacle.second["y"];
        result.emplace_back(make_pair(x, y));
    }
    return result;
}

ostream &operator<<(ostream &os, const param_parser &parser) {
    os << "[param_parse] config: \n" << parser.config_;
    return os;
}

double param_parser::get_solve_time() {
    return config_["OMPL"]["solve_time"].as<double>();
}

double param_parser::get_exploration_const() {
    return config_["VFRRT"]["exploration"].as<double>();
}

double param_parser::get_initial_lambda() {
    return config_["VFRRT"]["initial_lambda"].as<double>();
}

int param_parser::get_update_freq() {
    return config_["VFRRT"]["update_freq"].as<int>();
}

vector<float> param_parser::get_start_loc() {
    return config_["StartLoc"].as<vector<float>>();
}

vector<float> param_parser::get_goal_loca() {
    return config_["GoalLoc"].as<vector<float>>();
}

string param_parser::get_dataset_path() {
    return config_["Dataset"]["filepath"].as<string>();
}

int param_parser::get_depth_index() {
    return config_["Dataset"]["depth"].as<int>();
}

int param_parser::get_min_index() {
    return config_["Dataset"]["Index"]["low"].as<int>();
}

int param_parser::get_max_index() {
    return config_["Dataset"]["Index"]["high"].as<int>();
}

string param_parser::get_uu_file() {
    return config_["Dataset"]["uu"].as<string>();
}

string param_parser::get_vv_file() {
    return config_["Dataset"]["vv"].as<string>();
}
