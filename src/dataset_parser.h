//
// Created by redwan on 4/3/22.
//

#ifndef VFRRT_DATASET_PARSER_H
#define VFRRT_DATASET_PARSER_H

#include <iostream>
#include <netcdf>
#include <vector>

using namespace std;
using namespace netCDF;

enum ROMS_VAR{
    UU,
    VV,
    LAT,
    LON
};


class dataset_parser {
public:
    dataset_parser(const char *filepath);
    /*
     * read netcdf variable from the filepath
     * see examples in https://github.com/Unidata/netcdf-cxx4
     */
    bool open();

    /*
     * retrieve depth layer wise velocity information
     * The water currents are represented using UU, VV variables
     */
    vector<vector<float>> retrieve_data(int depth_indx, const ROMS_VAR& type);

private:

    const char *filepath_;
    float latsIn_[25][25];
    float lonsIn_[25][25];
    float veloU_[12][25][25];
    float veloV_[12][25][25];

};


#endif //VFRRT_DATASET_PARSER_H
