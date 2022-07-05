//
// Created by redwan on 4/3/22.
//

#ifndef VFRRT_DATASET_PARSER_H
#define VFRRT_DATASET_PARSER_H

#include <iostream>
#include <netcdf>
#include <vector>
#include <memory>
#include "param_parser.h"
#include "rapidcsv.h"
using namespace std;
using namespace netCDF;

enum ROMS_VAR{
    UU,
    VV,
    LAT,
    LON,
    gUU,
    gVV,
};

class dataset_parser;
typedef shared_ptr<dataset_parser> DatasetPtr;

class dataset_parser: public enable_shared_from_this<dataset_parser>{
public:
    using VELOCITIES = vector<vector<double>>;
    dataset_parser(ParamPtr params);
    /*
     * read netcdf variable from the filepath
     * see examples in https://github.com/Unidata/netcdf-cxx4
     */
    bool open(bool nc_file = false);

    DatasetPtr get_ptr();

    /*
     * retrieve depth layer wise velocity information
     * The water currents are represented using UU, VV variables
     */
    vector<vector<float>> retrieve_data(int depth_indx, const ROMS_VAR& type);

    ParamPtr params;

private:

    string filepath_;
    float latsIn_[25][25];
    float lonsIn_[25][25];
    VELOCITIES  veloU_, gveloU_;
    VELOCITIES  veloV_, gveloV_;


protected:
    bool parse_csv_files();
    bool parse_nc_file();
    vector<vector<double>> read_csv_data(const rapidcsv::Document& doc, int m, int n);
};


#endif //VFRRT_DATASET_PARSER_H
