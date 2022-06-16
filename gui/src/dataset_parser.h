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
    LON
};

class dataset_parser;
typedef shared_ptr<dataset_parser> DatasetPtr;

class dataset_parser: public enable_shared_from_this<dataset_parser>{
public:
    dataset_parser(ParamPtr params);
    /*
     * read netcdf variable from the filepath
     * see examples in https://github.com/Unidata/netcdf-cxx4
     */
    bool open();

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
    float veloU_[12][25][25];
    float veloV_[12][25][25];


protected:
    vector<vector<double>> read_csv_data(const rapidcsv::Document& doc, int m, int n)
    {
        vector<vector<double>> result;
        for(int i = 0; i < m; ++i)
        {
            std::vector<double> myvector = doc.GetRow<double>(i);
            myvector.erase(myvector.begin() + n, myvector.end());
            result.emplace_back(myvector);
        }
        return result;
    }
};


#endif //VFRRT_DATASET_PARSER_H
