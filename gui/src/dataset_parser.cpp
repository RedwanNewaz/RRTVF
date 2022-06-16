//
// Created by redwan on 4/3/22.
//

#include "dataset_parser.h"

dataset_parser::dataset_parser(ParamPtr params) : params(params) {
    filepath_ = params->get_dataset_path();
}

bool dataset_parser::open() {
    // Now read the data back in
    try {

        // Open the file for read access
        netCDF::NcFile dataFile(filepath_, netCDF::NcFile::read);
        cout <<"[dataset_parser]: data from " <<filepath_ << endl;
        // Store variables from nc file locally
        NcVar longitude, latitude, depth, U, V, time;
        longitude = dataFile.getVar("lon");
        latitude = dataFile.getVar("lat");
        depth = dataFile.getVar("depth");
        U = dataFile.getVar("u");
        V = dataFile.getVar("v");
        time = dataFile.getVar("time");

        // Store them in private variables of this class
        longitude.getVar(lonsIn_);
        latitude.getVar((latsIn_));

        vector<size_t> startp{0, 0, 0, 0},countp{1,12,25,25};
        U.getVar(startp,countp,veloU_);
        V.getVar(startp,countp,veloV_);

        auto err = longitude.isNull() || latitude.isNull() || depth.isNull() || U.isNull() || V.isNull() || time.isNull();
        return !err;

    } catch (netCDF::exceptions::NcException &e) {
        std::cout << e.what() << std::endl;
        int m, n, d;
        m = n = 25;
        d = params->get_depth_index();
        rapidcsv::Document doc1(filepath_ + "/" + params->get_uu_file() );
        auto result = read_csv_data(doc1, m, n);
        cout << result.size() << " x " << result[0].size() << endl;
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                veloU_[d][i][j] = result[i][j];
            }
        }


        rapidcsv::Document doc2(filepath_ + "/" + params->get_vv_file() );

        auto result2 = read_csv_data(doc2, m, n);
        cout << result2.size() << " x " << result2[0].size() << endl;
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                veloV_[d][i][j] = result2[i][j];
            }
        }

        return true;
    }
}

vector<vector<float>> dataset_parser::retrieve_data(int depth_indx, const ROMS_VAR &type) {
    vector<vector<float>> result;
    for (int i = 0; i < 25; ++i) {
        vector<float> temp;
        for (int j = 0; j < 25; ++j) {
            float value;
//                value = (type=="U")? veloU_[depth_indx][i][j] : veloV_[depth_indx][i][j];
            switch (type) {
                case UU:  value = veloU_[depth_indx][i][j]; break;
                case VV:  value = veloV_[depth_indx][i][j]; break;
                case LAT: value = latsIn_[i][j];break;
                case LON: value = lonsIn_[i][j];break;
            }
            temp.push_back(value);
        }
        result.push_back(temp);
    }
    return result;
}

DatasetPtr dataset_parser::get_ptr() {
    return shared_from_this();
}
