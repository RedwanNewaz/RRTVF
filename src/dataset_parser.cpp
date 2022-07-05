//
// Created by redwan on 4/3/22.
//

#include "dataset_parser.h"

dataset_parser::dataset_parser(ParamPtr params) : params(params) {
    filepath_ = params->get_dataset_path();
}

bool dataset_parser::open(bool nc_file) {
    // Now read the data back in

    return (nc_file) ? parse_nc_file() : parse_csv_files();
}

vector<vector<float>> dataset_parser::retrieve_data(int depth_indx, const ROMS_VAR &type) {
    vector<vector<float>> result;
    for (int i = 0; i < params->get_max_index(); ++i) {
        vector<float> temp;
        for (int j = 0; j < params->get_max_index(); ++j) {
            float value;
//                value = (type=="U")? veloU_[depth_indx][i][j] : veloV_[depth_indx][i][j];
            switch (type) {
                case UU:  value = veloU_[i][j]; break;
                case VV:  value = veloV_[i][j]; break;
                case gUU:  value = gveloU_[i][j]; break;
                case gVV:  value = gveloV_[i][j]; break;
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

bool dataset_parser::parse_csv_files() {
    int m, n;
    m = n = params->get_max_index();
    rapidcsv::Document doc1(filepath_ + "/" + params->get_uu_file() );
    veloU_ = read_csv_data(doc1, m, n);
    rapidcsv::Document doc2(filepath_ + "/" + params->get_vv_file() );
    veloV_ = read_csv_data(doc2, m, n);

    rapidcsv::Document docgu(filepath_ + "/" + "uu.csv" );
    gveloU_ = read_csv_data(docgu, m, n);
    rapidcsv::Document docgv(filepath_ + "/" + "vv.csv" );
    gveloV_ = read_csv_data(docgv, m, n);

//    cout << veloU_.size() << " x " << veloU_[0].size() << endl;
//    cout << veloV_.size() << " x " << veloV_[0].size() << endl;
//    cout << gveloU_.size() << " x " << gveloU_[0].size() << endl;
//    cout << gveloV_.size() << " x " << gveloV_[0].size() << endl;
    return true;
}



vector<vector<double>> dataset_parser::read_csv_data(const rapidcsv::Document& doc, int m, int n)
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

bool dataset_parser::parse_nc_file() {
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
        float veloU[12][25][25], veloV[12][25][25];
        U.getVar(startp,countp,veloU);
        V.getVar(startp,countp,veloV);
        int d = params->get_depth_index();
        veloU_.resize(25);
        veloV_.resize(25);
        for (int i = 0; i < 25; ++i){
            veloU_[i].resize(25);
            veloV_[i].resize(25);
            for (int j = 0; j < 25; ++j) {
                veloU_[i][j] = veloU[d][i][j];
                veloV_[i][j] = veloV[d][i][j];
            }
        }

        auto err = longitude.isNull() || latitude.isNull() || depth.isNull() || U.isNull() || V.isNull() || time.isNull();
        return !err;

    } catch (netCDF::exceptions::NcException &e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
}
