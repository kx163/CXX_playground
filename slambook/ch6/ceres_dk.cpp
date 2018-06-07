#include <iostream>
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <string>
#include <iterator>
#include <ceres/ceres.h>
using namespace std;


struct DK_COST
{
    DK_COST (double time, double distance, double obstacle) : _t(time), _d(distance), _n(obstacle) {}

    const double _t, _d, _n;

    template<typename T>
    bool operator() (const T* const params, T* residual) const {
        // params: T_D, T_A, d_D, d_A, v_R
        residual[0] = (params[0] + params[1] * T(_n) - T(_t)) * params[4] - (params[2] + params[3] * T(_n) - T(_d));
        return true;
    }
};


int main(int argc, char const *argv[])
{
    // read data from csv
    ifstream csv(argv[1]);
    vector<vector<double>> dataset1, dataset2;
    string line = "";
    
    while(getline(csv, line)){
        vector<string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(","));
        vector<double> res;
        int k = 0;
        for (auto i = 0; i < vec.size(); i++){
            if (i == 0)
            {
                k = atoi(vec[i].c_str());
            } 
            else
            {
                res.push_back(atof(vec[i].c_str()));
            }
        }
        if (k == 1) dataset1.push_back(res);
        if (k == 2) dataset2.push_back(res);
    };

    // test print out data
    cout << "data set from run #1:" << endl;
    for (auto const& v : dataset1) {
        for (auto const& d : v) {
            cout << d << ", ";
        }
        cout << endl;
    }
    cout << endl;
    cout << "data set from run #2:" << endl;
    for (auto const& v : dataset2) {
        for (auto const& d : v) {
            cout << d << ", ";
        }
        cout << endl;
    }
    cout << endl;

    // build problem
    double params[5] = {30.0, 10.0, 3.0, 2.0, 0.6};
    ceres::Problem problem;
    for (auto const& data : dataset1) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<DK_COST, 1, 5>(
                new DK_COST(data[0], data[1], data[2])
            ),
            nullptr,
            params
        );
    }
    for (auto const& data : dataset2) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<DK_COST, 1, 5>(
                new DK_COST(data[0], data[1], data[2])
            ),
            nullptr,
            params
        );
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.BriefReport() << endl;
    cout << "Estimated T_D, T_A, d_D, d_A, v_R: ";
    for (auto const& p : params) {
        cout << p << ", ";
    }
    cout << endl;

    double T_D = params[0], T_A = params[1], d_D = params[2], d_A = params[3], v_R = params[4];
    double T_R_1 = 0.0, T_R_2 = 0.0;

    for (auto const& data : dataset1) {
        T_R_1 += data[0] - T_D - T_A * data[2];
        T_R_1 += (data[1] - d_D - d_A * data[2]) / v_R;
    }
    T_R_1 /= dataset1.size() * 2;
    for (auto const& data : dataset2) {
        T_R_2 += data[0] - T_D - T_A * data[2];
        T_R_2 += (data[1] - d_D - d_A * data[2]) / v_R;
    }
    T_R_2 /= dataset2.size() * 2;

    double d_R_1 = T_R_1 * v_R, d_R_2 = T_R_2 * v_R;

    cout << "Estimated T_R_1, T_R_2: " << T_R_1 << ", " << T_R_2 << endl;
    cout << "Estimated d_R_1, d_R_2: " << d_R_1 << ", " << d_R_2 << endl;

    double lambda_1 = 0.6, lambda_2 = 0.95;
    double k = (lambda_1 / d_R_1 + lambda_2 / d_R_2) / 2;

    cout << "Estimated k, k^-1: " << k << ", " << 1 / k << endl;
    
    return 0;
}
