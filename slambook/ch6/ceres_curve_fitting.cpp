#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
using namespace std;

// cost function model

struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST (double x, double y) : _x(x), _y(y) {}
    // calculation of residual
    template <typename T>
    // abc is the model parameter, 3D
    bool operator() (const T* const abc, T* residual) const
    {
        // y-exp(ax^2+bx+c)
        residual[0] = T(_y) - ceres::exp(abc[0]*T(_x)*T(_x)+abc[1]*T(_x)+abc[2]);
        return true;
    }
    const double _x, _y;  // x, y data
};


int main(int argc, char const *argv[])
{
    double a = 1.0, b = 2.0, c = 1.0;  // true parameter values
    int N = 100;  // number of data
    double w_sigma = 1.0;  // sigma of noise
    cv::RNG rng;  // OpenCV random number generator
    double abc[3] = {0, 0, 0};  // estimated value of model parameter

    vector<double> x_data, y_data;  // data

    cout << "generating data ..." << endl;
    
    for(int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a*x*x+b*x+c)+rng.gaussian(w_sigma));
        cout << x_data[i] << " " << y_data[i] << endl;
    }

    // build up the least square problem
    ceres::Problem problem;
    for(int i = 0; i < N; i++)
    {
        problem.AddResidualBlock(
            // using the auto difference cost function
            // template of the function: residual type, output dimension, input dimension
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST( x_data[i], y_data[i] )
            ),  // adding residual into the problem
            nullptr,  // kernel function, we do not need it here
            abc  // parameters to estimate
        );
    }
    
    // configurate th solver
    ceres::Solver::Options options;  // many config options to fill
    options.linear_solver_type = ceres::DENSE_QR;  // how to solve the incremental equation
    options.minimizer_progress_to_stdout = true;  // output to cout

    ceres::Solver::Summary summary;  // optimisation info
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // start the optimisation
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds." << endl;

    // output result
    cout << summary.BriefReport() << endl;
    cout << "estimated a, b, c: ";
    for ( auto a : abc) cout << a << " ";
    cout << endl;
    
    return 0;
}
