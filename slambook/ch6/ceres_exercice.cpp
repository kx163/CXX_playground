#include <iostream>
#include <random>
#include <cmath>
#include <ceres/ceres.h>
using namespace std;

/*
    f(x,y) = a * (1-x)^2 + b * (y-x^2)^2
*/
struct RosenbrockCostFunctor
{
    template <typename T>
    bool operator() (const T* const ab, T* residuals) const
    {
        residuals[0] = T(_f) - (ab[0] * ceres::pow(T(1.0) - T(_x), 2) + ab[1] * ceres::pow(T(_y) - T(_x) * T(_x), 2));
        return true;
    }

    RosenbrockCostFunctor(double x, double y, double f): _x(x), _y(y), _f(f) {}

    protected:
    double _x;
    double _y;
    double _f;
};


int main(int argc, char const *argv[])
{
    // generate the data
    int N = 100;  // number of sample on 1 dimension
    double w_sigma = 10.0;  // sigma of the noise
    default_random_engine generator;  // default random generator
    normal_distribution<double> distribution(0.0, w_sigma);  // gaussian distribution

    // setup the problem
    ceres::Problem problem;
    double ab[2] = {0, 0};
    for(int i = 0; i < N; i++)
    {
        double x = double(i - N) / double(N);
        for(int j = 0; j < N; j++)
        {
            double y = double(j - N) / double(N);
            double f = (1.0 - x)*(1.0 - x) + 100.0 * (y - x*x)*(y - x*x);
            ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<RosenbrockCostFunctor, 1, 2>(new RosenbrockCostFunctor(x, y, f));
            problem.AddResidualBlock(cost_function, nullptr, ab);
        }
    }

    // solve the optimisation problem and print the report
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;
    cout << "a = " << ab[0] << ", b = " << ab[1] << endl;
    return 0;
}
