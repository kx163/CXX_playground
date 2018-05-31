#include <iostream>
#include <cmath>
#include <random>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
using namespace std;

/*
    f(x,y) = a * (1 - x) ^ 2 + b * (y - x ^ 2) ^ 2
*/
// class RosenbrockVertex : public g2o::BaseVertex<2, double*>
class RosenbrockVertex : public g2o::BaseVertex<2, Eigen::Vector2d>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl()
    {
        // _estimate[0] = 0;
        // _estimate[1] = 1;
        _estimate << 0, 0;
    }

    virtual void oplusImpl(const double* update)
    {
        // _estimate[0] += update[0];
        // _estimate[1] += update[1];
        _estimate += Eigen::Vector2d(update);
    }

    virtual bool read(istream& in) {}
    virtual bool write(ostream& out) const {}
};

class RosenbrockUnaryEdge : public g2o::BaseUnaryEdge<1, double, RosenbrockVertex>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RosenbrockUnaryEdge(double x, double y) : BaseUnaryEdge(), _x(x), _y(y) {}
    
    void computeError()
    {
        // double* ab = static_cast<const RosenbrockVertex*>(_vertices[0])->estimate();
        // _error(0, 0) = _measurement - (ab[0] * pow((1.0 - _x), 2) + ab[1] * pow(_y - _x * _x, 2));
        Eigen::Vector2d ab = static_cast<const RosenbrockVertex*>(_vertices[0])->estimate();
        _error(0, 0) = _measurement - (ab(0, 0) * pow((1.0 - _x), 2) + ab(1, 0) * pow(_y - _x * _x, 2));
    }

    virtual bool read(istream& in) {}
    virtual bool write(ostream& out) const {}

    private:
    double _x;
    double _y;
};


int main(int argc, char const *argv[])
{
    // setup variable
    int N = 100;
    double w_sigma = 10.0;
    double ab[2] = {1, 100};
    default_random_engine generator;
    normal_distribution<double> distribution(0.0, w_sigma);

    // build up the problem
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<2, 1>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // generating data
    RosenbrockVertex* v = new RosenbrockVertex();
    // v->setEstimate(new double[2] {0, 0});
    v->setEstimate(Eigen::Vector2d(0, 0));
    v->setId(0);
    optimizer.addVertex(v);
    for(int i = 0; i < N; i++)
    {
        double x = double(i - N) / double(N);
        for(int j = 0; j < N; j++)
        {
            double y = double(j - N) / double(N);
            double f = ab[0]*pow(1-x,2) + ab[1]*pow(y-x*x,2) + distribution(generator);
            RosenbrockUnaryEdge* edge = new RosenbrockUnaryEdge(x, y);
            edge->setId(i*N+j);
            edge->setVertex(0, v);
            edge->setMeasurement(f);
            edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1.0 / (w_sigma * w_sigma));
            optimizer.addEdge(edge);
        }
    }

    // execute the optimisation problem
    cout << "start optimistion" << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    // double* estimate = v->estimate();
    // cout << "a = " << estimate[0] << ", b = " << estimate[1] << endl;
    Eigen::Vector2d estimate = v->estimate();
    cout << estimate.transpose() << endl;
    return 0;
}
