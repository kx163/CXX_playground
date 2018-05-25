#include <iostream>
#include <ctime>
using namespace std;

// Eigen include
#include <Eigen/Core>
#include <Eigen/Dense>  // include the dense matrix calculation libraries

#define MATRIX_SIZE 50

/* examples of how to use Eigen library */

int main(int argc, char const *argv[])
{
    // different kind of data type
    Eigen::Matrix<float, 2, 3> matrix_23;  // the base type
    Eigen::Vector3d v_3d;  // actually Matrix<double, 3, 1>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();  // actually Matrix<double, 3, 3> and initialiation
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;  // dynamic size matrix
    Eigen::MatrixXd matrix_x;  // simple way to declare dynamic size matrix

    // examples for matrix operations
    matrix_23 << 1, 2, 3, 4, 5, 6;  // input via standard input
    cout << matrix_23 << endl;  // output via standard input

    // access matrix elements via loop

    for(int i = 0; i < 1; i++)
    {
        
        for(int j = 0; j < 2; j++)
        {
            cout << matrix_23(i, j) << endl;
        }
        
    }

    // matrix multiplication override the operator *
    v_3d << 3, 2, 1;  // vector initialisation via standard input
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d; // need to explictly cast between types
    cout << result << endl;

    // matrix calculation
    matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl << endl;

    cout << matrix_33.transpose() << endl;
    cout << matrix_33.sum() << endl;
    cout << matrix_33.trace() << endl;
    cout << 10 * matrix_33 << endl;
    cout << matrix_33.inverse() << endl;
    cout << matrix_33.determinant() << endl;

    // eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver (matrix_33.transpose() * matrix_33);
    cout << "Eigen values: " << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors: " << eigen_solver.eigenvectors() << endl;

    // sovle equation matrix_NN * x = v_Nd
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::VectorXd::Random(MATRIX_SIZE);

    clock_t time_start = clock();  // start time counter
    // solve equation by normal inverse
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "Time used in normal inverse is " << 1000 * (clock() - time_start) / (double) CLOCKS_PER_SEC << "ms" << endl;
    // sovle equation by QR decomposition
    time_start = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "Time used in QR decomposition is " << 1000 * (clock() - time_start) / (double) CLOCKS_PER_SEC << "ms" << endl;
    
    return 0;
}
