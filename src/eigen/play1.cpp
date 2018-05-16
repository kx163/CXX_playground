#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

int main(int argc, char const *argv[])
{
    /* getting start - ex 1 */
    MatrixXd m(2, 2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    cout << m << endl;
    MatrixXd n(2,3);
    n << 1, 2, 3, 4, 5, 6;
    cout << n << endl;
    /* getting start - ex 2 */
    m = MatrixXd::Random(3, 3);
    m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;
    cout << "m = " << endl << m << endl;
    VectorXd v(3);
    v << 1, 2, 3;
    cout << "m * v = " << endl << m * v << endl;
    Matrix3d mm = Matrix3d::Random();
    mm = (mm + Matrix3d::Constant(1.2)) * 50;
    cout << "mm = " << endl << mm << endl;
    Vector3d vv(1, 2, 3);
    cout << "mm * vv = " << endl << mm * vv << endl;
}