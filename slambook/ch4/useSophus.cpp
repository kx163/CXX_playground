#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>
#include <sophus/se3.h>


int main(int argc, char const *argv[])
{
    /* SO3 section */
    // rotation matrix for rotatinng 90 degrees around z axis
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

    // different ways to define SO3
    Sophus::SO3 SO3_R (R);  // declare from rotation matrix
    Sophus::SO3 SO3_v (0, 0, M_PI / 2);  // declare from rotation vector
    Eigen::Quaterniond q(R);
    Sophus::SO3 SO3_Q (q);  // declare from quaternion

    // output via standard output
    cout << "SO3 from matrix: " << SO3_R << endl;
    cout << "SO3 from vector: " << SO3_v << endl;
    cout << "SO3 from quaternion: " << SO3_Q << endl;

    // the coupled Lie Algebra
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    cout << "so3 hat = " << Sophus::SO3::hat(so3) << endl;
    cout << "SO3 hat vee = " << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;

    // update for small disturbation
    Eigen::Vector3d update_so3 (1e-4, 0, 0);  // a small disturbation on so3
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R;
    cout << "Updated SO3 = " << SO3_updated << endl;

    cout << "*************************" << endl;

    /* SE3 section */
    Eigen::Vector3d t (1, 0, 0);  // define the translate vector
    Sophus::SE3 SE3_Rt (R, t);  // declare from rotation matrix and translate vector
    Sophus::SE3 SE3_Qt (q, t);  // declare from quaternion and translate vector
    cout << "SE3 from R, t: " << endl << SE3_Rt << endl;
    cout << "SE3 from Q, t: " << endl << SE3_Qt << endl;

    // the coupled Lie Algebra
    typedef Eigen::Matrix<double, 6, 1> Vector6d;  // to ease the SE3 vector, which is 6D
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;
    cout << "se3 hat = " << Sophus::SE3::hat(se3) << endl;
    cout << "se3 hat vee = " << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << endl;

    // update from small disturbation
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;
    cout << "Updated SE3 = " << SE3_updated << endl;
    return 0;
}
