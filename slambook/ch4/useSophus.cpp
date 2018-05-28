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

    // verify left hand disturbation model
    Eigen::Vector3d p (1, 1, 1);  // define the initial point
    Eigen::Vector3d p_direct = Sophus::SO3::exp(update_so3) * (SO3_R * p);  // calculate final point by direct computation
    cout << "p after rotation by direct computation: " << p_direct.transpose() << endl;
    Eigen::Vector3d p_disturb = SO3_R * p + (- Sophus::SO3::hat(SO3_R * p)) * update_so3;  // calculate final point by disturbation model
    cout << "p after rotation by disturbation model: " << p_disturb.transpose() << endl;

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

    // verify left hand disturbation model
    p_direct = Sophus::SE3::exp(update_se3) * (SE3_Rt * p);  // calculate final point by direct computation
    cout << "p after transformation by direct computation: " << p_direct.transpose() << endl;
    Eigen::Matrix<double, 4, 6> derivative;  // declare the derivative matrix
    derivative.setZero();
    derivative.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    derivative.block<3, 3>(0, 3) = - Sophus::SO3::hat(SE3_Rt * p);
    p_disturb = (SE3_Rt * p) + (derivative * update_se3).block<3, 1>(0, 0);  // calculate final point by disturbation model
    cout << "p after transformation by disturbation model: " << p_disturb.transpose() << endl;

    return 0;
}
