#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>  // include Geometry relevant libraries


int main(int argc, char const *argv[])
{
    // declare the geometry matrix
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));  // this is not based on Matrix, but could mix as by operator override

    cout .precision(3);
    cout << "rotation matrix is:\n" << rotation_vector.matrix() << endl;  // conver to matrix
    rotation_matrix = rotation_vector.toRotationMatrix();  // or assign to a matrix

    Eigen::Vector3d v (1, 0 ,0);  // initiate a vector
    Eigen::Vector3d v_rotated = rotation_vector * v;  // rotate vector by direct multiplication
    cout << "(1, 0, 0) after rotation: " << v_rotated.transpose() << endl;
    v_rotated = rotation_matrix * v;  // rotate by rotation matrix
    cout << "(1, 0, 0) after rotation: " << v_rotated.transpose() << endl;

    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  // ZYX rder, i.e. yaw, pitch, roll
    cout << "yaw, pitch, roll = " << euler_angles.transpose() << endl;

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  // isometry transform matrix for 3D is in fact 4x4
    T.rotate(rotation_vector);  // set how it rotates
    T.pretranslate(Eigen::Vector3d(1, 3, 4));  // set how it pretranslates
    cout << "Transform matrix = \n" << T.matrix() << endl;

    Eigen::Vector3d v_transformed = T * v;  // i.e. R*v+t
    cout << "v transformed: " << v_transformed.transpose() << endl;

    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);  // Quaternion could be initialised by rotation vector
    cout << "quaternion = \n" << q.coeffs() << endl;  // the last field is the real field
    q = Eigen::Quaterniond(rotation_matrix);  // Quaternion could be initialised by rotation matrix as well
    cout << "quaternion = \n" << q.coeffs() << endl;
    v_rotated = q * v;  // In math, it is in fact q*v*q^(-1)
    cout << "(1, 0, 0) after rotation: " << v_rotated.transpose() << endl;

    // Little Robot exercice
    Eigen::Quaterniond q_1 = Eigen::Quaterniond(0.35, 0.2, 0.3, 0.1);
    Eigen::Vector3d t_1 (0.3, 0.1, 0.1);
    Eigen::Isometry3d Tcw_1 = Eigen::Isometry3d::Identity();
    Tcw_1.rotate(q_1);
    Tcw_1.pretranslate(t_1);
    cout << "Little Robot 1's T_cw: " << endl << Tcw_1.matrix() << endl;
    Eigen::Quaterniond q_2 = Eigen::Quaterniond(-0.5, 0.4, -0.1, 0.2);
    Eigen::Vector3d t_2 (-0.1, 0.5, 0.3);
    Eigen::Isometry3d Tcw_2 = Eigen::Isometry3d::Identity();
    Tcw_2.rotate(q_2);
    Tcw_2.pretranslate(t_2);
    cout << "Little Robot 2's T_cw: " << endl << Tcw_2.matrix() << endl;
    Eigen::Vector3d p_1 (0.5, 0, 0.2);
    Eigen::Vector3d p_2 = Tcw_2.inverse() * Tcw_1 * p_1;
    cout << "Little Robot 1 sees P as: " << p_1.transpose() << endl;
    cout << "Little robot 2 sees P as: " << p_2.transpose() << endl;
    
    return 0;
}
