#include <iostream>
#include <Eigen/Core>
#include "Eigen/Dense"
#include <math.h>

using namespace std;
int main()
{
    double fx = 160/tan(1.085/2);
    double d=0.115;
    Eigen::Matrix<double, 3, 3> K;
    Eigen::Matrix<double, 3, 3> K_I;
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 3> Homography;
    Eigen::Matrix<double, 3, 1> t;
    Eigen::Matrix<double, 3, 1> n;
    K<<     265, 0,  160,
            0,  265, 120,
            0,  0,  1;

    R<<     1, 0,  0,
            0, 0,  1,
            0, -1, 0;

    t << 0, 1, 1;

    n << 0, -1, 0;

    Homography = K * (R.transpose() - t*n.transpose()/d) * K.inverse();

    cout<<Homography<<endl;
    return 0;
}

