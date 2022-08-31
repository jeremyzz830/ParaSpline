#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

class ParametricSpline
{
    private:

        Eigen::MatrixXd InputData; // Input point positions as a 3xN matrix
        const double INTERVAL; // Interval constants for interpolation
        const int NUM_POINTS;  // Total Points received
        int point_idx;

    public:

        ParametricSpline(Eigen::MatrixXd& InputData);
        ~ParametricSpline();

        void changeConstants(double Modifier);
        void printData(Eigen::MatrixXd& InputMatrix, std::string OutputString);
        void showCurve3D();
        void getCoefficients(Eigen::MatrixXd& CoeffResult, int point_idx,  std::string ContinuousState, int t0 = 0, int t1 = 1);

};