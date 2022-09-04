#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

class ParametricSpline
{
public:
    const double INTERVAL; // Interval constants for interpolation
    int num_points;
    int num_curves;
    int point_idx;
    enum ContinuousState { C0_l, C0_r, C1_l, C1_r, C2_l, C2_r };

    Eigen::MatrixXd InputData; // Input point positions as a 3xN matrix
    Eigen::MatrixXd Result;
    Eigen::MatrixXd CtrlCondition;
    Eigen::MatrixXd ContinuityCoeff;
    Eigen::MatrixXd Constraints;
    Eigen::MatrixXd BoundaryCondition;

    ParametricSpline(Eigen::MatrixXd &InputMatrix);
    ~ParametricSpline();

    void printData(Eigen::MatrixXd &InputMatrix, std::string OutputString = "Default Output");

    void setConstants(double Modifier);
    void setBoundary(Eigen::Vector3d& StartSlope, Eigen::Vector3d& EndSlope);

    void calcCoefficients();
    void showCurve3D();
    void saveData();

    void getContinuityCoeff(double t0 = 0, double t1 = 1);
};