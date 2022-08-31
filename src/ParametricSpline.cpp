#include <ParametricSpline.hpp>

ParametricSpline::ParametricSpline(Eigen::MatrixXd& InputData): INTERVAL(0.01),
                                                                NUM_POINTS(InputData.cols())
{
    point_idx = 1;

    std::string OutputString = "Testing";
    printData(InputData, OutputString);

};

ParametricSpline::~ParametricSpline(){};

void ParametricSpline::printData(Eigen::MatrixXd& InputMatrix, std::string OutputString){

    std::cout << OutputString << std::endl;
    std::cout << InputMatrix << std::endl;
};

void ParametricSpline::changeConstants(double Modifier){
    const double INTERVAL = Modifier;
};

void ParametricSpline::showCurve3D(){

};

void ParametricSpline::getCoefficients(Eigen::MatrixXd& CoeffResult, int point_idx,  std::string ContinuousState, int t0 = 0, int t1 = 1){

};