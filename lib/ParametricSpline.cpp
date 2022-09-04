#include <fstream>
#include <iomanip>
#include "ParametricSpline.hpp"

ParametricSpline::ParametricSpline(Eigen::MatrixXd &InputMatrix) : INTERVAL(0.01),
                                                                   InputData(InputMatrix.transpose())
{ // perform parametric cubic interpolation w.r.t. t within [t0,t1]
    point_idx = 1;

    num_points = InputData.rows();
    num_curves = num_points - 1;

    getContinuityCoeff();
    // std::string OutputString = "Testing";
    // printData(InputData, OutputString);
};

ParametricSpline::~ParametricSpline(){};

void ParametricSpline::calcCoefficients()
{
    // CtrlCondition * CoeffMtrix = Constraints
    // CoeffMatrix = pseudo_inverse(CtrlCondition) * Constraints

    printData(BoundaryCondition, "Boundary Condition: ");

    Constraints.resize(4 * num_curves, 3);

    // Set all the Constraint Conditions on the right hand side
    for (int i = 0; i < num_curves; i++)
    {
        if (i == 0)
        {
            // ConState = C0_l;
            Constraints.row(0) = BoundaryCondition.row(0);

            // ConState = C1_l;
            Constraints.row(1) = BoundaryCondition.row(1);

            // ConState = C0_r;
            Constraints.row(2) = BoundaryCondition.row(2);

            // ConState = C1_r;
            // ConState = C2_r;
            Constraints.block<2, 3>(3, 0) = Eigen::MatrixXd::Zero(2, 3);
        }
        else if (i == num_curves - 1)
        {
            // ConState = C0_l;
            Constraints.row(4 * i + 1) = BoundaryCondition.row(i + 1);
            // ConState = C0_r;
            Constraints.row(4 * i + 2) = BoundaryCondition.row(i + 2);
            // ConState = C1_r;
            Constraints.row(4 * i + 3) = BoundaryCondition.row(i + 3);
        }
        else
        {
            // std::cout << "So far so good!\n";
            // ConState = C0_l;
            Constraints.row(4 * i + 1) = BoundaryCondition.row(i + 1);

            // ConState = C0_r;
            Constraints.row(4 * i + 2) = BoundaryCondition.row(i + 2);

            // ConState = C1_r;
            // ConState = C2_r;
            Constraints.block<2, 3>(4 * i + 3, 0) = Eigen::MatrixXd::Zero(2, 3);
        }
    }

    // std::cout << "So far so good!\n";
    // Set all the CtrlCondition Matrix
    CtrlCondition.resize(4 * num_curves, 4 * num_curves);

    CtrlCondition.block<5, 4>(0, 0) << ContinuityCoeff.row(C0_l),
        ContinuityCoeff.row(C1_l),
        ContinuityCoeff.row(C0_r),
        ContinuityCoeff.row(C1_r),
        ContinuityCoeff.row(C2_r);

    CtrlCondition.block<5, 4>(CtrlCondition.rows() - 5, CtrlCondition.cols() - 4) << -ContinuityCoeff.row(C1_l),
        -ContinuityCoeff.row(C2_l),
        ContinuityCoeff.row(C0_l),
        ContinuityCoeff.row(C0_r),
        ContinuityCoeff.row(C1_r);

    for (int i = 1; i < num_curves - 1; i++)
    {
        CtrlCondition.block<6, 4>(4 * i - 1, 4 * i) << -ContinuityCoeff.row(C1_l),
            -ContinuityCoeff.row(C2_l),
            ContinuityCoeff.row(C0_l),
            ContinuityCoeff.row(C0_r),
            ContinuityCoeff.row(C1_r),
            ContinuityCoeff.row(C2_r);
    }

    Result = CtrlCondition.completeOrthogonalDecomposition().pseudoInverse() * Constraints;
    printData(Constraints, "Constraints: ");
    printData(CtrlCondition, "Control Condition Matrix: ");

    printData(Result, "Result: ");
};

void ParametricSpline::printData(Eigen::MatrixXd &InputMatrix, std::string OutputString)
{
    std::cout << OutputString << std::endl;
    std::cout << InputMatrix << std::endl;
};

void ParametricSpline::setConstants(double Modifier)
{
    const double INTERVAL = Modifier;
    std::cout << "Interval has been modified to: " << INTERVAL
              << std::endl;
};

void ParametricSpline::setBoundary(Eigen::Vector3d &StartSlope, Eigen::Vector3d &EndSlope)
{

    // std::cout << "Concatenating...\n";

    BoundaryCondition.resize(InputData.rows() + 2, InputData.cols());
    int k = 0;
    for (int i = 0; i < InputData.rows(); i++)
    {
        if (i == 1)
        {
            BoundaryCondition.row(i + k) = StartSlope;
            k++;
        }
        else if (i == InputData.rows() - 1)
        {
            BoundaryCondition.row(i + k + 1) = EndSlope;
        }

        BoundaryCondition.row(i + k) = InputData.row(i);
    }
};

void ParametricSpline::saveData()
{
    std::ofstream outfile;
    outfile.open("/home/jeremy/ParaSpline/data/CoefficientResult.txt");

    int Width = 15;
    outfile << std::setfill(' ')
            << std::setw(Width)
            << "X"
            << std::setw(Width)
            << "Y"
            << std::setw(Width)
            << "Z"
            << std::endl;

    for (int i = 0; i < 4; i++)
    {
        switch (i)
        {
        case 0:
            outfile << "a:" << std::endl;
            break;
        case 1:
            outfile << "b:" << std::endl;
            break;
        case 2:
            outfile << "c:" << std::endl;
            break;
        case 3:
            outfile << "d:" << std::endl;
            break;
        }
        for (int j = 0; j < Result.rows() / 4; j++)
        {
            outfile << std::setfill(' ')
                    << std::setw(Width)
                    << Result(i + 4 * j, 0) << ","
                    << std::setw(Width)
                    << Result(i + 4 * j, 1)
                    << std::setw(Width)
                    << Result(i + 4 * j, 2)
                    << ";"
                    << std::endl;
        }
    }

    outfile.close();
}

void ParametricSpline::showCurve3D(){

};

void ParametricSpline::getContinuityCoeff(double t0, double t1)
{
    ContinuityCoeff.resize(6, 4);
    ContinuityCoeff.row(0) << 1, pow(t0, 1), pow(t0, 2), pow(t0, 3); // C0 left = 0
    ContinuityCoeff.row(1) << 1, pow(t1, 1), pow(t1, 2), pow(t1, 3); // C0 right = 1
    ContinuityCoeff.row(2) << 0, 1, 2 * t0, 3 * t0 * t0;             // C1 left = 2
    ContinuityCoeff.row(3) << 0, 1, 2 * t1, 3 * t1 * t1;             // C1 right = 3
    ContinuityCoeff.row(4) << 0, 0, 2, 6 * t0;                       // C2 left = 4
    ContinuityCoeff.row(5) << 0, 0, 2, 6 * t1;                       // C2 right = 5
};