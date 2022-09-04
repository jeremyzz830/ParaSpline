#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "ParametricSpline.hpp"

void getInputData(Eigen::MatrixXd &InputData)
// Convert a string type input to a Eigen Matrix as
// [    x1  x2  x3  x4  x5
//      y1  y2  y3  y4  y5
//      z1  z2  z3  z4  z5]
{
    // InputData is a Vector for storing point xyz coodinates
    std::string InputString; // Input data can be instream or through TCP/UDP
    InputString = "10.5 12.0 15.0 1.0 0.5 ;0.5 0.5 0.5 0.5 0.5 ;1.5 10 5.5 6.0 0.5 ";
    int NumMarkers = 0; // calculate the total number of the markers, just for resizing the Eigen Vector
    for (int i = 0; i < InputString.length(); i++)
    {
        if (std::isspace(InputString[i])) // One space corresponds to a number
        {
            NumMarkers++;
        }
    }

    NumMarkers = NumMarkers / 3; // +0 based on the structure of our input string, here we have 5 spaces with 5 numbers
    InputData.resize(3, NumMarkers);

    // Break the input string as x y z segements based on ';' marks
    int start_idx, end_idx, num_idx;
    int row_idx = 0;
    std::string TempString;
    std::size_t BreakPoint1 = InputString.find(';');
    std::size_t BreakPoint2 = InputString.find(';', BreakPoint1 + 1);

    while (row_idx < 3)
    {
        TempString.clear();

        start_idx = 0;
        num_idx = 0;

        switch (row_idx)
        {
        case 0:
            TempString = InputString.substr(0, BreakPoint1);
            break;
        case 1:
            TempString = InputString.substr(BreakPoint1 + 1, BreakPoint2 - 1 - BreakPoint1);
            break;
        case 2:
            TempString = InputString.substr(BreakPoint2 + 1, InputString.length() - BreakPoint2);
            break;
        }
        // std::cout << TempString << std::endl;
        for (end_idx = 0; end_idx < TempString.length(); end_idx++)
        {
            // if ( std::isspace(TempString[end_idx]))
            if (TempString[end_idx] == ' ')
            {
                std::string temp = TempString.substr(start_idx, end_idx - start_idx); // subtract the piece containing the numbers

                start_idx = end_idx + 1; // store the start index for the next number
                // std::cout << temp << std::endl;
                InputData(row_idx, num_idx) = std::stod(temp);

                num_idx++;
            }
        }

        row_idx++;
    }
}

// int main(int argv, char **argc)
int main()
{

    std::cout << "Receiving Optical Marker Position INPUTS..." << std::endl;
    Eigen::MatrixXd InputData;
    getInputData(InputData);
    std::cout << "Point Matrix has been successfully created" << std::endl;
    // Calculate the coefficients of the cubic formula
    std::cout << "Points: \n"
              << InputData << std::endl;

    ParametricSpline paraline(InputData);

    paraline.setConstants(.02);
    // std::cout << "Good: \n";

    Eigen::Vector3d StartSlope = {1.0, 1.0, 1.0}, EndSlope = {1.0, 1.0, 1.0};

    paraline.setBoundary(StartSlope, EndSlope);

    // std::cout << "So far so good!\n";
    // paraline.getCoefficients();
    // auto line = paraline.C0_l;
    // std::cout << paraline.ContinuityCoeff.row(line) << std::endl;
    paraline.calcCoefficients();
    paraline.saveData();

    return 0;
}