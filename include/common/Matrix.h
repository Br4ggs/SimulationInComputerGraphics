#pragma once

#include "linearSolver.h"
#include <vector>

class Matrix : public implicitMatrix
{
public:
    Matrix(unsigned int rows, unsigned int columns);
    Matrix() = delete;

    //https://stackoverflow.com/questions/33642050/overloading-subscript-operators-for-specific-read-and-write-operations
	std::vector<float>& operator[](const int index);				//for setting values
	const std::vector<float>& operator[](const int index) const;	//for getting values

    Matrix operator*(const Matrix& mat) const;
    std::vector<float> operator*(const std::vector<float>& vec) const; //vector is treated as row vector

    void matVecMult(double x[], double r[]) override;
    Matrix transpose() const;

    unsigned int nrColumns() const;

private:
    unsigned int rows;
    unsigned int columns;

    // vectors are used to allow construction during initialization
    std::vector<std::vector<float>> components;
};