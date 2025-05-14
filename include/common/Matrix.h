#pragma once

#include <vector>

class Matrix
{
public:
    Matrix(unsigned int rows, unsigned int columns);
    Matrix() = delete;

    //https://stackoverflow.com/questions/33642050/overloading-subscript-operators-for-specific-read-and-write-operations
	std::vector<float>& operator[](const int index);				//for setting values
	const std::vector<float>& operator[](const int index) const;	//for getting values

private:
    unsigned int rows;
    unsigned int columns;

    // vectors are used to allow construction during initialization
    std::vector<std::vector<float>> components;
};