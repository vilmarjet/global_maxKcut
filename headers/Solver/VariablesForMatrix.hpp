#ifndef VARIABLE_SYMMETRIC_MATRIX
#define VARIABLE_SYMMETRIC_MATRIX

#include <string>
#include <vector>
#include <cstddef>
#include "../Utils/Exception.hpp"
#include "Variable.hpp"

class VariableSymmetricMatrix
{
private:
    int dimension;
    int number_variables;
    VariableType type = VariableType::SDP;
    std::vector<Variable *> variables;

    /**
     * @param i and j are positions in matrix (starts by zero)
     **/
    int calculate_position_variable(const int &i, const int &j) const
    {
        if (i >= dimension || j >= dimension || i < 0 || j < 0)
        {
            std::string msg = "\nIndices larger than dimension in ";
            msg += "VariableSymmetricMatrix::add_variable() or not possible.";
            msg += ". It has got (" + std::to_string(i) + "," + std::to_string(j) + ") \n";
            throw Exception(msg, ExceptionType::STOP_EXECUTION);
        }

        int vi = i,
            vj = j;
        if (vi > vj)
        {
            std::swap(vi, vj);
        }

        int max_vi = get_number_variables_for_matrix(vi);
        return ((((vj) + (vi) * (this->dimension)) - max_vi));
    }

    /**
     * @param dim is dimension of matrix
     * */
    int get_number_variables_for_matrix(const int &dim) const
    {
        return (int)(((dim - 1) * dim) / 2.0) + dim;
    }

public:
    VariableSymmetricMatrix(const int &dim) : dimension(dim)
    {
        this->variables.clear();
        this->number_variables = get_number_variables_for_matrix(dim);
        this->variables.resize(number_variables, nullptr);
    }

    bool add_variable(const int &vi, const int &vj, const Variable &var)
    {
        int pos_index = calculate_position_variable(vi, vj);

        this->variables[pos_index] = new Variable(pos_index, var);
    }

    Variable *get_variable(const int &vi, const int &vj)
    {
        int pos_index = calculate_position_variable(vi, vj);
        return this->variables[pos_index];
    }

    ~VariableSymmetricMatrix() {}
};

#endif