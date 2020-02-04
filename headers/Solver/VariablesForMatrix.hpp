#ifndef VARIABLES_CONTAINER_MATRIX
#define VARIABLES_CONTAINER_MATRIX

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
    std::vector<Variable *> variables;

    /**
     * @info vi must be inferior than vj
     * @param vi and vj are positions in matrix (starts by zero)
     **/
    int calculate_position_variable(const int &i, const int &j) const
    {
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
    ~VariableSymmetricMatrix() {}

    bool add_variable(const int &vi, const int &vj, const Variable &var)
    {
        if (vi => this->dimension || vj => this->dimension)
        {
            std::string msg = "\nIndices larger than dimension in VariableSymmetricMatrix::add_variable() or not possible.";
            msg += "Got (" + std::to_string(vi) + "," + std::to_string(vj) + ") \n";
            throw Exception(msg, ExceptionType::STOP_EXECUTION);
        }

        int pos_index = calculate_position_variable(vi, vj);

        this->variables[pos_index] = new Variable(pos_index, var);
    }

    Variable *get_variable(const int &vi, const int &vj)
    {
        int pos_index = calculate_position_variable(vi, vj);
        return this->variables[pos_index];
    }

};

#endif