#ifndef SDP_VARIABLE_SDP
#define SDP_VARIABLE_SDP

#include <string>
#include <vector>
#include <cstddef>
#include "../Utils/Exception.hpp"
#include "Variable.hpp"

class SDPVariable
{
private:
    const int dimension;
    std::vector<Variable *> variables;
    const int number_variables;

    /**
     * @param i and j are positions in matrix (starts by zero)
     **/
    int calculate_position_variable(const int &i, const int &j) const
    {
        if (i >= dimension || j >= dimension || i < 0 || j < 0)
        {
            std::string msg = "\nIndices larger than dimension in ";
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
    SDPVariable(const int &dim) : dimension(dim),
    number_variables(get_number_variables_for_matrix(dim))
    {
        this->variables.clear();
        this->variables.resize(this->number_variables, nullptr);
    }

    const Variable* add_variable(const int &vi, const int &vj, Variable *var)
    {
        int pos_index = calculate_position_variable(vi, vj);
        this->variables[pos_index] = var;
        return var;
    }

    const Variable *get_variable(const int &vi, const int &vj)
    {
        int pos_index = calculate_position_variable(vi, vj);
        return this->variables[pos_index];
    }

    ~SDPVariable() {}
};

#endif