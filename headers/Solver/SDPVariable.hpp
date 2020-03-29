#ifndef SDP_VARIABLE_SDP
#define SDP_VARIABLE_SDP

#include <string>
#include <vector>
#include <cstddef>
#include "../Utils/Exception.hpp"
#include "Variable.hpp"
#include "Variables.hpp"

template <typename V>
class SDPVariable : public Variables<V>
{
private:
    const int dimension;
    const int number_variables;
    const double constant_object_function;
    std::vector<int> row_index;
    std::vector<int> col_index;
    

    /**
     * @param i and j are positions in matrix (starts by zero)
     **/
    int calculate_position_variable(const int &i, const int &j) const
    {
        if (i >= dimension || j >= dimension || i < 0 || j < 0 || i == j)
        {
            std::string msg = "\n Invalid indices in SDPVariable,";
            msg += " for sdp variable of dimension" + std::to_string(this->dimension);
            msg += ". It has got (" + std::to_string(i) + "," + std::to_string(j) + ") \n";
            Exception(msg, ExceptionType::STOP_EXECUTION).execute();
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

    void add_new_indices(const int &vi, const int &vj)
    {
        if (vi < vj)
        {
            row_index.push_back(vi);
            col_index.push_back(vj);
        }
        else
        {
            row_index.push_back(vj);
            col_index.push_back(vi);
        }
    }

public:
    SDPVariable(const int &dim, double cost=0.0) : dimension(dim),
                                  number_variables(get_number_variables_for_matrix(dim)),
                                  constant_object_function(cost)
    {
    }

    V *add_variable(V *var)
    {
        std::string error = "Method add_variable(variable*) not implemented for SDP variables.\n";
        error += "Use add_variable(int, int, variable*)  ";
        Exception(error, ExceptionType::STOP_EXECUTION).execute();
    }

    V *add_variable(const int &vi, const int &vj, Variable *var)
    {
        int pos_index = calculate_position_variable(vi, vj);
        add_new_indices(vi, vj);    

        return this->add_variable_with_index(var, pos_index);
    }

    const V *get_variable(const int &vi, const int &vj)
    {
        int pos_index = calculate_position_variable(vi, vj);

        return Variables<Variable>::get_variable(pos_index);
    }

    const int get_dimension() const
    {
        return this->dimension;
    }

    const int get_number_non_null_variables() const
    {
        return this->row_index.size();
    }

    const int *get_row_indices() const 
    {
        return row_index.empty()? 0 : &this->row_index[0];
    }

    const int *get_col_indices() const 
    {
        return col_index.empty()? 0 : &this->col_index[0];
    }

    const double &get_constant_object_function() const
    {
        return this->constant_object_function;
    }

    ~SDPVariable() {}
};

#endif