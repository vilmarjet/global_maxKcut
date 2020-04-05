#ifndef SDP_VARIABLE_SDP
#define SDP_VARIABLE_SDP

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <cstddef>
#include "../Utils/Exception.hpp"
#include "Variable.hpp"
#include "Variables.hpp"
#include "../Utils/Exception.hpp"

template <typename V>
class SDPVariable : public Variables<V>
{
private:
    const int dimension;
    const int number_variables;
    const double constant_object_function;
    std::vector<int> row_index;
    std::vector<int> col_index;

    std::map<const V *, std::pair<int, int>> variables_by_row_col;

    /**
     * @param i and j are positions in matrix (starts by zero)
     **/
    int calculate_position_variable(const int &i, const int &j) const
    {
        if (i >= dimension || j >= dimension || i < 0 || j < 0)
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

        int max_vi = get_number_variables_for_lower_matrix(vi);
        return ((((vj) + (vi) * (this->dimension)) - max_vi));
    }

    /**
     * @param dim is dimension of matrix
     * */
    int get_number_variables_for_lower_matrix(const int &dim) const
    {
        return (int)(((dim - 1) * dim) / 2.0) + dim;
    }

    void add_new_indices(const int &vi, const int &vj, const V *var)
    {
        int row = vi;
        int col = vj;

        if (row > col)
        {
            std::swap(row, col);
        }

        row_index.push_back(row);
        col_index.push_back(col);

        variables_by_row_col
            .insert(std::pair<const V *, std::pair<int, int>>(var, std::pair<int, int>(row, col)));
    }

public:
    SDPVariable(const int &dim, double cost = 0.0) : dimension(dim),
                                                     number_variables(get_number_variables_for_lower_matrix(dim)),
                                                     constant_object_function(cost)
    {
        int number_variables = get_number_variables_for_lower_matrix(dim);
        this->variables.resize(number_variables);

        for (int row = 0; row < this->dimension; ++row)
        {
            for (int col = row; col < this->dimension; ++col)
            {
                int pos = calculate_position_variable(row, col);
                this->variables[pos] = Variable::create();
                const V *var = this->variables[pos];
                variables_by_row_col
                    .insert(std::pair<const V *, std::pair<int, int>>(var, std::pair<int, int>(row, col)));
            }
        }
    }

    V *add_variable(V *var)
    {
        std::string error = "Method add_variable(variable*) not implemented for SDP variables.\n";
        error += "Use add_variable(int, int, variable*)  ";
        Exception(error, ExceptionType::STOP_EXECUTION).execute();
    }

    V *add_variable(const int &vi, const int &vj, V *var)
    {
        int pos_index = calculate_position_variable(vi, vj);

        V *var_created = this->add_variable_with_index(var, pos_index);

        add_new_indices(vi, vj, var_created);

        return var_created;
    }

    const V *get_variable(const int &vi, const int &vj) const
    {
        int pos_index = calculate_position_variable(vi, vj);

        return Variables<V>::get_variable(pos_index);
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
        return row_index.empty() ? 0 : &this->row_index[0];
    }

    const int *get_col_indices() const
    {
        return col_index.empty() ? 0 : &this->col_index[0];
    }

    const double &get_constant_object_function() const
    {
        return this->constant_object_function;
    }

    int get_col_index(const V *var) const
    {
        int idx = get_row_col_of_variables(var).second;
        return idx;
    }

    int get_row_index(const V *var) const
    {
        int idx = get_row_col_of_variables(var).first;
        return idx;
    }

    const std::pair<int, int> &get_row_col_of_variables(const V *var) const
    {
        typename std::map<const V *, std::pair<int, int>>::const_iterator it;

        it = variables_by_row_col.find(var);

        if (it == variables_by_row_col.end())
        {
            Exception("Object does not exist in SDPVariable::get_row_col_of_variables",
                            ExceptionType::STOP_EXECUTION). execute();
        }

        return it->second;
    }

    std::string to_string() const
    {
        return "SDP Variable";
    }

    ~SDPVariable() {}
};

#endif