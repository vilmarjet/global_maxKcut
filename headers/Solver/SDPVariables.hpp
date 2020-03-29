#ifndef VARIABLES1d_CONTAINER_HPP
#define VARIABLES1d_CONTAINER_HPP

#include "SDPVariable.hpp"
#include "Variables.hpp"
#include "DimensionVariable.hpp"
#include "../Utils/Exception.hpp"
#include <utility> // std::pair, std::make_pair
#include <vector>
#include <map>
#include <string>
#include <typeinfo>

class SDPVariables : public Variables<SDPVariable<Variable>>
{
private:
    std::vector<double> cost_index; //cache

public:
    SDPVariables(){};
    ~SDPVariables(){};

    const double *get_cost_indices(const int &idx)
    {
        validate_index(idx);

        if (cost_index.empty())
        {
            const int* col = variables[idx]->get_col_indices();
            const int* row = variables[idx]->get_row_indices();
            int n_zero = variables[idx]->get_number_non_null_variables();

            cost_index.resize(n_zero);

            for (int i = 0; i < n_zero; ++i)
            {
                cost_index[i] = variables[idx]->get_variable(row[i], col[i])->get_cost();
            }
        }

        return cost_index.empty() ? 0 : &cost_index[0];
    }

    // void set_solution_value(const int &idx, const double &val)
    // {
    //     validate_index(idx);
    //     variables[idx]->update_solution(val);
    // }
};

#endif
