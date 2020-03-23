#ifndef LP_VARIABLES1d_CONTAINER_HPP
#define LP_VARIABLES1d_CONTAINER_HPP

#include "Variable.hpp"
#include "Variables.hpp"
#include "DimensionVariable.hpp"
#include "../Utils/Exception.hpp"
#include <utility> // std::pair, std::make_pair
#include <vector>
#include <map>
#include <string>
#include <typeinfo>

class LPVariables : public Variables<Variable>
{

public:
    LPVariables(){};
    ~LPVariables(){};

    void set_solution_value(const int &idx, const double &val)
    {
        validate_index(idx);
        variables[idx]->update_solution(val);
    }
};

#endif
