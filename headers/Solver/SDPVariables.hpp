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

class SDPVariables : public Variables<SDPVariable>
{

public:
    SDPVariables(){};
    ~SDPVariables(){};

    // void set_solution_value(const int &idx, const double &val)
    // {
    //     validate_index(idx);
    //     variables[idx]->update_solution(val);
    // }
};

#endif
