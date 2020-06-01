#ifndef LP_VARIABLES1d_CONTAINER_HPP
#define LP_VARIABLES1d_CONTAINER_HPP

#include "Variable.hpp"
#include "Variables.hpp"
#include "../../Utils/Exception.hpp"
#include <utility> // std::pair, std::make_pair
#include <vector>
#include <map>
#include <string>
#include <typeinfo>

class LPVariables : public Variables<Variable>
{
private:
    LPVariables(){};

public:
    ~LPVariables(){};
    static LPVariables *create()
    {
        return new LPVariables();
    }
};

#endif
