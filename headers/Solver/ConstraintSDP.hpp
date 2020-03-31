#ifndef CONSTRAINT_SDP_HPP
#define CONSTRAINT_SDP_HPP

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include "Variable.hpp"
#include "SDPVariable.hpp"
#include "LPVariables.hpp"
#include "../MKCGraph.hpp"
#include "ConstraintGeneric.hpp"
#include <string>
#include <new>

class ConstraintSDP : public ConstraintGeneric<std::pair<const SDPVariable<Variable> *, const Variable *>>
{

private:

    std::map<const SDPVariable<Variable> *, std::vector<const Variable *>> map_sdp_var;


public:
    static ConstraintSDP *create()
    {
        return new ConstraintSDP(0.0, 0.0, EQUAL);
    }

    static ConstraintSDP *create(const double &lb,
                                 const double &ub,
                                 const ConstraintType &typ)
    {
        return new ConstraintSDP(lb, ub, typ);
    }

    ConstraintSDP(const double &lb,
                  const double &ub,
                  const ConstraintType &typ) : ConstraintGeneric(lb, ub, typ) {}


    void add_coefficient(const SDPVariable<Variable> * sdp_var, const Variable * var, const double &value)
    {
        map_sdp_var[sdp_var].push_back(var);

        add_coefficient(new std::pair<const SDPVariable<Variable> *, const Variable *> (sdp_var, var), value);
    }

/*    bool operator==(const ConstraintSDP &other) const
    {
        if (this->lowerBound == other.lowerBound &&
            this->upperBound == other.upperBound &&
            this->size() == other.size())
        {
            std::map<std::pair<const SDPVariable<Variable> *, const Variable *>, double>::iterator it;
            std::map<std::pair<const SDPVariable<Variable> *, const Variable *>, double>::iterator it_other =
                other.coefficients.begin();

            for (it = coefficients.begin(); it != coefficients.end(); ++it, ++it_other)
            {
                if (it_other->second != it->second || it_other->first != it->first)
                {
                    return false;
                }
            }

            return true;
        }

        return false;
    }*/

    ~ConstraintSDP()
    {
    }
};

#endif
