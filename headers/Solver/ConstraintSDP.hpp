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
#include "ConstraintCoefficient.hpp"
#include "ConstraintGeneric.hpp"
#include <string>
#include <new>

class ConstraintSDP : public ConstraintGeneric
{
private:
    std::map<const SDPVariable<Variable> *, std::vector<ConstraintCoefficient<Variable>>> map_sdp_var;

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

    void add_coefficient(const SDPVariable<Variable> *sdp_var, const Variable *var, const double &coeff)
    {
        map_sdp_var[sdp_var].push_back(ConstraintCoefficient<Variable>::create(var, coeff));
    }

    const std::map<const SDPVariable<Variable> *, std::vector<ConstraintCoefficient<Variable>>> &get_variables()
    {
        return map_sdp_var;
    }

    const size_t size()
    {
        return map_sdp_var.size();
    }

    ~ConstraintSDP()
    {
    }
};

#endif
