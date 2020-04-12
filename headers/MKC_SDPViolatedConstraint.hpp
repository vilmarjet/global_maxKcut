#ifndef MKC_SDP_VIOLATED_CONSTRAINT_HPP
#define MKC_SDP_VIOLATED_CONSTRAINT_HPP

#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <new>
#include "./Solver/Variable/Variable.hpp"
#include "./Solver/Constraint/ConstraintType.hpp"
#include "./Solver/Constraint/ConstraintSDP.hpp"
#include "./CPA/ViolatedConstraint.hpp"

namespace maxkcut
{
class SDPViolatedConstraint : public ViolatedConstraint
{

public:
    static SDPViolatedConstraint *create(const double &lb,
                                         const double &ub,
                                         const ConstraintType &typ,
                                         const double &vio)
    {
        return new SDPViolatedConstraint(lb, ub, typ, vio);
    }

private:
    SDPViolatedConstraint(const double &lb,
                          const double &ub,
                          const ConstraintType &typ,
                          const double &vio) : ViolatedConstraint(vio,
                                                                  ConstraintSDP::create(lb, ub, typ))
    {
    }

public:
    SDPViolatedConstraint *add_coefficient(const SDPVariable<Variable> *sdp_var, const Variable *var, const double &coeff)
    {
        ((ConstraintSDP *)constraint)->add_coefficient(sdp_var, var, coeff);
        return this;
    }

    std::string to_string() const
    {
        std::string strg;
        strg = ((LinearConstraint *)constraint)->to_string();
        strg += "; vioil=" + std::to_string(this->violation);

        return strg;
    }

    bool operator==(const SDPViolatedConstraint &other) const
    {
        return this->violation == other.violation &&
               *((ConstraintSDP *)constraint) == *((ConstraintSDP *)other.constraint);
    }

    ~SDPViolatedConstraint()
    {
        delete constraint;
    }
};
} // namespace maxkcut

#endif
