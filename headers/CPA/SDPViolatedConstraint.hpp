#ifndef MKC_SDP_VIOLATED_CONSTRAINT_HPP
#define MKC_SDP_VIOLATED_CONSTRAINT_HPP

#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <new>
#include "../Solver/Variable/Variable.hpp"
#include "../Solver/Constraint/ConstraintSDP.hpp"
#include "ViolatedConstraint.hpp"

class SDPViolatedConstraint : public ViolatedConstraint
{

public:
    static SDPViolatedConstraint *create(const double &lb,
                                         const double &ub,
                                         const ConstraintBoundKey &typ,
                                         const double &vio)
    {
        return new SDPViolatedConstraint(lb, ub, typ, vio);
    }

    static SDPViolatedConstraint *create(const double &vio,
                                         ConstraintSDP *constraintSDP)
    {
        return new SDPViolatedConstraint(vio, constraintSDP);
    }

private:
    SDPViolatedConstraint(const double &lb,
                          const double &ub,
                          const ConstraintBoundKey &typ,
                          const double &vio) : ViolatedConstraint(vio,
                                                                  ConstraintSDP::create(lb, ub, typ))
    {
    }

    SDPViolatedConstraint(const double &vio,
                          ConstraintSDP *constraintSDP) : ViolatedConstraint(vio, constraintSDP)
    {
    }

public:
    SDPViolatedConstraint *add_coefficient(const SDPVariable<Variable> *sdp_var,
                                           const Variable *var,
                                           const double &coeff)
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

    ConstraintSDP *get_constraint() const
    {
        return (ConstraintSDP *)(this->constraint);
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

#endif
