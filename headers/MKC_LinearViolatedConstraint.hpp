#ifndef MKC_Linear_VIOLATED_CONSTRAINT_HPP
#define MKC_Linear_VIOLATED_CONSTRAINT_HPP

#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <new>
#include "./Solver/Variable/Variable.hpp"
#include "./Solver/Constraint/LinearConstraint.hpp"
#include "./CPA/ViolatedConstraint.hpp"

namespace maxkcut
{
class LinearViolatedConstraint : public ViolatedConstraint
{
private:
    LinearViolatedConstraint(const double &lb,
                             const double &ub,
                             const ConstraintBoundKey &typ,
                             const double &vio,
                             const int &size,
                             const Variable **vars,
                             const double *coef) : ViolatedConstraint(vio,
                                                                      LinearConstraint::create(lb, ub, typ))
    {
        if (size > 0)
        {
            this->add_coefficients(vars, coef, size);
        }
    }

    LinearViolatedConstraint (const double &viol,
    LinearConstraint* linear): ViolatedConstraint(viol,LinearConstraint::from(linear)){}


public:
    static LinearViolatedConstraint *create(const double &lb,
                                            const double &ub,
                                            const ConstraintBoundKey &typ,
                                            const double &vio)
    {
        return create(lb, ub, typ, vio, 0, nullptr, nullptr);
    }

    static LinearViolatedConstraint *create(const double &lb,
                                            const double &ub,
                                            const ConstraintBoundKey &typ,
                                            const double &vio,
                                            const int &size,
                                            const Variable **vars,
                                            const double *coef)
    {
        return new LinearViolatedConstraint(lb, ub, typ, vio, size, vars, coef);
    }

    static LinearViolatedConstraint *from (LinearViolatedConstraint *violated)
    {
        return new LinearViolatedConstraint(violated->violation, (LinearConstraint*) (violated->get_constraint()));
    }

    ConstraintCoefficient<Variable> *add_coefficient(const Variable *var, const double &coeff)
    {
        return ((LinearConstraint *)constraint)->add_coefficient(var, coeff);
    }

    LinearConstraint *add_coefficients(const Variable *const *vars, const double *coeffs, const int size)
    {
        return ((LinearConstraint *)constraint)->add_coefficients(vars, coeffs, size);
    }

    std::string to_string() const
    {
        std::string strg;
        strg = ((LinearConstraint *)constraint)->to_string();
        strg += "; vioil=" + std::to_string(this->violation);

        return strg;
    }

    LinearConstraint *get_constraint() const
    {
        return (LinearConstraint *)constraint;
    }

    bool operator==(const LinearViolatedConstraint &other) const
    {
        return this->violation == other.violation &&
               *((LinearConstraint *)constraint) == *((LinearConstraint *)other.constraint);
    }

    ~LinearViolatedConstraint()
    {
        delete constraint;
    }
};
} // namespace maxkcut

#endif
