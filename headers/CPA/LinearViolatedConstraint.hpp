#ifndef VIOLATED_CONSTRAINT_HPP
#define VIOLATED_CONSTRAINT_HPP

#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <new>
#include "../Solver/Variable/Variable.hpp"
#include "../Solver/Constraint/ConstraintType.hpp"
#include "../Solver/Constraint/LinearConstraint.hpp"

class LinearViolatedConstraint
{
private:
    const double violation; //use for cutting plane algorithm
    LinearConstraint *constraintLinear;
    LinearViolatedConstraint(const double &lb,
                             const double &ub,
                             const ConstraintType &typ,
                             const double &vio,
                             const int &size,
                             const Variable **vars,
                             const double *coef) : constraintLinear(LinearConstraint::create(lb, ub, typ)),
                                                   violation(vio)
    {
        if (size > 0)
        {
            this->add_coefficients(vars, coef, size);
        }
    }

public:
    static LinearViolatedConstraint *create(const double &lb,
                                            const double &ub,
                                            const ConstraintType &typ,
                                            const double &vio)
    {
        return create(lb, ub, typ, vio, 0, nullptr, nullptr);
    }

    static LinearViolatedConstraint *create(const double &lb,
                                            const double &ub,
                                            const ConstraintType &typ,
                                            const double &vio,
                                            const int &size,
                                            const Variable **vars,
                                            const double *coef)
    {
        return new LinearViolatedConstraint(lb, ub, typ, vio, size, vars, coef);
    }

    bool operator<(const LinearViolatedConstraint &rhs) const
    {
        return violation >= rhs.violation; // inversed (larger to smaller)
    }

    const LinearConstraint *get_constraint() const
    {
        return this->constraintLinear;
    }

    ConstraintCoefficient<Variable> *add_coefficient(const Variable *var, const double &coeff)
    {
        return constraintLinear->add_coefficient(var, coeff);
    }

    LinearConstraint *add_coefficients(const Variable *const *vars, const double *coeffs, const int size)
    {
        return constraintLinear->add_coefficients(vars, coeffs, size);
    }

    double get_violation() const
    {
        return this->violation;
    }

    std::string to_string() const
    {
        std::string strg;
        strg = this->constraintLinear->to_string();
        strg += "; vioil=" + std::to_string(this->violation);

        return strg;
    }

    bool operator==(const LinearViolatedConstraint &other) const
    {
        return this->violation == other.violation &&
               *constraintLinear == *(other.constraintLinear);
    }

    ~LinearViolatedConstraint()
    {
        delete constraintLinear;
    }
};

#endif
