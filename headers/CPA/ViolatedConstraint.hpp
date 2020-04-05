#ifndef VIOLATED_CONSTRAINT_HPP
#define VIOLATED_CONSTRAINT_HPP

#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <new>
#include "../Solver/Variable.hpp"
#include "../Solver/Constraint.hpp"
#include "../Solver/ConstraintLinear.hpp"

class ViolatedConstraint
{
private:
    const double violation; //use for cutting plane algorithm
    const Constraint *constraint;
    ConstraintLinear *constraintLinear;

public:
    static ViolatedConstraint *create(const double &lb,
                                      const double &ub,
                                      const ConstraintType &typ,
                                      const double &vio)
    {
        return create(lb, ub, typ, vio, 0, nullptr, nullptr);
    }

    static ViolatedConstraint *create(const double &lb,
                                      const double &ub,
                                      const ConstraintType &typ,
                                      const double &vio,
                                      const int &size,
                                      const Variable **vars,
                                      const double *coef)
    {
        return new ViolatedConstraint(lb, ub, typ, vio, size, vars, coef);
    }

    ViolatedConstraint(const double &lb,
                       const double &ub,
                       const ConstraintType &typ,
                       const double &vio,
                       const int &size,
                       const Variable **vars,
                       const double *coef) : constraint(new Constraint()),
                                             constraintLinear(ConstraintLinear::create(lb, ub, typ)),
                                             violation(vio)
    {
        this->add_coefficients(vars, coef, size);
    }

    ViolatedConstraint() : violation(0.0)
    {
    }

    ViolatedConstraint(const std::vector<const Variable *> &vars,
                       const std::vector<double> &coef,
                       const double &lb,
                       const double &ub,
                       const ConstraintType &typ,
                       const double &vio) : constraint(new Constraint(vars, coef, lb, ub, typ)),
                                            violation(vio) {}
    ViolatedConstraint(const Variable **vars,
                       double *coef,
                       const int &size,
                       const double &lb,
                       const double &ub,
                       const ConstraintType &typ,
                       const double &vio) : constraint(new Constraint(vars, coef, size, lb, ub, typ)),
                                            violation(vio) {}

    bool operator<(const ViolatedConstraint &rhs) const
    {
        return violation >= rhs.violation; // inversed (larger to smaller)
    }

    const Constraint *get_constraint() const
    {
        return this->constraint;
    }

    void add_coefficient(const Variable *var, const double &coeff)
    {
        constraintLinear->add_coefficient(var, coeff);
    }

    void add_coefficients(const Variable *const *vars, const double *coeffs, const int size)
    {
        constraintLinear->add_coefficients(vars, coeffs, size);
    }

    double get_violation() const
    {
        return this->violation;
    }

    std::string to_string() const
    {
        std::string strg;
        strg = this->constraint->to_string();
        strg += "; vioil=" + std::to_string(this->violation);

        return strg;
    }

    bool operator==(const ViolatedConstraint &other) const
    {
        return this->violation == other.violation &&
               constraint == other.constraint;
    }

    ~ViolatedConstraint()
    {
        delete constraint;
    }
};

struct CompViolatedConstraint
{
    double ZERO = 1e-6;
    bool operator()(const ViolatedConstraint *lhs, const ViolatedConstraint *rhs)
    {
        if (std::abs(lhs->get_violation() - rhs->get_violation()) <= ZERO)
        {
            return lhs->get_constraint()->size() <= rhs->get_constraint()->size();
        }

        return lhs->get_violation() >= rhs->get_violation();
    }
};

#endif
