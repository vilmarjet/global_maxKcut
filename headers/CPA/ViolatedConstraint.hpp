#ifndef CPA_VIOLATED_CONSTRAINT_HPP
#define CPA_VIOLATED_CONSTRAINT_HPP

#include "../Solver/Constraint/ConstraintAbstract.hpp"

class ViolatedConstraint
{
protected:
    double violation;
    ConstraintAbstract *constraint;

public:
    ViolatedConstraint(const double &viol, ConstraintAbstract *constraint_) : violation(viol),
                                                                              constraint(constraint_) {}
    ~ViolatedConstraint() 
    {
        delete constraint;
    }

    const double &get_violation() const
    {
        return this->violation;
    }

    ConstraintAbstract *get_constraint() const
    {
        return this->constraint;
    }

    bool operator<(const ViolatedConstraint &rhs) const
    {
        return violation >= rhs.violation; // inversed (larger to smaller)
    }
};

#endif