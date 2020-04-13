#ifndef CPA_VIOLATED_CONSTRAINTS_HPP
#define CPA_VIOLATED_CONSTRAINTS_HPP

#include "ViolatedConstraint.hpp"
#include <set>

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

class ProcessViolatedConstraints
{
protected:
    std::set<ViolatedConstraint *, CompViolatedConstraint> violated_constraints;
    const int max_number_inequalities;

public:
    const static int DEFAULT_NUMBER_MAX_VIOLATIONS = 100;
    ProcessViolatedConstraints(const int &nb) : max_number_inequalities(nb) {}
    ~ProcessViolatedConstraints() {}

    virtual ProcessViolatedConstraints* populate() = 0;
    virtual ProcessViolatedConstraints* find() = 0;

    ViolatedConstraint *add_violated_constraint(ViolatedConstraint *constraint)
    {
        std::set<ViolatedConstraint *, CompViolatedConstraint>::iterator it =
            violated_constraints.insert(constraint).first;
        return *it;
    }

    ProcessViolatedConstraints *add_violated_constraints (ViolatedConstraint **constraint, const int &size)
    {
        for (int i=0; i<size; ++i)
        {
            add_violated_constraint(constraint[i]);
        }

        return this;
    }

    int get_number_violated_constraints() const
    {
        return violated_constraints.size();
    }

    const int &get_max_number_inequalities() const
    {
        return max_number_inequalities;
    }
};

#endif