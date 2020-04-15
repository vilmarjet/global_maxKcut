#ifndef CPA_VIOLATED_CONSTRAINTS_HPP
#define CPA_VIOLATED_CONSTRAINTS_HPP

#include "ViolatedConstraint.hpp"
#include "ViolatedConstraints.hpp"
#include "../Solver/Abstract/Solver.hpp"
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

class ProcessorViolatedConstraints
{
protected:
    std::set<ViolatedConstraint *, CompViolatedConstraint> violated_constraints;
    const int max_number_inequalities;
    Solver *solver;

public:
    const static int DEFAULT_NUMBER_MAX_VIOLATIONS = 100;
    ProcessorViolatedConstraints(const int &nb, Solver *solver_) : max_number_inequalities(nb),
                                                                   solver(solver_) {}
    ~ProcessorViolatedConstraints() {}

    virtual ProcessorViolatedConstraints *populate() = 0;
    virtual ProcessorViolatedConstraints *find_violation(ViolatedConstraints **inequalities_type, const size_t &size) = 0;
    virtual ProcessorViolatedConstraints *find_violation(ViolatedConstraints *inequalities_type) = 0;

    ViolatedConstraint *add_violated_constraint(ViolatedConstraint *constraint)
    {
        std::set<ViolatedConstraint *, CompViolatedConstraint>::iterator it =
            violated_constraints.insert(constraint).first;
        return *it;
    }

    ProcessorViolatedConstraints *add_violated_constraints(ViolatedConstraint **constraint, const int &size)
    {
        for (int i = 0; i < size; ++i)
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