#ifndef Linear_VIOLATED_CONSTRAINTs_HPP
#define Linear_VIOLATED_CONSTRAINTs_HPP

#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <new>
#include <set>
#include "LinearViolatedConstraint.hpp"

struct CompViolatedConstraint
{
    double ZERO = 1e-6;
    bool operator()(const LinearViolatedConstraint *lhs, const LinearViolatedConstraint *rhs)
    {
        if (std::abs(lhs->get_violation() - rhs->get_violation()) <= ZERO)
        {
            return lhs->get_constraint()->size() <= rhs->get_constraint()->size();
        }

        return lhs->get_violation() >= rhs->get_violation();
    }
};

class LinearViolatedConstraints
{
private:
    std::set<LinearViolatedConstraint *, CompViolatedConstraint> violated_constraints;
    const int user_max_number_inequalities;
    Solver *solver;

    LinearViolatedConstraints(const int &nb, Solver *solver_) : user_max_number_inequalities(nb),
                                                                     solver(solver_) {}

public:
    const static int DEFAULT_NUMBER_MAX_VIOLATIONS = 100;

    static LinearViolatedConstraints *create()
    {
        return create(DEFAULT_NUMBER_MAX_VIOLATIONS, nullptr);
    }

    static LinearViolatedConstraints *create(const int &number_ineq, Solver *solver)
    {
        return new LinearViolatedConstraints(number_ineq, solver);
    }

    LinearViolatedConstraint *add_violated_constraint(LinearViolatedConstraint *constraint)
    {
        std::set<LinearViolatedConstraint *, CompViolatedConstraint>::iterator it =
            violated_constraints.insert(constraint).first;
        return *it;
    }

    int apply_constraints()
    {
        int counter_ineq = 0;
        for (std::set<LinearViolatedConstraint *, CompViolatedConstraint>::iterator it = violated_constraints.begin();
             it != violated_constraints.end() && counter_ineq < user_max_number_inequalities;
             ++it, ++counter_ineq)
        {
            solver->add_constraint((*it)->get_constraint());
        }

        return counter_ineq;
    }

    ~LinearViolatedConstraints()
    {
    }

    void clear()
    {
        violated_constraints.clear();
    }
};

#endif
