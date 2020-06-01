#ifndef Linear_VIOLATED_CONSTRAINTs_HPP
#define Linear_VIOLATED_CONSTRAINTs_HPP

#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <new>
#include <set>
#include "./CPA/LinearViolatedConstraint.hpp"
#include "./CPA/ProcessorViolatedConstraints.hpp"
#include "./CPA/ViolatedConstraints.hpp"
#include "MKCInstance.hpp"
namespace maxkcut
{
class ProcessorLinearViolatedConstraints : public ProcessorViolatedConstraints
{
public:
    static ProcessorLinearViolatedConstraints *create()
    {
        return create(DEFAULT_NUMBER_MAX_VIOLATIONS, nullptr);
    }

    static ProcessorLinearViolatedConstraints *create(const int &nb,
                                                      Solver *solver_)
    {
        return new ProcessorLinearViolatedConstraints(nb, solver_);
    }

    static ProcessorLinearViolatedConstraints *create(Solver *solver_)
    {
        return create(DEFAULT_NUMBER_MAX_VIOLATIONS, solver_);
    }

private:
    ProcessorLinearViolatedConstraints(const int &nb,
                                       Solver *solver_) : ProcessorViolatedConstraints(nb, solver_)

    {
    }

public:
    ProcessorLinearViolatedConstraints *find_violation(ViolatedConstraints **inequalities_type, const size_t &size)
    {
        for (size_t i = 0; i < size; ++i)
        {
            find_violation(inequalities_type[i]);
        }

        return this;
    }

    ProcessorLinearViolatedConstraints *find_violation(ViolatedConstraints *inequality)
    {
        inequality->find_violated_constraints();

        for (auto violated_constraint : inequality->get_constraints())
        {
            add_violated_constraint(violated_constraint);
        }

        inequality->reset();

        return this;
    }

    ProcessorLinearViolatedConstraints *populate()
    {
        int counter_ineq = 0;
        for (auto constraint : violated_constraints)
        {
            if (constraint->get_constraint()->get_type_constraint() != ConstraintType::LINEAR)
            {
                continue;
            }

            solver->add_constraint_linear(LinearConstraint::from((LinearConstraint *)constraint->get_constraint()));

            if (++counter_ineq > get_max_number_inequalities())
            {
                break;
            }
        }

        return this;
    }

    ~ProcessorLinearViolatedConstraints()
    {
        for (auto p : violated_constraints)
        {
            delete p;
        }
    }

    void clear()
    {
        violated_constraints.clear();
    }
};
} // namespace maxkcut

#endif
