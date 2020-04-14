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
        return create(DEFAULT_NUMBER_MAX_VIOLATIONS, nullptr, nullptr);
    }

    static ProcessorLinearViolatedConstraints *create(const int &nb,
                                             Solver *solver_,
                                             std::vector<ViolatedConstraints *> *types)
    {
        return new ProcessorLinearViolatedConstraints(nb, solver_, types);
    }

    static ProcessorLinearViolatedConstraints *create(Solver *solver_,
                                             std::vector<ViolatedConstraints *> *types)
    {
        return create(DEFAULT_NUMBER_MAX_VIOLATIONS, solver_, types);
    }

private:
    Solver *solver;
    const std::vector<ViolatedConstraints *> *inequalities_type;

    ProcessorLinearViolatedConstraints(const int &nb,
                              Solver *solver_,
                              std::vector<ViolatedConstraints *> *types) : ProcessorViolatedConstraints(nb),
                                                                        solver(solver_),
                                                                        inequalities_type(types)

    {
    }

public:
    ProcessorLinearViolatedConstraints *find()
    {
        for (auto inequality : *inequalities_type)
        {
            inequality->find_violated_constraints();
            for (auto violated_constraint : inequality->get_constraints())
            {
                add_violated_constraint(violated_constraint);
            }

            inequality->reset();
        }

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
