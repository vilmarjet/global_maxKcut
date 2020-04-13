#ifndef Linear_VIOLATED_CONSTRAINTs_HPP
#define Linear_VIOLATED_CONSTRAINTs_HPP

#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <new>
#include <set>
#include "MKC_LinearViolatedConstraint.hpp"
#include "./CPA/ViolatedConstraints.hpp"
#include "MKC_Inequalities.hpp"
#include "MKCInstance.hpp"
namespace maxkcut
{
class LinearViolatedConstraints : public ProcessViolatedConstraints
{
public:
    static LinearViolatedConstraints *create()
    {
        return create(DEFAULT_NUMBER_MAX_VIOLATIONS, nullptr, nullptr);
    }

    static LinearViolatedConstraints *create(const int &nb,
                                             Solver *solver_,
                                             std::vector<MKC_Inequalities *> *types)
    {
        return new LinearViolatedConstraints(nb, solver_, types);
    }

    static LinearViolatedConstraints *create(Solver *solver_,
                                             std::vector<MKC_Inequalities *> *types)
    {
        return create(DEFAULT_NUMBER_MAX_VIOLATIONS, solver_, types);
    }

private:
    Solver *solver;
    const std::vector<MKC_Inequalities *> *inequalities_type;

    LinearViolatedConstraints(const int &nb,
                              Solver *solver_,
                              std::vector<MKC_Inequalities *> *types) : ProcessViolatedConstraints(nb),
                                                                         solver(solver_),
                                                                         inequalities_type(types)

    {
    }

public:
    LinearViolatedConstraints *find()
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

    LinearViolatedConstraints *populate()
    {
        int counter_ineq = 0;
        for (auto constraint : violated_constraints)
        {
            solver->add_constraint_linear(LinearConstraint::from((LinearConstraint *)(constraint->get_constraint())));

            if (++counter_ineq > get_max_number_inequalities())
            {
                break;
            }
        }

        return this;
    }

    ~LinearViolatedConstraints()
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
