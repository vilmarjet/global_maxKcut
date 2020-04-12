#ifndef MKC_INEQUALITIESSS_HPP
#define MKC_INEQUALITIESSS_HPP


#include "./Solver/Abstract/Solver.hpp"
#include "./MKCGraph.hpp"
#include "./MKCInstance.hpp"
#include "./VariablesEdge.hpp"
#include "MKC_LinearViolatedConstraint.hpp"
#include <vector>

namespace maxkcut
{
class MKC_Inequalities
{
protected:
    double rhs;
    std::vector<LinearViolatedConstraint*> violated_constraints;

public:
    MKC_Inequalities(const double &_rhs) : rhs(_rhs) {}
    virtual void find_violated_constraints(const VariablesEdge *Variables,
                                           const MKCInstance *instance) = 0;
    virtual std::string to_string() = 0;
    
    const std::vector<LinearViolatedConstraint*> &get_constraints() const
    {
        return  violated_constraints;
    }

    LinearViolatedConstraint *add_violated_constraint(LinearViolatedConstraint * constraint)
    {
        violated_constraints.push_back(constraint);
        return violated_constraints[get_number_constraints() - 1];
    }

    int get_number_constraints () const
    {
        return violated_constraints.size();
    }

    void reset()
    {
        violated_constraints.clear();
    }

};
} // namespace maxkcut
#endif
