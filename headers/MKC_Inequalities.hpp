#ifndef MKC_INEQUALITIESSS_HPP
#define MKC_INEQUALITIESSS_HPP


#include "./Solver/Abstract/Solver.hpp"
#include "./MKCGraph.hpp"
#include "./MKCInstance.hpp"
#include "./VariablesEdge.hpp"
#include "CPA/ViolatedConstraint.hpp"
#include <vector>

namespace maxkcut
{
class MKC_Inequalities
{
protected:
    std::vector<ViolatedConstraint*> violated_constraints;

public:
    MKC_Inequalities() {}

    virtual void find_violated_constraints() = 0;
    virtual std::string to_string() = 0;
    
    const std::vector<ViolatedConstraint*> &get_constraints() const
    {
        return  violated_constraints;
    }

    ViolatedConstraint *add_violated_constraint(ViolatedConstraint * constraint)
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
