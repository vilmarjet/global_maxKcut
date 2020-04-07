#ifndef MKC_INEQUALITIESSS_HPP
#define MKC_INEQUALITIESSS_HPP

#include "./CPA/LinearViolatedConstraints.hpp"
#include "./Solver/Abstract/Solver.hpp"
#include "./MKCGraph.hpp"
#include "./MKCInstance.hpp"
#include "./VariablesEdge.hpp"

#include <vector>

namespace maxkcut
{
class MKC_Inequalities
{
protected:
    double rhs;

public:
    MKC_Inequalities(const double &_rhs) : rhs(_rhs) {}
    virtual void find_violated_constraints(const VariablesEdge *Variables,
                                           const MKCInstance *instance,
                                           LinearViolatedConstraints  *violated_constraints) = 0;
};
} // namespace maxkcut
#endif
