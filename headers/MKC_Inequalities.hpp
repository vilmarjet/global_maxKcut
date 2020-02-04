#ifndef MKC_INEQUALITIESSS_HPP
#define MKC_INEQUALITIESSS_HPP

#include "./CPA/ViolatedConstraint.hpp"
#include "./Solver/Solver.hpp"
#include "./MKCGraph.hpp"
#include "./MKCInstance.hpp"

#include <vector>

namespace maxkcut
{
class MKC_Inequalities
{
protected:
    double rhs;

public:
    MKC_Inequalities(const double &_rhs) : rhs(_rhs) {}
    virtual void find_violated_constraints(const Solver *solver,
                                           const MKCInstance *instance,
                                           std::set<ViolatedConstraint *, CompViolatedConstraint>  *violated_constraints) = 0;
};
} // namespace maxkcut
#endif
