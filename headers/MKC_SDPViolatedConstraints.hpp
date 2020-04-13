#ifndef SDP_VIOLATED_CONSTRAINTS_HPP
#define SDP_VIOLATED_CONSTRAINTS_HPP

#include "MKC_SDPViolatedConstraint.hpp"
#include "./Solver/Constraint/LinearConstraint.hpp"

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
#include "VariablesEdgeSDP.hpp"

namespace maxkcut
{
class ProcessSDPViolatedConstraints : public ProcessViolatedConstraints
{
public:
    static ProcessSDPViolatedConstraints *create(const int &nb,
                                                 Solver *solver_,
                                                 std::vector<MKC_Inequalities *> *types,
                                                 const MKCInstance *instance,
                                                 const VariablesEdgeSDP *variablesEdge)
    {
        return new ProcessSDPViolatedConstraints(nb, solver_, types, instance, variablesEdge);
    }

private:
    Solver *solver;
    const std::vector<MKC_Inequalities *> *inequalities_type;
    const MKCInstance *instance;
    const VariablesEdgeSDP *variablesEdgeSDP;

    ProcessSDPViolatedConstraints(const int &nb,
                                  Solver *solver_,
                                  std::vector<MKC_Inequalities *> *types,
                                  const MKCInstance *instance_,
                                  const VariablesEdgeSDP *variablesEdge_) : ProcessViolatedConstraints(nb),
                                                                            solver(solver_),
                                                                            inequalities_type(types),
                                                                            instance(instance_),
                                                                            variablesEdgeSDP(variablesEdge_)
    {
    }

    /**
     * from lp y = (0,1) we want to pass to SDP where X = (Lb, Ub)
     *       	From line equation We have:
     *       		y = (X - Lb)/(Ub - Lb),
     *       		  = X//(Ub - Lb) -  Lb/(Ub - Lb)
     *       
     *       		Thus if we have vl*y,
     *       we transfor vl[i] = vl[i]*(1/(Ub - Lb)) and  bound = bound + Sum_i (vl[i]*(Lb/(Ub - Lb)))
     *       end of tranformation 
     * */
    double transforme_LpIneq01_in_SDPineq(const double &linearValue)
    {
        double LBsdp = -1.0 / (instance->get_K() - 1.0);
        double UBsdp = 1.0;

        return 0.5 * linearValue * (1.0 / (UBsdp - LBsdp));
    }

public:
    ~ProcessSDPViolatedConstraints() {}

    ProcessSDPViolatedConstraints *find()
    {
        for (auto inequality : *inequalities_type)
        {
            inequality->find_violated_constraints();
            for (auto violated_constraint : inequality->get_constraints())
            {
                add_violated_constraint_from_linear_to_sdp((LinearViolatedConstraint *)violated_constraint);
            }

            inequality->reset();
        }

        return this;
    }

    void add_violated_constraint_from_linear_to_sdp(LinearViolatedConstraint *violatedLinearConstraint)
    {

        LinearConstraint *linearC = violatedLinearConstraint->get_constraint();
        SDPViolatedConstraint *sdp_viol = (SDPViolatedConstraint *)add_violated_constraint(
            SDPViolatedConstraint::create(linearC->get_lower_bound(),
                                          linearC->get_upper_bound(),
                                          linearC->get_bound_key(),
                                          violatedLinearConstraint->get_violation()));

        for (ConstraintCoefficient<Variable> *constraintCoeff : linearC->get_constraint_coefficients())
        {
            sdp_viol->add_coefficient(variablesEdgeSDP->get_variable_sdp(),
                                      constraintCoeff->get_variable(),
                                      transforme_LpIneq01_in_SDPineq(constraintCoeff->get_value()));
        }
    }

    ProcessSDPViolatedConstraints *populate()
    {
        int counter_ineq = 0;
        for (auto constraint : violated_constraints)
        {
            solver->add_constraint_SDP(ConstraintSDP::from((ConstraintSDP *)(constraint->get_constraint())));

            if (++counter_ineq > get_max_number_inequalities())
            {
                break;
            }
        }

        return this;
    }
};
} // namespace maxkcut

#endif