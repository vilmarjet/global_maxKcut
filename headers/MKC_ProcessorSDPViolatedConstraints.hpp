#ifndef SDP_VIOLATED_CONSTRAINTS_HPP
#define SDP_VIOLATED_CONSTRAINTS_HPP

#include "./CPA/SDPViolatedConstraint.hpp"
#include "./Solver/Constraint/LinearConstraint.hpp"

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
#include "VariablesEdgeSDP.hpp"
#include "./Utils/Exception.hpp"

namespace maxkcut
{
class ProcessorSDPViolatedConstraints : public ProcessorViolatedConstraints
{
public:
    static ProcessorSDPViolatedConstraints *create(const int &nb,
                                                   Solver *solver_,
                                                   const MKCInstance *instance_,
                                                   const VariablesEdgeSDP *variablesEdge_)
    {
        return new ProcessorSDPViolatedConstraints(nb, solver_, instance_, variablesEdge_);
    }

private:
    const MKCInstance *instance;
    const VariablesEdgeSDP *variablesEdgeSDP;

    ProcessorSDPViolatedConstraints(const int &nb,
                                    Solver *solver_,
                                    const MKCInstance *instance_,
                                    const VariablesEdgeSDP *variablesEdge_) : ProcessorViolatedConstraints(nb, solver_),
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
    ProcessorSDPViolatedConstraints *find_violation(ViolatedConstraints **inequalities_type, const size_t &size)
    {
        for (size_t i = 0; i < size; ++i)
        {
            find_violation(inequalities_type[i]);
        }

        return this;
    }

    ProcessorSDPViolatedConstraints *find_violation(ViolatedConstraints *inequality)
    {
        inequality->find_violated_constraints();

        for (auto violated_constraint : inequality->get_constraints())
        {
            switch (violated_constraint->get_constraint()->get_type_constraint())
            {
            case ConstraintType::LINEAR:
                add_violated_constraint_from_linear_to_sdp((LinearViolatedConstraint *)violated_constraint);
                break;
            case ConstraintType::SYMMETRIC:
                add_violated_constraint(violated_constraint);
                break;
            default:
                Exception("Not defined constraint type for SDP solver", ExceptionType::STOP_EXECUTION).execute();
                break;
            }
        }

        inequality->reset();

        return this;
    }

    ProcessorSDPViolatedConstraints *populate()
    {
        int counter_ineq = 0;
        for (auto constraint : violated_constraints)
        {
            //std::cout << constraint->to_string() << "\n";
            solver->add_constraint_SDP(ConstraintSDP::from((ConstraintSDP *)constraint->get_constraint()));

            if (++counter_ineq > get_max_number_inequalities())
            {
                break;
            }
        }

        return this;
    }

    void add_violated_constraint_from_linear_to_sdp(LinearViolatedConstraint *violatedLinearConstraint)
    {

        LinearConstraint *linearC = violatedLinearConstraint->get_constraint();
        double increase_rhs = compute_increase_rhs_LP_to_SDP(linearC);

        SDPViolatedConstraint *sdp_viol = (SDPViolatedConstraint *)add_violated_constraint(
            SDPViolatedConstraint::create(linearC->get_lower_bound() + increase_rhs,
                                          linearC->get_upper_bound() + increase_rhs,
                                          linearC->get_bound_key(),
                                          violatedLinearConstraint->get_violation()));

        for (ConstraintCoefficient<Variable> *constraintCoeff : linearC->get_constraint_coefficients())
        {
            sdp_viol->add_coefficient(variablesEdgeSDP->get_variable_sdp(),
                                      constraintCoeff->get_variable(),
                                      transforme_LpIneq01_in_SDPineq(constraintCoeff->get_value()));
        }
    }

    double compute_increase_rhs_LP_to_SDP(LinearConstraint *linearC)
    {
        double sum = 0.0;
        double LBsdp = -1.0 / (instance->get_K() - 1.0);
        double UBsdp = 1.0;
        double mulCst = LBsdp/(UBsdp - LBsdp);

        for (ConstraintCoefficient<Variable> *constraintCoeff : linearC->get_constraint_coefficients())
        {
            sum += constraintCoeff->get_value() * mulCst;
        }

        return sum;
    }

    ~ProcessorSDPViolatedConstraints()
    {
        for (auto p : violated_constraints)
        {
            delete p;
        }
    }
};
} // namespace maxkcut

#endif