#ifndef MKC_INEQUALITY_SDP_BOUND_HPP
#define MKC_INEQUALITY_SDP_BOUND_HPP

#include "./CPA/ViolatedConstraints.hpp"
#include "./Solver/Abstract/Solver.hpp"
#include "MKCInstance.hpp"
#include "MKCUtil.hpp"
#include "./CPA/SDPViolatedConstraint.hpp"
#include "./Solver/Constraint/ConstraintBoundKey.hpp"
#include "./Solver/Constraint/ConstraintBoundKey.hpp"

namespace maxkcut
{
class MKC_InequalitySDPBound : public ViolatedConstraints
{

public:
    static MKC_InequalitySDPBound *create(const VariablesEdgeSDP *variables_,
                                          const MKCInstance *instance_)
    {
        return new MKC_InequalitySDPBound(variables_, instance_);
    }

private:
    const VariablesEdgeSDP *variables;
    const MKCInstance *instance;
    const double LBsdp;
    const double UBsdp;
    const ConstraintBoundKey type_bound;
    MKC_InequalitySDPBound(const VariablesEdgeSDP *variables_,
                           const MKCInstance *instance_) : variables(variables_),
                                                           instance(instance_),
                                                           LBsdp(-1.0 / (instance->get_K() - 1.0)),
                                                           UBsdp(1.0),
                                                           type_bound(ConstraintBoundKey::SUPERIOR_EQUAL)
    {
    }

public:
    ~MKC_InequalitySDPBound()
    {
    }

    void find_violated_constraints()
    {
        //Do nothing. It is coded in
    }

    //MUST BE EXECUTED BEFORE TRANSFORMATION IN SOLUTION
    void find_violated_bound_constraints_sdp()
    {
        SDPVariable<Variable> *sdp_var = variables->get_variable_sdp();

        for (auto var : sdp_var->get_variables())
        {
            double sdp_value = var->get_solution();
            double diff = LBsdp - sdp_value;

            if (diff > EPSILON)
            {
                add_violation(sdp_var, var, diff);
            }
        }
    }

    inline void add_violation(const SDPVariable<Variable> *sdp_var, const Variable *var, const double &viol)
    {

        SDPViolatedConstraint *constraint = (SDPViolatedConstraint *)
            add_violated_constraint(SDPViolatedConstraint::create(2.0 * viol,
                                                                  ConstraintSDP::create(LBsdp, UBsdp, type_bound)));
        constraint->add_coefficient(sdp_var, var, 0.5);
    }

    std::string to_string() const override
    {
        std::string print;
        print = "In MKC_InequalitySDPBound \n";
        print += "Nb of constraints = " + std::to_string(violated_constraints.size()) + "\n";

        for (auto violated_constraint : violated_constraints)
        {
            print += ((SDPViolatedConstraint *)violated_constraint)->to_string();
            print += "\n";
        }

        return print;
    }
};
} // namespace maxkcut

#endif