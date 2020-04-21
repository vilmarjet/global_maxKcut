#ifndef MKC_EDGE_MODEL_SDP_HPP
#define MKC_EDGE_MODEL_SDP_HPP

#include "./Solver/Abstract/Solver.hpp"
#include "./MKCInstance.hpp"
#include "./MKCGraph.hpp"
#include "MKC_ProcessorSDPViolatedConstraints.hpp"
#include <algorithm> // use of min and max
#include <set>
#include <vector>
#include "VariablesEdgeSDP.hpp"
#include "MKC_InequalitySDPDiagonal.hpp"

#include "./CPA/ViolatedConstraints.hpp"
#include "MKC_InequalityTriangle.hpp"
#include "MKC_InequalityClique.hpp"
#include "MKC_InequalityWheel.hpp"
#include "MKC_InequalityLpSdp.hpp"
#include "MKC_InequalitySDPBound.hpp"

namespace maxkcut
{

class MKC_ModelEdgeSDP
{
private:
    Solver *solver;
    MKCInstance *instance;
    VariablesEdgeSDP *variablesEdgeSDP;
    std::vector<ViolatedConstraints *> inequalities_type;
    MKC_InequalitySDPBound* boundIneq;

public:
    MKC_ModelEdgeSDP(MKCInstance *instance_, Solver *solver_) : instance(instance_),
                                                                solver(solver_)
    {
        this->initilize();
    }

    void solve()
    {
        this->solver->solve();
        boundIneq->find_violated_bound_constraints_sdp();
        transforme_SDP_solution();
        std::cout << solver->to_string();
    }

    void reset_solver()
    {
        this->solver->reset_solver();
    }

    void initilize()
    {
        variablesEdgeSDP = VariablesEdgeSDP::create(solver, instance);
        MKC_InequalitySDPDiagonal::create(solver)->populate();
        boundIneq = MKC_InequalitySDPBound::create(variablesEdgeSDP, instance);

        initialize_constraints();

        this->set_objective_function();
    }
    void set_objective_function()
    {
        double cst = instance->get_graph()->get_edges()->sum_weight_all_edges();
        int K = instance->get_K();
        cst *= ((K - 1.0) / K);
        solver->set_const_objective_function(cst);
    }

    void add_type_inequality(ViolatedConstraints *ineq_type)
    {
        this->inequalities_type.push_back(ineq_type);
    }

    void initialize_constraints()
    {
        add_type_inequality(MKC_InequalityTriangle::create(variablesEdgeSDP, instance));
        add_type_inequality(MKC_InequalityClique::create(variablesEdgeSDP, instance, instance->get_K() + 1));
        add_type_inequality(MKC_InequalityClique::create(variablesEdgeSDP, instance, instance->get_K() + 2));
        add_type_inequality(MKC_InequalityWheel::create(variablesEdgeSDP, instance));
        add_type_inequality(MKC_InequalityWheel::create(variablesEdgeSDP, instance, 3, 2));
        add_type_inequality(MKC_InequalityLpSdp::create(variablesEdgeSDP, instance));
    }

    void find_violated_constraints(const int &nb_max_ineq)
    {
        ProcessorSDPViolatedConstraints *sdpViolatedConstraints =
            ProcessorSDPViolatedConstraints::create(nb_max_ineq,
                                                    solver,
                                                    this->instance,
                                                    this->variablesEdgeSDP)
                ->find_violation(&inequalities_type[0], inequalities_type.size())
                ->find_violation(boundIneq)
                ->populate();

        delete sdpViolatedConstraints;

        std::cout << "Nb constraints after= " << solver->get_linear_constraints()->size() + solver->get_sdp_constraints()->size();
    }

    void transforme_SDP_solution()
    {
        double LBsdp = -1.0 / (instance->get_K() - 1.0);
        double UBsdp = 1.0;
        double divCst = (UBsdp - LBsdp);

        for (auto var : variablesEdgeSDP->get_variable_sdp()->get_variables())
        {
            double sdp_value = var->get_solution();

            //For sdp the lower and upper bound should be set as constraints
            //Thus, we need to check if their values are respected.
            if (sdp_value < LBsdp)
            {
                sdp_value = LBsdp;
            }

            var->update_solution((sdp_value - LBsdp) / divCst);
        }
    }

    ~MKC_ModelEdgeSDP()
    {
    }
};
} // namespace maxkcut

#endif
