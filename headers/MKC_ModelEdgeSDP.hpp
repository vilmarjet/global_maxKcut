#ifndef MKC_EDGE_MODEL_SDP_HPP
#define MKC_EDGE_MODEL_SDP_HPP

#include "./Solver/Abstract/Solver.hpp"
#include "./MKCInstance.hpp"
#include "./MKCGraph.hpp"
#include "MKC_LinearViolatedConstraints.hpp"
#include "MKC_Inequalities.hpp"
#include <algorithm> // use of min and max
#include <set>
#include <vector>
#include "VariablesEdgeSDP.hpp"
#include "MKC_InequalitySDPDiagonal.hpp"

namespace maxkcut
{

class MKC_ModelEdgeSDP
{
private:
    Solver *solver;
    MKCInstance *instance;
    VariablesEdgeSDP *variablesEdgeSDP;
    std::vector<MKC_Inequalities *> inequalities_type;

public:
    MKC_ModelEdgeSDP(MKCInstance *instance_, Solver *solver_) : instance(instance_),
                                                                solver(solver_)
    {
        this->initilize();
        inequalities_type.clear();
    }

    void solve()
    {
        this->solver->solve();
        variablesEdgeSDP->transforme_SDP_solution();
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

        this->set_objective_function();
    }
    void set_objective_function()
    {
        double cst = instance->get_graph()->get_edges()->sum_weight_all_edges();
        int K = instance->get_K();
        cst *= ((K - 1.0) / K);
        solver->set_const_objective_function(cst);
    }

    void add_type_inequality(MKC_Inequalities *ineq_type)
    {
        this->inequalities_type.push_back(ineq_type);
    }

    void find_violated_constraints(const int &nb_max_ineq)
    {
        LinearViolatedConstraints *linearViolatedConstraints =
            LinearViolatedConstraints::create(nb_max_ineq,
                                              solver,
                                              &inequalities_type,
                                              instance,
                                              variablesEdgeSDP)->find();

        // linearViolatedConstraints->apply_constraints();
        delete linearViolatedConstraints;

        std::cout << "Nb constraints after= " << solver->get_linear_constraints()->size();
    }

    ~MKC_ModelEdgeSDP()
    {
    }
};
} // namespace maxkcut

#endif
