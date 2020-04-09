#ifndef MKC_EDGE_MODEL_LP_HPP
#define MKC_EDGE_MODEL_LP_HPP

#include "./Solver/Abstract/Solver.hpp"
#include "./MKCInstance.hpp"
#include "./MKCGraph.hpp"
#include "./CPA/LinearViolatedConstraints.hpp"
#include "./MKC_Inequalities.hpp"
#include <algorithm> // use of min and max
#include <set>
#include <vector>
#include "VariablesEdge.hpp"

namespace maxkcut
{

class MKC_ModelEdgeLP
{
private:
    Solver *solver;
    MKCInstance *instance;
    VariablesEdge *variablesEdge;
    std::vector<MKC_Inequalities *> inequalities_type;

public:
    MKC_ModelEdgeLP(MKCInstance *instance_, Solver *solver_) : instance(instance_),
                                                               solver(solver_)
    {
        this->initilize();
        inequalities_type.clear();
    }

    void solve()
    {
        this->solver->solve();
        std::cout << solver->to_string();
    }

    void reset_solver()
    {
        this->solver->reset_solver();
    }

    void initilize()
    {
        variablesEdge = VariablesEdge::create(solver, instance);
        this->set_objective_function();
    }
    void set_objective_function()
    {
        double cst = instance->get_graph()->get_edges()->sum_cost_all_edges();
        solver->set_const_objective_function(cst);
    }

    void add_type_inequality(MKC_Inequalities *ineq_type)
    {
        this->inequalities_type.push_back(ineq_type);
    }

    void find_violated_constraints(const int &nb_max_ineq)
    {
        //violated_constraints.clear();
        LinearViolatedConstraints *linearViolatedConstraints = LinearViolatedConstraints::create(nb_max_ineq, solver);

        for (std::size_t idx_ineq = 0; idx_ineq < inequalities_type.size(); ++idx_ineq)
        {
            //@todo: create class for violated constraints and send as parameter or return in get violated inequalities
            inequalities_type[idx_ineq]->find_violated_constraints(this->variablesEdge,
                                                                   this->instance,
                                                                   linearViolatedConstraints);
        }

        linearViolatedConstraints->apply_constraints();
        

        std::cout << "Nb constraints after= " << solver->get_linear_constraints()->size();
    }

    ~MKC_ModelEdgeLP()
    {
    }
};
} // namespace maxkcut

#endif
