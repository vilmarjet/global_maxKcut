#ifndef MKC_EDGE_MODEL_LP_HPP
#define MKC_EDGE_MODEL_LP_HPP

#include "./Solver/Abstract/Solver.hpp"
#include "./MKCInstance.hpp"
#include "./MKCGraph.hpp"
#include "MKC_LinearViolatedConstraints.hpp"
#include "./MKC_Inequalities.hpp"
#include <algorithm> // use of min and max
#include <set>
#include <vector>
#include "VariablesEdge.hpp"

#include "MKC_Inequalities.hpp"
#include "MKC_InequalityTriangle.hpp"
#include "MKC_InequalityClique.hpp"
#include "MKC_InequalityWheel.hpp"
#include "MKC_InequalityLpSdp.hpp"

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
        inequalities_type.clear();
        this->initilize();
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

        initialize_constraints();

        this->set_objective_function();
    }

    void initialize_constraints()
    {

        add_type_inequality(MKC_InequalityTriangle::create(variablesEdge, instance));
        add_type_inequality(MKC_InequalityClique::create(variablesEdge, instance, instance->get_K() + 1));
        add_type_inequality(MKC_InequalityClique::create(variablesEdge, instance, instance->get_K() + 2));
        add_type_inequality(MKC_InequalityWheel::create(variablesEdge, instance));
        add_type_inequality(MKC_InequalityWheel::create(variablesEdge, instance, 3, 2));
        add_type_inequality(MKC_InequalityLpSdp::create(variablesEdge, instance));

    }

    void set_objective_function()
    {
        double cst = instance->get_graph()->get_edges()->sum_weight_all_edges();
        solver->set_const_objective_function(cst);
    }

    void add_type_inequality(MKC_Inequalities *ineq_type)
    {
        this->inequalities_type.push_back(ineq_type);
    }

    void find_violated_constraints(const int &nb_max_ineq)
    {
        //violated_constraints.clear();
        LinearViolatedConstraints *linearViolatedConstraints =
            LinearViolatedConstraints::create(nb_max_ineq, solver, &inequalities_type)
                ->find()
                ->populate();

        delete linearViolatedConstraints;

        std::cout << "Nb constraints after= " << solver->get_linear_constraints()->size();
    }

    ~MKC_ModelEdgeLP()
    {
    }
};
} // namespace maxkcut

#endif
