#ifndef MKC_EDGE_MODEL_LP_HPP
#define MKC_EDGE_MODEL_LP_HPP

#include "./Solver/Abstract/Solver.hpp"
#include "./MKCInstance.hpp"
#include "./MKCGraph.hpp"
#include "MKC_ProcessorLinearViolatedConstraints.hpp"
#include <algorithm> // use of min and max
#include <set>
#include <vector>
#include "VariablesEdge.hpp"

#include "./CPA/ViolatedConstraints.hpp"
#include "MKC_InequalityTriangle.hpp"
#include "MKC_InequalityClique.hpp"
#include "MKC_InequalityWheel.hpp"
#include "MKC_InequalityLpSdp.hpp"
#include "./Models/ModelAbstract.hpp"

namespace maxkcut
{

class MKC_ModelEdgeLP : public ModelAbstract
{
private:
    MKCInstance *instance;
    VariablesEdge *variablesEdge;
    std::vector<ViolatedConstraints *> inequalities_type;

public:
    MKC_ModelEdgeLP(MKCInstance *instance_, Solver *solver_) : instance(instance_),
                                                               ModelAbstract(solver_)
    {
        inequalities_type.clear();
        this->initilize();
    }

    MKC_ModelEdgeLP *solve()
    {
        this->solver->solve();
        std::cout << solver->to_string();
        return this;
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

    void add_type_inequality(ViolatedConstraints *ineq_type)
    {
        this->inequalities_type.push_back(ineq_type);
    }

    int find_violated_constraints(const int &nb_max_ineq)
    {
        //violated_constraints.clear();
        ProcessorLinearViolatedConstraints *linearViolatedConstraints =
            ProcessorLinearViolatedConstraints::create(nb_max_ineq, solver)
                ->find_violation(&inequalities_type[0], inequalities_type.size())
                ->populate();

        int nb_violations = linearViolatedConstraints->get_number_violated_constraints();

        delete linearViolatedConstraints;

        std::cout << "Nb constraints after= " << solver->get_linear_constraints()->size();
        return nb_violations;
    }

    ~MKC_ModelEdgeLP()
    {
    }
};
} // namespace maxkcut

#endif
