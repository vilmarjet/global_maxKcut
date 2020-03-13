#ifndef MKC_EDGE_MODEL_LP_HPP
#define MKC_EDGE_MODEL_LP_HPP

#include "./Solver/Solver.hpp"
#include "./MKCInstance.hpp"
#include "./MKCGraph.hpp"
#include "./Solver/Variable.hpp"
#include "./CPA/ViolatedConstraint.hpp"
#include "./MKC_Inequalities.hpp"
#include <algorithm> // use of min and max
#include <set>
#include <vector>
#include "Variables1D.hpp"

namespace maxkcut
{

class MKC_ModelEdgeLP
{
private:
    Solver *solver;
    MKCInstance *instance;
    Variables1D<Edge> *variablesEdge;
    std::vector<MKC_Inequalities *> inequalities_type;
    std::set<ViolatedConstraint *, CompViolatedConstraint> violated_constraints;

public:
    MKC_ModelEdgeLP(MKCInstance *instance_, Solver *solver_) : instance(instance_),
                                                               solver(solver_)
    {
        variablesEdge = new VariablesEdge<Edge>();
        this->initilize();
        inequalities_type.clear();
        
    }

    void solve()
    {
        this->solver->solve();
    }

    void reset_solver()
    {
        this->solver->reset_solver();
    }

    void initilize()
    {
        this->set_edge_variables();
        this->set_objective_function();
    }

    void set_edge_variables()
    {
        const Edges *edges = instance->get_graph()->get_edges();

        for (int i = 0; i < edges->get_number_edges(); ++i)
        {
            const Edge *edge = edges->get_edge_by_index(i);
            this->solver->add_variable(Variable(0.0,
                                                1.0,
                                                0.0,
                                                edge->get_weight(),
                                                VariableType::CONTINOUS));

            variablesEdge->add_variable(i, edge, this->solver->get_variables()->get_variable(i));
        }
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
        violated_constraints.clear();
        int counter_ineq = 0;
        std::cout << solver->to_string();

        for (std::size_t idx_ineq = 0; idx_ineq < inequalities_type.size(); ++idx_ineq)
        {
            //@todo: create class for violated constraints and send as parameter or return in get violated inequalities
            inequalities_type[idx_ineq]->find_violated_constraints(this->variablesEdge,
                                                                   this->instance,
                                                                   &violated_constraints);
        }

        for (std::set<ViolatedConstraint *, CompViolatedConstraint>::iterator it = violated_constraints.begin();
             it != violated_constraints.end() && counter_ineq < nb_max_ineq;
             ++it, ++counter_ineq)
        {
            // std::cout << (*it)->get_constraint()->to_string(this->instance->get_graph(), solver->get_variables()) << "\n";
            //std::cout << (*it)->to_string() << "\n";
            solver->add_constraint((*it)->get_constraint());
        }

        std::cout << "Nb constraints after= " << solver->get_nb_constraints();
        // std::cout << "End violation end ";

        // std::cin.get();
    }

    ~MKC_ModelEdgeLP()
    {
    }
};
} // namespace maxkcut

#endif
