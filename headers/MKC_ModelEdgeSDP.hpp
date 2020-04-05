#ifndef MKC_EDGE_MODEL_SDP_HPP
#define MKC_EDGE_MODEL_SDP_HPP

#include "./Solver/Solver.hpp"
#include "./MKCInstance.hpp"
#include "./MKCGraph.hpp"
#include "./Solver/Variable.hpp"
#include "./CPA/ViolatedConstraint.hpp"
#include "./MKC_Inequalities.hpp"
#include <algorithm> // use of min and max
#include <set>
#include <vector>
#include "VariablesEdgeSDP.hpp"

namespace maxkcut
{

class MKC_ModelEdgeSDP
{
private:
    Solver *solver;
    MKCInstance *instance;
    VariablesEdgeSDP *variablesEdgeSDP;
    std::vector<MKC_Inequalities *> inequalities_type;
    std::set<ViolatedConstraint *, CompViolatedConstraint> violated_constraints;

public:
    MKC_ModelEdgeSDP(MKCInstance *instance_, Solver *solver_) : instance(instance_),
                                                                solver(solver_)
    {
        this->initilize();
        inequalities_type.clear();
    }

    void solve()
    {
        diagonal_constraint();
        this->solver->solve();
        std::cout << solver->to_string();
    }

    void reset_solver()
    {
        this->solver->reset_solver();
    }

    void initilize()
    {
        variablesEdgeSDP = VariablesEdgeSDP::create(solver, instance);
        this->set_objective_function();
        solver->initialize();
    }
    void set_objective_function()
    {
        double cst = instance->get_graph()->get_edges()->sum_cost_all_edges();
        int K = instance->get_K();
        cst *= (-1.0) * ((K - 1.0) / K);
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

        for (std::size_t idx_ineq = 0; idx_ineq < inequalities_type.size(); ++idx_ineq)
        {
            //@todo: create class for violated constraints and send as parameter or return in get violated inequalities
            inequalities_type[idx_ineq]->find_violated_constraints(this->variablesEdgeSDP,
                                                                   this->instance,
                                                                   &violated_constraints);
        }

        for (std::set<ViolatedConstraint *, CompViolatedConstraint>::iterator it = violated_constraints.begin();
             it != violated_constraints.end() && counter_ineq < nb_max_ineq;
             ++it, ++counter_ineq)
        {
            int idx_sdp_variable = 0;
            //solver->add_constraint_single_SDP_variable(idx_sdp_variable, (*it)->get_constraint());
        }

        std::cout << "Nb constraints after= " << solver->get_nb_constraints();
    }

    void diagonal_constraint()
    {
        const SDPVariables *sdp_vars = solver->get_sdp_variables();
        const SDPVariable<Variable> *sdp_var = sdp_vars->get_variable(0); //

        int dim = sdp_var->get_dimension();
        double lowerBound = 1.0;
        double upperBound = 1.0;
        ConstraintType type = ConstraintType::EQUAL;
        double coeff = 1.0;

        for (int i = 0; i < dim; ++i)
        {
            const Variable *variable = sdp_var->get_variable(i, i);
            ConstraintSDP* constraint = solver->add_constraint_SDP(lowerBound, upperBound, type);
            constraint->add_coefficient(sdp_var, variable, coeff);
        }

       solver->execute_constraints();
    }

    ~MKC_ModelEdgeSDP()
    {
    }
};
} // namespace maxkcut

#endif
