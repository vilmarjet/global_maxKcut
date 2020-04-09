#ifndef VARIABLESEdgeSDP_CONTAINER_HPP
#define VARIABLESEdgeSDP_CONTAINER_HPP

#include "./Solver/Abstract/Solver.hpp"
#include "./Solver/Variable/Variable.hpp"
#include "./Utils/Exception.hpp"
#include <vector>
#include <map>
#include <string>
#include <utility>
#include "./MKCInstance.hpp"
#include "./VariablesEdge.hpp"

namespace maxkcut
{

class VariablesEdgeSDP : public VariablesEdge
{
public:
    static VariablesEdgeSDP *create(Solver *_solver, MKCInstance *instance)
    {
        return (new VariablesEdgeSDP(_solver, instance))->populate();
    }

private:
    int dimension;
    int K;
    SDPVariable<Variable> *var_sdp;

    VariablesEdgeSDP(Solver *_solver, const MKCInstance *instance) : VariablesEdge(_solver, instance)
    {
        this->dimension = instance->get_graph()->get_dimension();
        this->K = instance->get_K();
    }

public:
    ~VariablesEdgeSDP() {}

    VariablesEdgeSDP *populate()
    {
        double const_sdp_var = (-1.0) * ((this->K - 1.0) / this->K);
        var_sdp = solver->add_sdp_variable(new SDPVariable<Variable>(dimension, const_sdp_var));

        //todo: Impemente populate
        double lower_bound = -1.0 / (K - 1.0);
        double upper_bound = 1.0;
        double initial_solution = 0.0;
        VariableType type = VariableType::SDP;
        for (int i = 0; i < edges->get_number_edges(); ++i)
        {
            const Edge *edge = edges->get_edge_by_index(i);
            int vi = edge->get_vertex_i() - 1;
            int vj = edge->get_vertex_j() - 1;
            double cost_var = -1.0*edge->get_weight() / 2.0;

            const Variable *variable = var_sdp->add_variable(vi, vj,
                                                             Variable::create(lower_bound,
                                                                              upper_bound,
                                                                              initial_solution,
                                                                              cost_var,
                                                                              type));
            add_variable(edge, variable);
        }

        return this;
    }

    const SDPVariable<Variable> *get_variable_sdp()
    {
        return var_sdp;
    }
};
} // namespace maxkcut

#endif
