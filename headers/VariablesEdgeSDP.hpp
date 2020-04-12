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
    const int dimension;
    const int K;
    SDPVariable<Variable> *var_sdp;

    VariablesEdgeSDP(Solver *_solver, const MKCInstance *instance) : VariablesEdge(_solver, instance),
                                                                     K(instance->get_K()),
                                                                     dimension(instance->get_graph()
                                                                                   ->get_dimension()) {}

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
        for (auto edge : edges->get_edges())
        {
            int vi = edge->get_vertex_i() - 1;
            int vj = edge->get_vertex_j() - 1;
            double cost_var = edge->get_weight() / 2.0;

            Variable *variable = var_sdp->add_variable(vi, vj,
                                                       Variable::create(lower_bound,
                                                                        upper_bound,
                                                                        initial_solution,
                                                                        cost_var,
                                                                        type));
            add_variable(edge, variable);
        }

        return this;
    }

    SDPVariable<Variable> *get_variable_sdp() const
    {
        return var_sdp;
    }

    void transforme_SDP_solution()
    {
        double LBsdp = -1.0 / (K - 1.0);
        double UBsdp = 1.0;
        double divCst = (UBsdp - LBsdp);

        for (auto edge : edges->get_edges())
        {
            Variable *var = get_variable(edge);
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
};
} // namespace maxkcut

#endif
