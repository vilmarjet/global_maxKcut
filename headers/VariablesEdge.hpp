#ifndef VARIABLESEdge_CONTAINER_HPP
#define VARIABLESEdge_CONTAINER_HPP

#include "./Solver/Solver.hpp"
#include "./Solver/Variables1D.hpp"
#include "./Utils/Exception.hpp"
#include <vector>
#include <map>
#include <string>
#include <utility>
#include "./MKCInstance.hpp"

namespace maxkcut
{

class VariablesEdge : public Variables1D<const Edge *>
{
public:
    static VariablesEdge *create(Solver *_solver, MKCInstance *instance)
    {
        return (new VariablesEdge(_solver, instance))->populate();
    }

private:
    Solver *solver;
    const Edges *edges;

    VariablesEdge(Solver *_solver, const MKCInstance *instance) : solver(_solver)
    {
        edges = instance->get_graph()->get_edges();
    }

public:
    ~VariablesEdge() {}

    VariablesEdge * populate()
    {
        double lower_bound = 0.0;
        double upper_bound = 1.0;
        double initial_solution = 0.0;
        VariableType type = VariableType::CONTINOUS;
        for (int i = 0; i < edges->get_number_edges(); ++i)
        {
            const Edge *edge = edges->get_edge_by_index(i);

            const Variable *variable = solver->add_variable(new Variable(lower_bound,
                                                                         upper_bound,
                                                                         initial_solution,
                                                                         edge->get_weight(),
                                                                         type));
            int idx = solver->get_variables()->get_index(variable);
            add_variable(idx, edge, variable);
        }

        return this;
    }
};
} // namespace maxkcut

#endif
