#ifndef MKC_INEQUALITY_WHEEL_HPP
#define MKC_INEQUALITY_WHEEL_HPP

#include <vector>
#include <string>
#include <algorithm>
#include "./MKC_Inequalities.hpp"
#include "./MKCUtil.hpp"
#include "./Utils/Exception.hpp"
#include "./MKCGraph.hpp"
#include "./Heuristics/Tabu.hpp"
#include "./Heuristics/Grasp.hpp"

namespace maxkcut
{
// TODO include Tabu

// Inequality of type
// sum(u,v_i) - sum(v_{i-1},v_i) - u_{1,2} <= rhs
class MKC_InequalityWheel : public MKC_Inequalities
{
private:
    const int size_cycle;
    const int size_hub;
    Tabu<int> tabu;
    Grasp<int> grasp;

protected:
    void set_rhs()
    {
        this->rhs = (double)size_hub * ((int)(size_cycle / 2));
    }

    //For max-k-cut variables are the edges
    //return number of variables in the hub of the constraints
    int number_variables_in_hub() const
    {
        return (size_hub * (size_hub - 1) / 2);
    }

    //For the max-k-cut: variables = edges
    //return number total of variables in wheel constraint
    int total_number_variables_constraint() const
    {
        return size_cycle + size_cycle * size_hub + number_variables_in_hub();
    }

public:
    MKC_InequalityWheel(const int &size_c, const int &size_h) : MKC_Inequalities(0.0),
                                                                size_cycle(size_c),
                                                                size_hub(size_h)
    {
        this->set_rhs();
    }

    void find_violated_constraints(const VariablesEdge *variables,
                                   const MKCInstance *instance,
                                   LinearViolatedConstraints *violated_constraints)
    {
        try
        {
            const MKCGraph *graph = instance->get_graph();

            GRASP_heurisistic(variables, graph->get_edges(), violated_constraints);
        }
        catch (Exception &e)
        {
            e.add_to_msg("\n In MKC_InequalityWheel");
            e.execute();
        }
        catch (const std::exception &e)
        {
            Exception ept = Exception(e.what(), ExceptionType::STOP_EXECUTION);
            ept.execute();
        }
        catch (...)
        {
            std::string msg = "Not handled exception in MKC_InequalityWheel \n";
            Exception ept = Exception(msg, ExceptionType::STOP_EXECUTION);
            ept.execute();
        }
    }

    void GRASP_heurisistic(const VariablesEdge *variables,
                           const Edges *edges,
                           LinearViolatedConstraints *violated_constraints)
    {

        tabu.clean(-1);
        std::vector<double> coefficients(total_number_variables_constraint(), 1.0);
        this->set_coefficients_wheel(&coefficients);
        std::vector<const Variable *> variables_in_wheel(total_number_variables_constraint());

        //vertices start by 1
        for (int i = 1; i <= edges->get_number_vertices(); ++i)
        {
            if (tabu.has_element(i))
                continue;

            std::vector<int> vertices_in_cycle(size_cycle, -1);
            std::vector<int> vertices_in_hub(size_hub, -1);

            //set first vertex in cycle
            vertices_in_cycle[0] = i;
            if (this->Construct_grasp_random_wheel(variables, edges, &vertices_in_cycle, &vertices_in_hub))
            {
                double val;
                if (this->evaluate_wheel(variables, edges, vertices_in_cycle, vertices_in_hub, &val))
                {
                    this->local_search_wheel(variables, edges, &vertices_in_cycle, &vertices_in_hub, &val);
                    if ((val - this->rhs) > maxkcut::EPSILON)
                    {

                        if (this->set_variables_of_wheel(variables,
                                                         edges,
                                                         vertices_in_cycle, vertices_in_hub,
                                                         &variables_in_wheel))
                        {
                            this->add_vertex_in_tabu_list(edges, vertices_in_cycle);

                            violated_constraints->add_violated_constraint(
                                LinearViolatedConstraint::create(0.0,
                                                                 this->rhs,
                                                                 ConstraintType::INFERIOR_EQUAL,
                                                                 val - this->rhs,
                                                                 variables_in_wheel.size(),
                                                                 &variables_in_wheel[0],
                                                                 &coefficients[0]));
                        }
                    }
                }
            }
        }
    }

    void add_vertex_in_tabu_list(const Edges *edges, const std::vector<int> &vertices_in_cycle)
    {
        double val;
        int vertex_min_sum,
            idx_vj,
            idx_vi;
        double current_min = maxkcut::INFINITY_DOUBLE;

        for (int vi = 0; vi < vertices_in_cycle.size() - 1; ++vi)
        {
            idx_vi = vertices_in_cycle[vi];
            for (int vj = vi + 1; vj < vertices_in_cycle.size(); ++vj)
            {
                idx_vj = vertices_in_cycle[vj];
                val = edges->get_edge_by_vertices(idx_vi, idx_vj)->get_weight();
                if (val < current_min)
                {
                    vertex_min_sum = idx_vi;
                    current_min = std::abs(val);
                }
            }
        }

        //insert vertex in tabu
        tabu.add_value(vertex_min_sum);
    }

    void local_search_wheel(const VariablesEdge *variables,
                            const Edges *edges,
                            std::vector<int> *vertices_in_cycle,
                            std::vector<int> *vertices_in_hub,
                            double *previous_val)
    {
        double new_val;
        //vertices start by 1
        for (int i = 1; i <= edges->get_number_vertices(); ++i)
        {

            int flag = 0;
            if (!tabu.has_element(i) &&
                find(vertices_in_hub->begin(), vertices_in_hub->end(), i) == vertices_in_hub->end() &&
                find(vertices_in_cycle->begin(), vertices_in_cycle->end(), i) == vertices_in_cycle->end())
            {
                //test cycle
                for (int v = 0; v < vertices_in_cycle->size() && flag == 0; ++v)
                {
                    int v_in_cycle = (*vertices_in_cycle)[v];
                    (*vertices_in_cycle)[v] = i;
                    if (this->evaluate_wheel(variables, edges, *vertices_in_cycle, *vertices_in_hub, &new_val) &&
                        new_val > (*previous_val))
                    {
                        (*previous_val) = new_val;
                        flag = 1;
                    }
                    else
                    {
                        (*vertices_in_cycle)[v] = v_in_cycle;
                    }
                }

                //test hub
                for (int u = 0; u < vertices_in_hub->size() && flag == 0; ++u)
                {
                    int u_in_hub = (*vertices_in_hub)[u];
                    (*vertices_in_hub)[u] = i;

                    if (this->evaluate_wheel(variables, edges, *vertices_in_cycle, *vertices_in_hub, &new_val) &&
                        new_val > (*previous_val))
                    {
                        (*previous_val) = new_val;
                        flag = 1;
                    }
                    else
                    {
                        (*vertices_in_hub)[u] = u_in_hub;
                    }
                }
            }
        }
    }

    bool evaluate_wheel(const VariablesEdge *variables,
                        const Edges *edges,
                        const std::vector<int> &vertices_in_cycle,
                        const std::vector<int> &vertices_in_hub,
                        double *sum)
    {
        *sum = 0.0;

        //(edges in hub): - (X_{u_1,u_2})
        for (int ui = 0; ui < vertices_in_hub.size() - 1; ++ui)
        {
            int v_ui = vertices_in_hub[ui];
            for (int uj = ui + 1; uj < vertices_in_hub.size(); ++uj)
            {

                const Edge *edge = edges->get_edge_by_vertices(v_ui, vertices_in_hub[uj]);
                if (edge == nullptr)
                {
                    return false;
                }
                *sum -= variables->get_variable(edge)->get_solution();
            }
        }

        //Cycle: - sum(v_{i-1},v_i)
        for (int v = 0; v < vertices_in_cycle.size(); ++v)
        {
            int previous_vertex = v != 0 ? v - 1 : vertices_in_cycle.size() - 1;

            const Edge *edge = edges->get_edge_by_vertices(vertices_in_cycle[v],
                                                           vertices_in_cycle[previous_vertex]);
            if (edge == nullptr)
            {
                return false;
            }
            *sum -= variables->get_variable(edge)->get_solution();
        }

        //hub_and_cycle: +sum(v,v_i)
        for (int ui = 0; ui < vertices_in_hub.size(); ++ui)
        {
            int v_ui = vertices_in_hub[ui];
            for (int v = 0; v < vertices_in_cycle.size(); ++v)
            {
                const Edge *edge = edges->get_edge_by_vertices(v_ui, vertices_in_cycle[v]);

                if (edge == nullptr)
                {
                    return false;
                }
                *sum += variables->get_variable(edge)->get_solution();
            }
        }

        return true;
    }

    bool Construct_grasp_random_wheel(const VariablesEdge *variables,
                                      const Edges *edges,
                                      std::vector<int> *vertices_in_cycle,
                                      std::vector<int> *vertices_in_hub)
    {
        int dimension = edges->get_number_vertices();
        double val;

        //Build cycle (first vertex already set)
        for (int i = 1; i < size_cycle; ++i)
        {
            grasp.clear_rcl();
            int vertex_selected = -1;
            int prev_vertex = (*vertices_in_cycle)[i - 1];
            //vertices start by 1
            for (int v = 1; v <= dimension; ++v)
            {
                const Edge *edge = edges->get_edge_by_vertices(v, prev_vertex);
                if (edge != nullptr &&
                    !tabu.has_element(v) &&
                    find(vertices_in_cycle->begin(), vertices_in_cycle->end(), v) == vertices_in_cycle->end())
                {
                    val = variables->get_variable(edge)->get_solution();
                    if (i == size_cycle - 1) //last vertex in cycle
                    {
                        edge = edges->get_edge_by_vertices(v, (*vertices_in_cycle)[0]);
                        if (edge != nullptr)
                        {
                            val += variables->get_variable(edge)->get_solution();
                            grasp.add_candidate_to_rcl(v, val);
                        }
                    }
                    else
                    {
                        grasp.add_candidate_to_rcl(v, val);
                    }
                }
            }

            vertex_selected = grasp.select_candidate_from_rcl(-1);

            if (vertex_selected == -1)
            {
                return false;
            }
            (*vertices_in_cycle)[i] = vertex_selected;
        }

        //Build hub
        for (int i = 0; i < size_hub; ++i)
        {
            grasp.clear_rcl();
            int vertex_selected = -1;

            //vertices start by 1
            for (int v = 1; v <= dimension; ++v)
            {
                if (!tabu.has_element(v) &&
                    this->is_vertex_valid_to_hub(variables, edges, vertices_in_cycle, vertices_in_hub, v, &val))
                {
                    grasp.add_candidate_to_rcl(v, -val);
                }
            }

            vertex_selected = grasp.select_candidate_from_rcl(-1);

            if (vertex_selected == -1)
            {
                return false;
            }

            (*vertices_in_hub)[i] = vertex_selected;
        }

        return true;
    }

    bool is_vertex_valid_to_hub(const VariablesEdge *variables,
                                const Edges *edges,
                                std::vector<int> *vertices_in_cycle,
                                std::vector<int> *vertices_in_hub,
                                const int &vertex,
                                double *sum)
    {
        *sum = 0.0;

        //check hub
        for (int i = 0; i < vertices_in_hub->size(); ++i)
        {
            if ((*vertices_in_hub)[i] == -1)
            {
                break;
            }

            const Edge *edge = edges->get_edge_by_vertices(vertex, (*vertices_in_hub)[i]);
            if (edge == nullptr)
            {
                return false;
            }

            *sum -= variables->get_variable(edge)->get_solution();
        }

        //check cycle
        for (int i = 0; i < vertices_in_cycle->size(); ++i)
        {

            const Edge *edge = edges->get_edge_by_vertices(vertex, (*vertices_in_cycle)[i]);
            if (edge == nullptr)
            {
                return false;
            }

            *sum += variables->get_variable(edge)->get_solution();
        }

        return true;
    }

    /* Set Coefficients of constraints. All constraints have same set of coefficients then we must set only once 
     * Order of coefficients ==> - sum(v_{i-1},v_i) + sum(u_i,v_i) - (X_{u_1,u_2}) 
     * Obs: at the begin all coefficients are (+1.0) then change only negatives coefficients 
    */
    void set_coefficients_wheel(std::vector<double> *coefficients)
    {
        // - sum(v_{i-1},v_i)
        for (int i = 0; i < size_cycle; ++i)
        {
            (*coefficients)[i] = -1.0;
        }

        //+ sum(u_i,v_i)
        //Do nothing (coefficients are already positive)

        // - (X_{u_1,u_2})
        int last_position = coefficients->size() - 1;
        for (int i = 0; i < number_variables_in_hub(); ++i)
        {
            (*coefficients)[last_position - i] = -1.0;
        }
    }

    /* set Variables in wheel constraint\
    * Order:
    *   1) cycle
    *    2) edges of hub and cycle 
    *    3) edges in hub    
    * */
    bool set_variables_of_wheel(const VariablesEdge *variables,
                                const Edges *edges,
                                const std::vector<int> &vertices_in_cycle,
                                const std::vector<int> &vertices_in_hub,
                                std::vector<const Variable *> *variables_in_wheel)
    {
        int counter = 0;
        //Cycle: - sum(v_{i-1},v_i)
        for (int v = 0; v < vertices_in_cycle.size(); ++v)
        {
            int previous_vertex = v != 0 ? v - 1 : vertices_in_cycle.size() - 1;

            const Edge *edge = edges->get_edge_by_vertices(vertices_in_cycle[v],
                                                           vertices_in_cycle[previous_vertex]);
            if (edge == nullptr)
            {
                return false;
            }
            (*variables_in_wheel)[counter++] = variables->get_variable(edge);
        }

        //hub_and_cycle: +sum(v,v_i)
        for (int ui = 0; ui < vertices_in_hub.size(); ++ui)
        {
            int v_ui = vertices_in_hub[ui];
            for (int v = 0; v < vertices_in_cycle.size(); ++v)
            {
                int v_cycle = vertices_in_cycle[v];
                const Edge *edge = edges->get_edge_by_vertices(v_ui, v_cycle);
                if (edge == nullptr)
                {
                    return false;
                }
                (*variables_in_wheel)[counter++] = variables->get_variable(edge);
            }
        }

        //(edges in hub): - (X_{u_1,u_2})
        for (int ui = 0; ui < vertices_in_hub.size() - 1; ++ui)
        {
            int v_ui = vertices_in_hub[ui];
            for (int uj = ui + 1; uj < vertices_in_hub.size(); ++uj)
            {
                int v_uj = vertices_in_hub[uj];
                const Edge *edge = edges->get_edge_by_vertices(v_ui, v_uj);
                if (edge == nullptr)
                {
                    return false;
                }
                (*variables_in_wheel)[counter++] = variables->get_variable(edge);
            }
        }

        return true;
    }

    std::string wheel_to_string(const std::vector<int> &vertices_in_cycle, const std::vector<int> &vertices_in_hub)
    {
        std::string s = "Wheel: cycle{";
        for (int i = 0; i < vertices_in_cycle.size(); ++i)
        {
            s += std::to_string(vertices_in_cycle[i]) + ",";
        }

        s += "} hub{";
        for (int i = 0; i < vertices_in_hub.size(); ++i)
        {
            s += std::to_string(vertices_in_hub[i]) + ",";
        }

        s += "};";

        return s;
    }
};
} // namespace maxkcut

#endif
