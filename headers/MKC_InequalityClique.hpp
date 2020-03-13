#ifndef MKC_INEQUALITY_CLIQUE_HPP
#define MKC_INEQUALITY_CLIQUE_HPP

#include <vector>
#include <string>
#include <algorithm>
#include "./MKC_Inequalities.hpp"
#include "./MKCUtil.hpp"
#include "./Utils/Exception.hpp"
#include "./MKCGraph.hpp"
#include "./Heuristics/Tabu.hpp"
#include <cmath> // std::abs

namespace maxkcut
{

class MKC_InequalityClique : public MKC_Inequalities
{
protected:
  int size_clique;
  int nb_edges_clique;
  Tabu<int> tabu;

public:
  MKC_InequalityClique(const int &size_c) : MKC_Inequalities(0.0),
                                            size_clique(size_c)
  {
    this->nb_edges_clique = (int)((size_c * (size_c - 1)) / 2.0);
  }

  ~MKC_InequalityClique() {}

  void find_violated_constraints(const VariablesEdge* variables,
                                 const MKCInstance *instance,
                                 std::set<ViolatedConstraint *, CompViolatedConstraint> *violated_constraints)
  {
    this->rhs = compute_rhs(instance);
    const Edges *edges = instance->get_graph()->get_edges();

    this->deterministic_heuristic(variables, edges, violated_constraints);
  }

private:
  void deterministic_heuristic(const VariablesEdge *variables,
                               const Edges *edges,
                               std::set<ViolatedConstraint *, CompViolatedConstraint> *violated_constraints)
  {
    try
    {
      std::vector<int> vertices_in_clique(this->size_clique);
      std::vector<const Variable *> variables_in_clique(this->nb_edges_clique);
      std::vector<double> coeeficient(this->nb_edges_clique, 1.0);

      //tabu
      tabu.clean(-1);

      for (int v_i = 0; v_i < edges->get_number_vertices(); ++v_i)
      {
        if (tabu.has_element(v_i + 1))
        {
          continue;
        }

        std::fill(vertices_in_clique.begin(), vertices_in_clique.end(), -1);
        vertices_in_clique[0] = v_i + 1; //vertices starts in 1

        if (this->find_minimum_clique_deterministic(variables, edges, &vertices_in_clique))
        {
          double val = this->local_search_deterministic_clique(variables, edges, &vertices_in_clique);
          this->set_vertex_in_tabu_list(edges, &vertices_in_clique);

          if (val < this->rhs - maxkcut::EPSILON)
          {
            double violation = this->rhs - val;
            this->set_edges_in_clique(vertices_in_clique, &variables_in_clique, edges, variables);

            violated_constraints->insert(new ViolatedConstraint(variables_in_clique,
                                                                coeeficient,
                                                                this->rhs,
                                                                variables_in_clique.size(),
                                                                ConstraintType::SUPERIOR_EQUAL,
                                                                violation));
          }
        }
      }
    }
    catch (Exception &e)
    {
      e.add_to_msg("\n In deterministic_heuristic of Clique");
      e.execute();
    }
    catch (const std::exception &e)
    {
      Exception ept = Exception(e.what(), ExceptionType::STOP_EXECUTION);
      ept.execute();
    }
    catch (...)
    {
      std::string msg = "Not handled exception in MKC_InequalityClique \n";
      Exception ept = Exception(msg, ExceptionType::STOP_EXECUTION);
      ept.execute();
    }
  }

  void set_edges_in_clique(std::vector<int> &vertices_in_clique,
                           std::vector<const Variable *> *variables_in_clique,
                           const Edges *edges,
                           const VariablesEdge *variables)
  {
    int counter_edge = 0;
    for (int vi = 0; vi < vertices_in_clique.size() - 1; ++vi)
    {
      int idx_vi = vertices_in_clique[vi];
      for (int vj = vi + 1; vj < vertices_in_clique.size(); ++vj)
      {
        int idx_vj = vertices_in_clique[vj];
        const Edge *edge = edges->get_edge_by_vertices(idx_vi, idx_vj);
        (*variables_in_clique)[counter_edge++] = variables->get_variable(edge);
      }
    }
  }

  double local_search_deterministic_clique(const VariablesEdge *variables, const Edges *edges, std::vector<int> *vertices_in_clique)
  {
    double current_val = this->evaluate_clique(variables, edges, vertices_in_clique);

    for (int v = 0; v < edges->get_number_vertices(); ++v)
    {
      int current_vertex = v + 1;

      if (this->is_vertex_allowed_in_clique_tabu(current_vertex, vertices_in_clique, edges))
      {
        for (int indx_clique = 0; indx_clique < vertices_in_clique->size(); ++indx_clique)
        {
          int v_in_clique = (*vertices_in_clique)[indx_clique];
          (*vertices_in_clique)[indx_clique] = current_vertex;

          double new_val = this->evaluate_clique(variables, edges, vertices_in_clique);

          if (new_val < current_val)
          {
            current_val = new_val;
            break;
          }
          else
          {
            (*vertices_in_clique)[indx_clique] = v_in_clique;
          }
        }
      }
    }

    return current_val;
  }

  void set_vertex_in_tabu_list(const Edges *edges, std::vector<int> *vertices_in_clique)
  {
    double val;
    int vertex_min_sum,
        idx_vj,
        idx_vi;
    double current_min = maxkcut::INFINITY_DOUBLE;

    for (int vi = 0; vi < vertices_in_clique->size() - 1; ++vi)
    {
      idx_vi = (*vertices_in_clique)[vi];
      for (int vj = vi + 1; vj < vertices_in_clique->size(); ++vj)
      {
        idx_vj = (*vertices_in_clique)[vj];
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

  double evaluate_clique(const VariablesEdge *variables, const Edges *edges, std::vector<int> *vertices_in_clique)
  {
    double sum = 0.0;
    for (int vi = 0; vi < vertices_in_clique->size() - 1; ++vi)
    {
      int idx_vi = (*vertices_in_clique)[vi];
      for (int vj = vi + 1; vj < vertices_in_clique->size(); ++vj)
      {
        int idx_vj = (*vertices_in_clique)[vj];
        const Edge *edge = edges->get_edge_by_vertices(idx_vi, idx_vj);
        sum += variables->get_variable(edge)->get_solution();
      }
    }
  }

  bool find_minimum_clique_deterministic(const VariablesEdge *variables, const Edges *edges, std::vector<int> *vertices_in_clique)
  {
    int index_set_vertex = 1; // first vertex has already been set

    for (int index_set_vertex = 1; index_set_vertex < vertices_in_clique->size(); ++index_set_vertex)
    {
      int vertex_min_sum = -1;

      for (int v_i = 0; v_i < edges->get_number_vertices(); ++v_i)
      {
        double current_min = maxkcut::INFINITY_DOUBLE;
        int current_vi = v_i + 1; //vertices start by 1

        if (this->is_vertex_allowed_in_clique_tabu(current_vi, vertices_in_clique, edges))
        {
          double sum = this->get_sum_variable_solution_if_vertex_in_clique(current_vi,
                                                                           vertices_in_clique,
                                                                           edges,
                                                                           variables);

          if (sum < current_min)
          {
            current_min = sum;
            vertex_min_sum = current_vi;

            if (sum <= maxkcut::ZERO) //best ever
            {
              goto assignm_vertex;
            }
          }
        }
      }

      if (vertex_min_sum == -1)
      {
        return false;
      }
    assignm_vertex:
      (*vertices_in_clique)[index_set_vertex] = vertex_min_sum;
    }

    return true;
  }

  double get_sum_variable_solution_if_vertex_in_clique(const int &vertex,
                                                       const std::vector<int> *vertices_in_clique,
                                                       const Edges *edges,
                                                       const VariablesEdge *variables)
  {
    double sum = 0.0;

    for (int i = 0; i < vertices_in_clique->size(); ++i)
    {
      int vertex_in_clique = (*vertices_in_clique)[i];
      if (vertex_in_clique == -1) //end of vertices in clique
      {
        break;
      }

      const Edge *edge = edges->get_edge_by_vertices(vertex, vertex_in_clique);
      sum += variables->get_variable(edge)->get_solution();
    }

    return sum;
  }

  bool is_vertex_allowed_in_clique(const int &vertex, const std::vector<int> *vertices_in_clique, const Edges *edges)
  {
    for (int i = 0; i < vertices_in_clique->size(); ++i)
    {
      int vertex_already_in_clique = (*vertices_in_clique)[i];
      if (vertex_already_in_clique == -1) //end of vertices in clique
      {
        break;
      }

      if (vertex == vertex_already_in_clique || !edges->has_edge(vertex, vertex_already_in_clique))
      {
        return false;
      }
    }

    return true;
  }

  bool is_vertex_allowed_in_clique_tabu(const int &vertex, const std::vector<int> *vertices_in_clique, const Edges *edges)
  {
    if (tabu.has_element(vertex))
      return false;
    return is_vertex_allowed_in_clique(vertex, vertices_in_clique, edges);
  }

  double compute_rhs(const MKCInstance *instance)
  {

    if (this->rhs > maxkcut::ZERO)
    {
      return this->rhs;
    }

    int t_GClique, q_GClique;
    double z;

    /* size clique ought be greater than number of partitions  */
    if (this->size_clique <= instance->get_K())
    {
      std::string msg = "Size of clique: " + std::to_string(this->size_clique);
      msg += " ought be greater than number of partitions";
      throw Exception(msg, ExceptionType::STOP_EXECUTION);
    }

    //Calculating the "t" and "q" of the General_Clique
    t_GClique = (double)(this->size_clique) / instance->get_K();
    q_GClique = this->size_clique - t_GClique * instance->get_K();

    //Calcule of the right side of the inequality
    double cst = (1.0 / 2.0) * t_GClique * (t_GClique - 1) * (instance->get_K() - q_GClique);
    cst += (1.0 / 2.0) * t_GClique * (t_GClique + 1) * q_GClique;

    return cst;
  }
};
} // namespace maxkcut

#endif
