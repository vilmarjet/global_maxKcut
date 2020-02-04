#ifndef MKC_INEQUALITY_TRIANGLE_HPP
#define MKC_INEQUALITY_TRIANGLE_HPP

#include <set>
#include <vector>
#include "./MKC_Inequalities.hpp"
#include "./MKCUtil.hpp"
#include "./Solver/Solver.hpp"

namespace maxkcut
{
// TRIANGLE CONTRAINT -> X_ij + X_hj - X_hi <= 1
class MKC_InequalityTriangle : public MKC_Inequalities
{

public:
  MKC_InequalityTriangle(/* args */) : MKC_Inequalities(1.0)
  {
  }

  void find_violated_constraints(const Solver *solver,
                                 const MKCInstance *instance,
                                 std::set<ViolatedConstraint *, CompViolatedConstraint>  *violated_constraints)
  {
    try
    {
      const Edges *edges = instance->get_graph()->get_edges();
      const Variables *variables = solver->get_variables();
      std::cout << solver->to_string();
      double sum;

      int dim = edges->get_number_vertices();

      for (int i = 0; i < dim - 2; ++i)
      {
        for (int j = i + 1; j < dim - 1; ++j)
        {
          const GraphEdge *edge1 = edges->get_edge_by_vertices(i + 1, j + 1); //edges start with 1

          if (edge1 == nullptr)
          {
            continue;
          }

          double value_vi_vj = variables->get_variable(edge1)->get_solution();
          for (int h = j + 1; h < dim; ++h)
          {
            // std::cout << "i, j, h" << i << ","<< j << ","<< h << "\n";
            const GraphEdge *edge2 = edges->get_edge_by_vertices(i + 1, h + 1);
            const GraphEdge *edge3 = edges->get_edge_by_vertices(h + 1, j + 1);

            if (edge2 == nullptr || edge3 == nullptr)
            {
              continue;
            }

            const Variable *var_edges[] = {variables->get_variable(edge1),
                                           variables->get_variable(edge2),
                                           variables->get_variable(edge3)};

            double value_vh_vj = variables->get_variable(edge2)->get_solution();
            double value_vi_vh = variables->get_variable(edge3)->get_solution();
            
            //first contraint -> -X_ij + X_hj + X_hi <= 1/
            sum = -value_vi_vj + value_vh_vj + value_vi_vh;
            if (sum > this->rhs + maxkcut::EPSILON)
            {
              double coef[] = {-1.0, 1.0, 1.0};
              violated_constraints->insert(new ViolatedConstraint(var_edges,
                                                                  coef,
                                                                  3,
                                                                  -1.0,
                                                                  1.0,
                                                                  ConstraintType::INFERIOR_EQUAL,
                                                                  sum - this->rhs));
            }

            //second contraint -> X_ij - X_hj + X_hi <= 1/
            sum = value_vi_vj - value_vh_vj + value_vi_vh;
            if (sum > this->rhs + maxkcut::EPSILON)
            {
              double coef[] = {1.0, -1.0, 1.0};
              violated_constraints->insert(new ViolatedConstraint(var_edges,
                                                                  coef,
                                                                  3,
                                                                  -1.0,
                                                                  1.0,
                                                                  ConstraintType::INFERIOR_EQUAL,
                                                                  sum - this->rhs));
            }

            //Thrid contraint -> X_ij + X_hj - X_hi <= 1/
            sum = value_vi_vj + value_vh_vj - value_vi_vh;
            if (sum > this->rhs + maxkcut::EPSILON)
            {
              double coef[] = {1.0, 1.0, -1.0};
              violated_constraints->insert(new ViolatedConstraint(var_edges,
                                                                  coef,
                                                                  3,
                                                                  -1.0,
                                                                  1.0,
                                                                  ConstraintType::INFERIOR_EQUAL,
                                                                  sum - this->rhs));
            }
          }
        }
      }
    }
    catch (Exception e)
    {
      e.execute();
    }
    catch (const std::exception &e)
    {
      Exception ept = Exception(e.what(), ExceptionType::STOP_EXECUTION);
      ept.execute();
    }
    catch (...)
    {
      std::string msg = "Not handled exception in MKCGraph::get_weight_edge() \n";
      Exception ept = Exception(msg, ExceptionType::STOP_EXECUTION);
      ept.execute();
    }
  }

  ~MKC_InequalityTriangle() {}
};

} // namespace maxkcut

#endif
