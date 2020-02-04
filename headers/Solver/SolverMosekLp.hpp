#ifndef SOLVER_MOSEK_LP
#define SOLVER_MOSEK_LP

#include "mosek.h"
#include "SolverMosek.hpp"
#include "Solver.hpp"
#include "../Utils/Exception.hpp"
#include <stdlib.h> //malloc
#include "SolverParam.hpp"

class SolverMosekLp : public Solver, SolverMosek
{
private:
public:
  SolverMosekLp(const SolverParam &solverParm) : Solver(solverParm) {}
  ~SolverMosekLp()
  {
  }

  void solve()
  {
    try
    {
      if (this->task == NULL)
      {
        initialize();
      }

      run_optimizer();

      set_solution();

      add_time_of_solver(get_time_optimization());

      finalize_optimization();
    }
    catch (const Exception &e)
    {
      e.execute();
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
    catch (...)
    {
      Exception e = Exception("Not handled exception set varialbe \n", ExceptionType::STOP_EXECUTION);
      e.execute();
    }
  }

  void finalize_optimization()
  {
    //nop
  }

  void set_solution()
  {
    set_mosek_solution_statuss(&solsta, MSK_SOL_BAS);

    switch (solsta)
    {
    case MSK_SOL_STA_PRIM_AND_DUAL_FEAS:
    case MSK_SOL_STA_OPTIMAL:
    {
      int size = variables.size();

      // @todo fix problem with size (why should be at least size*2 ?)
      std::vector<double> var_x(size); //= (double *)calloc(size, sizeof(double));
      std::vector<double> Obj(2);      //double *Obj = (double *)calloc(2, sizeof(MSKrealt));

      //if basic solution is activated
      if (solsta == MSK_SOL_STA_OPTIMAL)
      {
        MSK_getxx(task, MSK_SOL_BAS, &var_x[0]);
        MSK_getprimalobj(task, MSK_SOL_BAS, &Obj[0]);
      }
      else
      {
        //Just interior point solution
        MSK_getxx(task, MSK_SOL_ITR, &var_x[0]);
        MSK_getprimalobj(task, MSK_SOL_ITR, &Obj[0]);
      }

      this->objectiveFunction.update_solution(Obj[0]);

      for (int i = 0; i < size; ++i)
      {
        this->variables.get_variable_by_index(i)->update_solution(var_x[i]);
      }

      break;
    }
    case MSK_SOL_STA_DUAL_INFEAS_CER:
    case MSK_SOL_STA_PRIM_INFEAS_CER:
      throw Exception("Infeasible problem", ExceptionType::STOP_EXECUTION);
      break;
    case MSK_SOL_STA_UNKNOWN:
    {
      char symname[MSK_MAX_STR_LEN];
      char desc[MSK_MAX_STR_LEN];
      /* If the solutions status is unknown, print the termination code
        indicating why the optimizer terminated prematurely. */
      MSK_getcodedesc(trmcode, symname, desc);
      std::printf("The optimizer terminitated with code: %s\n", symname);
      throw Exception("Unknown solution LP problem", ExceptionType::STOP_EXECUTION);
      break;
    }
    default:
      break;
    }
  }

  void initialize()
  {
    create_task(Solver::variables.size(), this->number_constraints);
    initialize_variables_mosek_task(Solver::variables);
    initilize_objective_function();

    r_code = MSK_putobjsense(task, get_mosek_objective_sense(this->objectiveFunction.get_optimization_sense()));

    SolverParam param = get_parameter();
    MSK_putintparam(task, MSK_IPAR_NUM_THREADS, param.get_number_threads()); /*Nber of cpus*/

    //Controls whether the interior-point optimizer also computes an optimal basis.
    r_code = MSK_putintparam(task, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_INTPNT);
  }

  void add_constraint(const Constraint *constraint)
  {
    this->add_constraint_append_mosek(constraint, true, this->number_constraints);
    ++this->number_constraints;
  }

  void add_constraints(const Constraint *constraints, const int &size)
  {
    r_code = MSK_appendcons(task, (MSKint32t)size);

    for (int i = 0; i < size; ++i)
    {
      this->add_constraint_append_mosek(&constraints[i], false, this->number_constraints);
      ++this->number_constraints;
    }
  }

  void add_constraints(const std::set<Constraint> *constraints)
  {
    r_code = MSK_appendcons(task, (MSKint32t)constraints->size());

    for (std::set<Constraint>::iterator it = constraints->begin();
         it != constraints->end();
         ++it)
    {
      this->add_constraint_append_mosek(&(*it), false, this->number_constraints);
      ++this->number_constraints;
    }
  }

  void reset_solver()
  {
    this->task = NULL;
    this->env = NULL;
    this->r_code = MSK_RES_OK;

    this->number_constraints = 0;
    this->objectiveFunction.update_solution(0.0);
  }

  void create_environnement()
  {
    r_code = MSK_makeenv(&env, NULL);
  }

  void initilize_objective_function()
  {
    if (r_code == MSK_RES_OK)
    {
      r_code = MSK_putcfix(task, this->objectiveFunction.get_constant_term());
    }

    int size = get_variables()->size();

    for (int i = 0; i < size && r_code == MSK_RES_OK; ++i)
    {
      Variable *variable = this->variables.get_variable_by_index(i);
      r_code = MSK_putcj(task, variable->get_index(), -1.0 * variable->get_cost());
    }

    if (r_code == MSK_RES_OK)
    {
      r_code = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE);
    }

    if (r_code != MSK_RES_OK)
    {
      throw Exception("r_code != MSK_RES_OK in initilize_Objective_function()", ExceptionType::STOP_EXECUTION);
    }
  }
};

#endif