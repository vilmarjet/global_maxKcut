#ifndef SOLVER_MOSEK_SDP
#define SOLVER_MOSEK_SDP

#include "mosek.h"
#include "SolverMosek.hpp"
#include "Solver.hpp"
#include "../Utils/Exception.hpp"
#include <stdlib.h> //malloc
#include "SolverParam.hpp"

class SolverMosekSDP : public Solver, SolverMosek
{
private:
public:
  SolverMosekSDP(const SolverParam &solverParm) : Solver(solverParm) {}
  ~SolverMosekSDP()
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
      Exception("Not handled exception set varialbe \n", ExceptionType::STOP_EXECUTION).execute();
    }
  }

  void finalize_optimization()
  {
    //nop
  }

  void set_solution()
  {
    set_mosek_solution_statuss(&solsta, MSK_SOL_ITR);

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
        this->variables.set_solution_value(i, var_x[i]);
      }

      break;
    }
    case MSK_SOL_STA_DUAL_INFEAS_CER:
    case MSK_SOL_STA_PRIM_INFEAS_CER:
      throw Exception("Infeasible problem", ExceptionType::DO_NOTHING);
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
    std::cout << "Begin of initilize";

    create_task(Solver::variables.size(), this->number_constraints);
    initialize_variables_mosek_task(Solver::variables);
    initialize_sdp_variables_mosek_task(&(Solver::variables_sdp));
    initilize_objective_function();

    SolverParam param = get_parameter();
    MSK_putintparam(task, MSK_IPAR_INFEAS_REPORT_AUTO, MSK_ON);
    //r_code = MSK_putintparam(task, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_INTPNT);
    MSK_putintparam(task, MSK_IPAR_NUM_THREADS, param.get_number_threads()); /*Nber of cpus*/
  }

  void add_constraint(const Constraint *constraint, bool is_to_append_new = true)
  {
    this->add_constraint_append_mosek(constraint, is_to_append_new, this->number_constraints, get_variables());
    ++this->number_constraints;
  }

  void add_constraint_SDP(const ConstraintSDP *constraint, bool is_to_append_new = true)
  {
    this->add_constraint_append_mosek_SDP(constraint, is_to_append_new, this->number_constraints, get_variables());
    ++this->number_constraints;
  }

  void add_constraints(const Constraint *constraints, const int &size)
  {
    r_code = MSK_appendcons(task, (MSKint32t)size);

    for (int i = 0; i < size; ++i)
    {
      add_constraint(&constraints[i], false);
    }
  }

  void add_constraints(const std::set<Constraint> *constraints)
  {
    r_code = MSK_appendcons(task, (MSKint32t)constraints->size());

    for (std::set<Constraint>::iterator it = constraints->begin();
         it != constraints->end();
         ++it)
    {
      add_constraint(&(*it), false);
    }
  }

  void add_constraint_single_SDP_variable(const int &idx_var_sdp, const Constraint *constraints)
  {
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

  void run_optimizer()
  {
    r_code = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
    /* Run optimizer */
    if (r_code == MSK_RES_OK)
    {
      r_code = MSK_optimizetrm(task, NULL);
    }

    if (r_code != MSK_RES_OK)
    {
      throw Exception("r_code not MSK_RES_OK in run_optimizer()", ExceptionType::STOP_EXECUTION);
    }
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
      const Variable *variable = this->variables.get_variable(i);
      r_code = MSK_putcj(task, i, -1.0 * variable->get_cost());
    }

    if (r_code == MSK_RES_OK)
    {
      r_code = MSK_putobjsense(task, get_mosek_objective_sense(this->objectiveFunction.get_optimization_sense()));
    }

    /*For SDP*/
    MSKint64t idx; //use for index in mosek
    SDPVariables *vars = &(Solver::variables_sdp);
    size = vars->size();
    for (int i = 0; i < size && r_code == MSK_RES_OK; ++i)
    {
      if (vars->get_variable(i)->get_row_indices()) //check no null
      {
        r_code = MSK_appendsparsesymmat(task,
                                        vars->get_variable(i)->get_dimension(),
                                        vars->get_variable(i)->get_number_non_null_variables(),
                                        vars->get_variable(i)->get_col_indices(),
                                        vars->get_variable(i)->get_row_indices(),
                                        vars->get_cost_indices(i),
                                        &idx);
      }
      if (r_code == MSK_RES_OK)
      {
        double val = vars->get_variable(i)->get_constant_object_function();
        r_code = MSK_putbarcj(task, i, 1, &idx, &val);
      }
    }

    if (r_code != MSK_RES_OK)
    {
      Exception("r_code != MSK_RES_OK in initilize_Objective_function()", ExceptionType::STOP_EXECUTION).execute();
    }
  }
};

#endif