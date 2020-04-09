#ifndef SOLVER_MOSEK_SDP
#define SOLVER_MOSEK_SDP

#include "SolverMosek.hpp"
#include "../Abstract/Solver.hpp"
#include "../../Utils/Exception.hpp"
#include <stdlib.h> //malloc

class SolverMosekSDP : public Solver, SolverMosek
{
private:
public:
  SolverMosekSDP(const SolverParam &solverParm) : Solver(solverParm)
  {
    create_environnement();
  }
  ~SolverMosekSDP()
  {
  }

  void solve()
  {
    try
    {
      if (this->task == NULL)
      {
        throw Exception("Null task in solve", ExceptionType::STOP_EXECUTION);
      }

      append_variables();
      append_constraints();

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
      int size = variables->size();

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
        this->variables->set_solution_value(i, var_x[i]);
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
    initilize_objective_function();

    SolverParam param = get_parameter();
    MSK_putintparam(task, MSK_IPAR_INFEAS_REPORT_AUTO, MSK_ON);
    MSK_putintparam(task, MSK_IPAR_NUM_THREADS, param.get_number_threads()); /*Nber of cpus*/
  }

  void append_variables()
  {
    add_linear_variables_mosek_task(get_lp_variables());
    add_sdp_variables_mosek_task(get_sdp_variables());
  }

  void add_constraint(const LinearConstraint *constraint, bool is_to_append_new = true)
  {
    this->add_constraint_append_mosek(constraint, is_to_append_new, get_lp_variables());
  }

  void add_constraint_SDP(const ConstraintSDP *constraint, bool is_to_append_new = true)
  {
    this->add_constraint_append_mosek_SDP(constraint, is_to_append_new, get_sdp_variables());
  }

  void append_constraints()
  {

    /*Linear Constraints*/
    int size = get_linear_constraints()->get_number_non_appended_constraints();
    if (size > 0)
    {
      if (r_code == MSK_RES_OK)
      {
        r_code = MSK_appendcons(task, (MSKint32t)size);
      }

      for (int i = 0; i < size && r_code == MSK_RES_OK; ++i)
      {
        const LinearConstraint *constraint = get_linear_constraints()->get_next_constraint_to_append();
        add_constraint(constraint, false);
      }

      if (r_code != MSK_RES_OK)
      {
        std::string msg = "r_code = " + std::to_string(r_code) + "!= MSK_RES_OK in linear execute_constraints()";
        Exception(msg, ExceptionType::STOP_EXECUTION).execute();
      }
    }

    /*SDP constraints*/
    size = get_sdp_constraints()->get_number_non_appended_constraints();
    if (size > 0)
    {
      if (r_code == MSK_RES_OK)
      {
        r_code = MSK_appendcons(task, (MSKint32t)size);
      }

      for (int i = 0; i < size && r_code == MSK_RES_OK; ++i)
      {
        const ConstraintSDP *constraint = get_sdp_constraints()->get_next_constraint_to_append();
        add_constraint_SDP(constraint, false);
      }

      if (r_code != MSK_RES_OK)
      {
        std::string msg = "r_code = " + std::to_string(r_code) + "!= MSK_RES_OK in execute_constraints()";
        Exception(msg, ExceptionType::STOP_EXECUTION).execute();
      }
    }
    //nop
  }
  void reset_solver()
  {
    this->task = NULL;
    this->env = NULL;
    this->r_code = MSK_RES_OK;

    this->objectiveFunction.update_solution(0.0);
    this->get_lp_variables()->reset_position_append_variable();
    this->get_linear_constraints()->reset_position_append_constraint();
  }

  void create_environnement()
  {
    r_code = MSK_makeenv(&env, NULL);
    create_task(0, 0);
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

    if (r_code == MSK_RES_OK)
    {
      r_code = MSK_putobjsense(task, get_mosek_objective_sense(this->objectiveFunction.get_optimization_sense()));
    }

    if (r_code != MSK_RES_OK)
    {
      Exception("r_code != MSK_RES_OK in initilize_Objective_function()", ExceptionType::STOP_EXECUTION).execute();
    }
  }
};

#endif