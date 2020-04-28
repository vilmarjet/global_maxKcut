#ifndef SOLVER_MOSEK_SDP
#define SOLVER_MOSEK_SDP

#include "SolverMosek.hpp"
#include "../Abstract/Solver.hpp"
#include "../../Utils/Exception.hpp"
#include <stdlib.h> //malloc

class SolverMosekSDP : public Solver, public SolverMosek
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
        initialize();
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

  virtual void finalize_optimization()
  {
    //nop
  }

  void set_solution()
  {
    MSKsoltype_enum type_solution = MSK_SOL_ITR;
    set_mosek_solution_statuss(&solsta, type_solution);

    switch (solsta)
    {
    case MSK_SOL_STA_PRIM_AND_DUAL_FEAS:
    case MSK_SOL_STA_OPTIMAL:
    {
      int size = variables->size();

      // @todo fix problem with size (why should be at least size*2 ?)
      std::vector<double> var_x(size); //= (double *)calloc(size, sizeof(double));
      std::vector<double> Obj(2);      //double *Obj = (double *)calloc(2, sizeof(MSKrealt));

      //Just interior point solution
      MSK_getxx(task, type_solution, &var_x[0]);
      MSK_getprimalobj(task, type_solution, &Obj[0]);

      this->objectiveFunction.update_solution(Obj[0]);

      for (int i = 0; i < size; ++i)
      {
        this->get_lp_variables()->get_variable(i)->update_solution(var_x[i]);
      }

      for (SDPVariable<Variable> *sdp_var : get_sdp_variables()->get_variables())
      {
        int idx = get_sdp_variables()->get_index(sdp_var);
        std::vector<double> barx(sdp_var->get_total_number_variables());
        MSK_getbarxj(task, type_solution, idx, &barx[0]);
        int dim = sdp_var->get_dimension();
        for (int i = 0; i < dim; ++i)
        {
          for (int j = i; j < dim; ++j)
          {
            Variable *var = sdp_var->get_variable(i, j);
            var->update_solution(getValueXij_SDP(i, j, &barx[0], dim));
          }
        }
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

  inline double getValueXij_SDP(const int &i, const int &j, double *barx, const int &DIM)
  {
    int row = i,
        col = j;

    if (row > col)
    {
      std::swap(row, col);
    }

    int aux = 0;
    for (int count = 0; count <= row; ++count)
    {
      aux += count;
    }

    return barx[col + row * DIM - aux];
  }

  void initialize()
  {
    
    create_task(get_lp_variables()->size(), get_linear_constraints()->size() + get_sdp_constraints()->size());
    initilize_objective_function();

    //MSK_putintparam(task, MSK_IPAR_INFEAS_REPORT_AUTO, MSK_ON);
    MSK_putintparam(task, MSK_IPAR_NUM_THREADS, param.get_number_threads()); /*Nber of cpus*/
    update_termination_param(TerminationParamBuilder<std::nullptr_t>::create()->build(), false);
  }

  void append_variables()
  {
    add_linear_variables_mosek_task(get_lp_variables());
    add_sdp_variables_mosek_task(get_sdp_variables());
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
        SolverMosek::add_constraint_append_mosek(constraint, false, get_lp_variables());
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
        SolverMosek::add_constraint_append_mosek_SDP(constraint, false, get_sdp_variables());
      }

      if (r_code != MSK_RES_OK)
      {
        std::string msg = "r_code = " + std::to_string(r_code) + "!= MSK_RES_OK in execute_constraints()";
        Exception(msg, ExceptionType::STOP_EXECUTION).execute();
      }
    }
    //nop
  }
  virtual void reset_solver()
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

  void update_termination_param(TerminationParam *early_param, const bool &is_early)
  {
    if (is_early)
    {
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, early_param->get_gap_tolerance());           //Relative complementarity gap tolerance.
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_PFEAS, early_param->get_gap_primal());               // primal feasibility
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_DFEAS, early_param->get_gap_tolerance());            // dual feasibility
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, early_param->get_gap_relative_tolerance()); // Tol to optimality
    }
    else
    {
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, CONSTANTS::ZERO);  //Relative complementarity gap tolerance.
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_PFEAS, CONSTANTS::ZERO);   // primal feasibility
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_DFEAS, CONSTANTS::ZERO);   // dual feasibility
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, CONSTANTS::ZERO); // Tol to optimality
    }
  }
};

#endif