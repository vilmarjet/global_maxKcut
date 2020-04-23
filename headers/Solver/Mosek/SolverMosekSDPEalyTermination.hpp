#ifndef SOLVER_MOSEK_EARLY_SDP
#define SOLVER_MOSEK_EARLY_SDP

#include "SolverMosekSDP.hpp"
#include "../Abstract/Solver.hpp"
#include "../../Utils/Exception.hpp"
#include <stdlib.h> //malloc

class SolverMosekSDPEarlyTermination : public SolverMosekSDP
{
private:
  bool early_termination;
  int number_consective_early_termination;
  double ZERO = 1e-6;

public:
  SolverMosekSDPEarlyTermination(const SolverParam &solverParm) : SolverMosekSDP(solverParm)
  {
    bool early_termination = false;
    int number_consective_early_termination = 0;
    create_environnement();
  }
  ~SolverMosekSDPEarlyTermination()
  {
  }

  void finalize_optimization() override
  {
    if (early_termination)
    {
      number_consective_early_termination++;
      if (number_consective_early_termination > param.get_number_iterations_before_optimality())
      {
        this->early_termination = false;
      }
    }
    else
    {
      this->early_termination = true;
      this->number_consective_early_termination = 0;
    }
  }

  void reset_solver() override
  {
    SolverMosekSDP::reset_solver();
    this->early_termination = false;
  }

  void run_optimizer() override
  {
    // r_code = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
    if (early_termination)
    {
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, param.get_gap_tolerance());           //Relative complementarity gap tolerance.
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_PFEAS, param.get_gap_primal());               // primal feasibility
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_DFEAS, param.get_gap_tolerance());            // dual feasibility
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, param.get_gap_relative_tolerance()); // Tol to optimality
    }
    else
    {
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, ZERO);  //Relative complementarity gap tolerance.
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_PFEAS, ZERO);   // primal feasibility
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_DFEAS, ZERO);   // dual feasibility
      r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, ZERO); // Tol to optimality
    }

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
};

#endif