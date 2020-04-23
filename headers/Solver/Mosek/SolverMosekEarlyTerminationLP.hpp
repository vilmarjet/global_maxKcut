#ifndef SOLVER_MOSEK_EARLY_LP_HPP
#define SOLVER_MOSEK_EARLY_LP_HPP

#include "SolverMosek.hpp"
#include "../Abstract/Solver.hpp"

class SolverMosekEarlyTerminationLP : public SolverMosekLp
{
private:
    bool early_termination;
    int number_consective_early_termination;

public:
    SolverMosekEarlyTerminationLP(const SolverParam &solverParm) : SolverMosekLp(solverParm)
    {
        early_termination = false;
        number_consective_early_termination = 0;
    }
    ~SolverMosekEarlyTerminationLP() {}

    void set_mosek_solution_status(MSKsolstae *solsta)
    {
        if (early_termination)
        {
            set_mosek_solution_statuss(solsta, MSK_SOL_ITR);
        }
        else
        {
            set_mosek_solution_statuss(solsta, MSK_SOL_BAS);
        }
    }

    void run_optimizer() override
    {
        if (early_termination)
        {
            r_code = MSK_putintparam(task, MSK_IPAR_INTPNT_BASIS, MSK_BI_NEVER); //Controls whether the interior-point optimizer also computes an optimal basis.

            r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_MU_RED, param.get_gap_tolerance());           //Relative complementarity gap tolerance.
            r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_PFEAS, param.get_gap_primal());               // primal feasibility
            r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_DFEAS, param.get_gap_tolerance());            // dual feasibility
            r_code = MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_REL_GAP, param.get_gap_relative_tolerance()); // Tol to optimality
        }
        else
        {
            r_code = MSK_putintparam(task, MSK_IPAR_INTPNT_BASIS, MSK_BI_ALWAYS);
        }

        // r_code = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
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

    void finalize_optimization() override
    {
        if (early_termination)
        {
            number_consective_early_termination++;
            if (number_consective_early_termination > param.get_number_iterations_before_optimality())
            {
                this->early_termination = false;
                std::cout << "Changed to early = false \n";
            }
        }
        else
        {
            this->early_termination = true;
            this->number_consective_early_termination = 0;
        }
    }

    void set_solution() override
    {
        set_mosek_solution_status(&solsta);

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
            if (!early_termination)
            {
                std::cout << "\n**** optimal  \n";
                MSK_getxx(task, MSK_SOL_BAS, &var_x[0]);
                MSK_getprimalobj(task, MSK_SOL_BAS, &Obj[0]);
            }
            else
            {
                std::cout << "\n ***** Not optimal  \n";
                //Just interior point solution
                MSK_getxx(task, MSK_SOL_ITR, &var_x[0]);
                MSK_getprimalobj(task, MSK_SOL_ITR, &Obj[0]);
            }

            this->objectiveFunction.update_solution(Obj[0]);

            for (int i = 0; i < size; ++i)
            {
                this->variables->get_variable(i)->update_solution(var_x[i]);
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
            std::printf("The optimizer terminitated with code (MSK_SOL_STA_UNKNOWN): %s\n", symname);
            throw Exception("Unknown solution LP problem", ExceptionType::STOP_EXECUTION);
            break;
        }
        default:
            char symname[MSK_MAX_STR_LEN];
            char desc[MSK_MAX_STR_LEN];
            MSK_getcodedesc(trmcode, symname, desc);
            std::printf("The optimizer terminitated with code: %s\n", symname);
            throw Exception("Unknown status", ExceptionType::STOP_EXECUTION);
            break;
            break;
        }
    }

    void reset_solver() override
    {
        SolverMosekLp::reset_solver();
        this->get_lp_variables()->reset_position_append_variable();
    }
};

#endif