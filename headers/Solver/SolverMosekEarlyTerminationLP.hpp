#ifndef SOLVER_MOSEK_EARLY_LP_HPP
#define SOLVER_MOSEK_EARLY_LP_HPP

#include "mosek.h"
#include "SolverParam.hpp"
#include "SolverMosekLp.hpp"
#include "SolverMosek.hpp"
#include "Solver.hpp"

class SolverMosekEarlyTerminationLP : public Solver, SolverMosek
{
private:
    bool early_termination;
    int number_consective_early_termination;

public:
    SolverMosekEarlyTerminationLP(const SolverParam &solverParm) : Solver(solverParm)
    {
        early_termination = false;
        number_consective_early_termination = 0;
    }
    ~SolverMosekEarlyTerminationLP() {}

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

    void initialize()
    {
        create_task(Solver::variables.size(), this->number_constraints);
        SolverMosek::initialize_variables_mosek_task(Solver::variables);
        initilize_objective_function();

        r_code = MSK_putobjsense(task, get_mosek_objective_sense(this->objectiveFunction.get_optimization_sense()));

        SolverParam param = get_parameter();
        MSK_putintparam(task, MSK_IPAR_NUM_THREADS, param.get_number_threads()); /*Nber of cpus*/

        //Controls whether the interior-point optimizer also computes an optimal basis.
        r_code = MSK_putintparam(task, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_INTPNT);
    }

    void set_mosek_solution_status(MSKsolstae *solsta)
    {
        if (early_termination)
        {
            set_mosek_solution_statuss(solsta,  MSK_SOL_ITR);
        }
        else
        {
            set_mosek_solution_statuss(solsta, MSK_SOL_BAS);
        }
    }

    void run_optimizer()
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

    void finalize_optimization()
    {
        if (early_termination)
        {
            number_consective_early_termination++;
            if (number_consective_early_termination > param.get_number_iterations_before_optimality())
            {
                this->early_termination = false;
                std::cout<<"Changed to early = false \n";
            }
        }
        else
        {
            this->early_termination = true;
            this->number_consective_early_termination = 0;
        }
    }

    void initilize_objective_function()
    {
        if (r_code == MSK_RES_OK)
        {
            r_code = MSK_putcfix(task, this->objectiveFunction.get_constant_term());
        }

        int size = get_lp_variables()->size();

        for (int i = 0; i < size && r_code == MSK_RES_OK; ++i)
        {
            const Variable *variable = this->variables.get_variable(i);
            r_code = MSK_putcj(task, i, -1.0 * variable->get_cost());
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

    void set_solution()
    {
        set_mosek_solution_status(&solsta);

        std::cout << "\n @@@@@ Solution status  = "<< solsta <<"\n" ;
        

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
                this->variables.set_solution_value(i, var_x[i]);
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

    void add_constraint(const Constraint *constraint, bool is_to_append_new = true)
    {
        this->add_constraint_append_mosek(constraint, is_to_append_new, this->number_constraints, get_lp_variables());
        ++this->number_constraints;
    }

    void execute_constraints()
    {
        //nop 
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
};

#endif