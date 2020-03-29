
#ifndef MOSEK_SOLVERS_HPP
#define MOSEK_SOLVERS_HPP

#include "mosek.h"
#include "Constraint.hpp"
#include "Variable.hpp"
#include "Solver.hpp"
#include "LPVariables.hpp"
#include <vector>

class SolverMosek
{
protected:
    MSKrescodee r_code;
    MSKtask_t task;
    MSKenv_t env;
    MSKrescodee trmcode;
    MSKsolstae solsta;

public:
    SolverMosek() : task(NULL), env(NULL), r_code(MSK_RES_OK) {}
    //virtual void set_type_mosek_solver() = 0;

    MSKboundkeye get_mosek_constraint_type(const ConstraintType &type)
    {
        switch (type)
        {
        case SUPERIOR_EQUAL:
            return MSK_BK_LO;
        case EQUAL:
            return MSK_BK_FX;
        case INFERIOR_EQUAL:
            return MSK_BK_UP;
        default:
            return MSK_BK_LO;
        }
    }

    MSKvariabletypee get_mosek_variable_type(const VariableType &type)
    {
        switch (type)
        {
        case CONTINOUS:
            return MSK_VAR_TYPE_CONT;
        case INTEGER:
            return MSK_VAR_TYPE_INT;
        default:
            return MSK_VAR_TYPE_CONT;
        }
    }

    MSKobjsensee get_mosek_objective_sense(const SenseOptimization &type)
    {
        switch (type)
        {
        case MAXIMIZATION:
            return MSK_OBJECTIVE_SENSE_MAXIMIZE;

        case MINIMIZATION:
            return MSK_OBJECTIVE_SENSE_MINIMIZE;

        default:
            return MSK_OBJECTIVE_SENSE_MAXIMIZE;
        }
    }

    static void MSKAPI printstr(void *handle,
                                const char str[])
    {
        printf("%s", str);
    } /* printstr */

    /**
    * Create task for mosek problem
    * @param nb_variables is number of variables in problem
    * @param nb_constraints nubmer of constraint in model
    */
    void create_task(const int &nb_variables, const int &nb_constraints)
    {
        std::cout << nb_variables ;
        std::cin.get();
        r_code = MSK_maketask(env, (MSKint32t)nb_constraints, (MSKint32t)nb_variables, &task);
        if (r_code != MSK_RES_OK)
        {
            std::cout << r_code;
            std::cin.get();
            throw Exception("r_code != MSK_RES_OK in create_task()", ExceptionType::STOP_EXECUTION);
        }
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

    double get_time_optimization()
    {
        double time;
        r_code = MSK_getdouinf(task, MSK_DINF_OPTIMIZER_TIME, &time);

        if (r_code != MSK_RES_OK)
        {
            throw Exception("r_code != MSK_RES_OK in get_info_optimization", ExceptionType::STOP_EXECUTION);
        }

        return time;
    }

    void set_mosek_solution_statuss(MSKsolstae *solution_sta, const MSKsoltype_enum &status)
    {
        r_code = MSK_getsolsta(task, status, solution_sta);

        if (r_code != MSK_RES_OK)
        {
            throw Exception("r_code = " + std::to_string(r_code) + " != MSK_RES_OK in set_solution_status", 
            ExceptionType::STOP_EXECUTION);
        }
    }

    ~SolverMosek()
    {
        MSK_deleteenv(&env);
        MSK_deletetask(&task);
    }

    void initialize_variables_mosek_task(const LPVariables &vars)
    {
        if (r_code == MSK_RES_OK)
        {
            size_t size = vars.size();

            if (size == 0)
            {
                return;
            }

            r_code = MSK_appendvars(task, (MSKint32t)size);

            for (int i = 0; i < size && r_code == MSK_RES_OK; ++i)
            {
                const Variable *variable = vars.get_variable(i);
                int idx = vars.get_index(variable);

                r_code = MSK_putvarbound(task,
                                         (MSKint32t)idx,               // Index of variable.
                                         MSK_BK_RA,                    // Bound key. (@todo addapt type in variable)
                                         variable->get_lower_bound(),  // Numerical value of lower bound.
                                         variable->get_upper_bound()); // Numerical value of upper bound.

                r_code = MSK_putvartype(task,
                                        (MSKint32t)i,
                                        get_mosek_variable_type(variable->get_type())); //integer
            }

            if (r_code != MSK_RES_OK)
            {
                throw Exception("r_code != MSK_RES_OK in initialize_variables_mosek_task",
                                ExceptionType::STOP_EXECUTION);
            }
        }
    }

    void initialize_sdp_variables_mosek_task(const SDPVariables *vars)
    {
        if (r_code == MSK_RES_OK)
        {
            size_t size = vars->size();
            if (size == 0)
            {
                return;
            }

            MSKint32t *DIMBARVAR = new MSKint32t[size + 1];

            for (size_t i = 0; i < size; ++i)
            {
                DIMBARVAR[i] = vars->get_variable(i)->get_dimension();
            }
            r_code = MSK_appendbarvars(task, size, DIMBARVAR);

            if (r_code != MSK_RES_OK)
            {
                throw Exception("r_code != MSK_RES_OK in initialize_sdp_variables_mosek_task",
                                ExceptionType::STOP_EXECUTION);
            }
        }
    }

    void add_constraint_append_mosek(const Constraint *constraint,
                                     const bool &is_to_append,
                                     const int &position,
                                     const LPVariables *variables)
    {
        if (is_to_append)
        {
            r_code = MSK_appendcons(task, 1);
        }

        r_code = MSK_putconbound(task,
                                 position,
                                 get_mosek_constraint_type(constraint->get_type()),
                                 constraint->get_lower_bound(),
                                 constraint->get_upper_bound());

        std::vector<int> indices_variables;
        if (r_code == MSK_RES_OK)
        {
            r_code = MSK_putarow(task,
                                 position,
                                 constraint->size(),                                              // Number of non-zeros in row i.
                                 constraint->get_indices_variables(variables, indices_variables), // Pointer to column indexes of row i.
                                 constraint->get_coefficients());
        }

        if (r_code != MSK_RES_OK)
        {
            throw Exception("r_code != MSK_RES_OK in add_constraint_mosek_task()", ExceptionType::STOP_EXECUTION);
        }
    }
};

#endif