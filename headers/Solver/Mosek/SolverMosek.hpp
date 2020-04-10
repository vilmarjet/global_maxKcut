
#ifndef MOSEK_SOLVERS_HPP
#define MOSEK_SOLVERS_HPP

#include "mosek.h"
#include "../Abstract/Solver.hpp"
#include <vector>

class SolverMosek
{
protected:
    MSKrescodee r_code;
    MSKtask_t task;
    MSKenv_t env;
    MSKrescodee trmcode;
    MSKsolstae solsta;
    int position_constraints;

public:
    SolverMosek() : task(NULL), env(NULL), r_code(MSK_RES_OK), position_constraints(0) {}
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
        r_code = MSK_maketask(env, (MSKint32t)nb_constraints, (MSKint32t)nb_variables, &task);
        if (r_code != MSK_RES_OK)
        {
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

    void add_linear_variables_mosek_task(LPVariables *vars)
    {

        size_t size = vars->get_number_non_appended_variables();

        if (size == 0)
        {
            return;
        }

        r_code = MSK_appendvars(task, (MSKint32t)size);

        for (int i = 0; i < size && r_code == MSK_RES_OK; ++i)
        {
            const Variable *variable = vars->get_next_variable_to_append();
            int idx = vars->get_index(variable);

            r_code = MSK_putvarbound(task,
                                     (MSKint32t)idx,               // Index of variable.
                                     MSK_BK_RA,                    // Bound key. (@todo addapt type in variable)
                                     variable->get_lower_bound(),  // Numerical value of lower bound.
                                     variable->get_upper_bound()); // Numerical value of upper bound.

            r_code = MSK_putvartype(task,
                                    (MSKint32t)idx,
                                    get_mosek_variable_type(variable->get_type())); //integer

            //objective function
            r_code = MSK_putcj(task, idx, variable->get_cost());
        }

        if (r_code != MSK_RES_OK)
        {
            throw Exception("r_code != MSK_RES_OK in initialize_variables_mosek_task",
                            ExceptionType::STOP_EXECUTION);
        }
    }

    void add_sdp_variables_mosek_task(SDPVariables *vars)
    {

        size_t size = vars->get_number_non_appended_variables();

        if (size == 0)
        {
            return;
        }

        MSKint32t *DIMBARVAR = new MSKint32t[size + 1];
        MSKint64t idx; //use for index in mosek
        for (size_t i = 0; i < size && r_code == MSK_RES_OK; ++i)
        {
            SDPVariable<Variable> *variable_sdp = vars->get_next_variable_to_append();
            DIMBARVAR[i] = variable_sdp->get_dimension();

            r_code = MSK_appendbarvars(task, 1, &DIMBARVAR[i]);
            int index_of_sdp = vars->get_index(variable_sdp);

            //OBJECTIVE FUNCTION
            if (variable_sdp->get_row_indices()) //check no null
            {
                r_code = MSK_appendsparsesymmat(task,
                                                variable_sdp->get_dimension(),
                                                variable_sdp->get_number_non_null_variables(),
                                                variable_sdp->get_col_indices(),
                                                variable_sdp->get_row_indices(),
                                                vars->get_cost_indices(index_of_sdp),
                                                &idx);

                if (r_code == MSK_RES_OK)
                {
                    double val = variable_sdp->get_constant_object_function();
                    r_code = MSK_putbarcj(task, index_of_sdp, 1, &idx, &val);
                }
            }
            else
            {
                Exception("null SDP variable ", ExceptionType::STOP_EXECUTION).execute();
            }
        }

        if (r_code != MSK_RES_OK)
        {
            throw Exception("r_code != MSK_RES_OK in initialize_sdp_variables_mosek_task",
                            ExceptionType::STOP_EXECUTION);
        }
    }

    void add_constraint_append_mosek(const LinearConstraint *constraint,
                                     const bool &is_to_append,
                                     const LPVariables *variablesLP)
    {
        if (is_to_append)
        {
            r_code = MSK_appendcons(task, 1);
        }

        r_code = MSK_putconbound(task,
                                 position_constraints,
                                 get_mosek_constraint_type(constraint->get_type()),
                                 constraint->get_lower_bound(),
                                 constraint->get_upper_bound());

        std::vector<int> indices_variables;
        if (r_code == MSK_RES_OK)
        {
            int size = constraint->get_number_non_null_variables();
            std::vector<int> indx_vars;
            std::vector<double> coef_vals;

            indx_vars.reserve(size);
            coef_vals.reserve(size);

            for (auto constraintCoefficient : constraint->get_constraint_coefficients())
            {
                indx_vars.push_back(variablesLP->get_index(constraintCoefficient->get_variable()));
                coef_vals.push_back(constraintCoefficient->get_value());
            }

            r_code = MSK_putarow(task,
                                 position_constraints,
                                 size,          // Number of non-zeros in row i.
                                 &indx_vars[0], // Pointer to column indexes of row i.
                                 &coef_vals[0]);
        }

        if (r_code != MSK_RES_OK)
        {
            throw Exception("r_code != MSK_RES_OK in add_constraint_mosek_task()", ExceptionType::STOP_EXECUTION);
        }

        ++this->position_constraints;
    }

    void add_constraint_append_mosek_SDP(const ConstraintSDP *constraint,
                                         const bool &is_to_append,
                                         const SDPVariables *variablesSDP)
    {
        double falpha = 1.0; //weight of A will always be 1.0
        if (is_to_append)
        {
            r_code = MSK_appendcons(task, 1);
        }

        r_code = MSK_putconbound(task,
                                 position_constraints,
                                 get_mosek_constraint_type(constraint->get_type()),
                                 constraint->get_lower_bound(),
                                 constraint->get_upper_bound());

        for (auto sdp_var : constraint->get_sdp_variables())
        {
            std::vector<ConstraintCoefficient<Variable> *> variables =
                constraint->get_coefficeints_of_variable(sdp_var);

            std::vector<int> col_idx, row_idx;
            std::vector<double> coeff;
            int non_null_variables = variables.size();

            col_idx.reserve(non_null_variables);
            row_idx.reserve(non_null_variables);
            coeff.reserve(non_null_variables);

            for (auto constraintCoefficient : constraint->get_coefficeints_of_variable(sdp_var))
            {
                const Variable *var = constraintCoefficient->get_variable();
                col_idx.push_back(sdp_var->get_col_index(var));
                row_idx.push_back(sdp_var->get_row_index(var));
                coeff.push_back(constraintCoefficient->get_value());
            }

            MSKint64t idx = 100;
            r_code = MSK_appendsparsesymmat(task,
                                            sdp_var->get_dimension(),
                                            non_null_variables,
                                            &col_idx[0],
                                            &row_idx[0],
                                            &coeff[0],
                                            &idx);

            if (r_code == MSK_RES_OK)
            {
                r_code = MSK_putbaraij(task, position_constraints, variablesSDP->get_index(sdp_var), 1, &idx, &falpha);
            }
        }

        if (r_code != MSK_RES_OK)
        {
            Exception("r_code != MSK_RES_OK in add_constraint_mosek_task()", ExceptionType::STOP_EXECUTION)
                .execute();
        }

        ++this->position_constraints;
    }
};

#endif