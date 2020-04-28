#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "../Constraint/LinearConstraint.hpp"
#include "../Variable/LPVariables.hpp"
#include "../Variable/SDPVariables.hpp"
#include "../../Utils/Exception.hpp"
#include "../ObjectiveFunction/ObjectiveFunction.hpp"
#include <string>
#include <vector>
#include <set>
#include "../Parameter/SolverParam.hpp"
#include "../Constraint/ConstraintsSDP.hpp"
#include "../Constraint/ConstraintsSDP.hpp"
#include "../Constraint/LinearConstraints.hpp"
#include "../Utils/Constants.hpp"
#include "../Parameter/TerminationParam.hpp"

class Solver
{
protected:
    LPVariables *variables;
    SDPVariables *variables_sdp;
    ObjectiveFunction objectiveFunction;
    double time_solver;
    SolverParam param;
    ConstraintsSDP *constraints_sdp;
    LinearConstraints *constraints_lp;

public:
    virtual void solve() = 0;
    virtual void create_environnement() = 0;
    virtual void reset_solver() = 0;
    virtual void initialize() = 0;
    virtual void finalize_optimization() = 0;

    //constraints
    virtual void append_constraints() = 0;
    virtual void append_variables() = 0;
    virtual void update_termination_param(TerminationParam *early_param, const bool &is_early) = 0;

    Solver(const SolverParam &solverParm) : objectiveFunction(ObjectiveFunction::create()),
                                            param(solverParm),
                                            time_solver(0.0),
                                            constraints_lp(LinearConstraints::create()),
                                            constraints_sdp(ConstraintsSDP::create()),
                                            variables(LPVariables::create()),
                                            variables_sdp(SDPVariables::create())

    {
       
    }

    LinearConstraint *add_constraint_linear(LinearConstraint *constraint)
    {
        return constraints_lp->add_constraint(constraint);
    }

    ConstraintSDP *add_constraint_SDP(ConstraintSDP *constraint)
    {
        return constraints_sdp->add_constraint(constraint);
    }

    //add variables and return idx of variable;
    Variable *add_linear_variable(Variable *vars)
    {
        return this->variables->add_variable(vars);
    }

    SDPVariable<Variable> *add_sdp_variable(SDPVariable<Variable> *var)
    {
        return this->variables_sdp->add_variable(var);
    }

    LPVariables *get_lp_variables() const
    {
        return this->variables;
    }

    SDPVariables *get_sdp_variables() const
    {
        return this->variables_sdp;
    }

    LinearConstraints *get_linear_constraints()
    {
        return this->constraints_lp;
    }

    ConstraintsSDP *get_sdp_constraints()
    {
        return this->constraints_sdp;
        ;
    }

    virtual const double &get_optimal_solution_value() const
    {
        return  this->objectiveFunction.get_solution_value();
    }

    //objective function
    void set_const_objective_function(const double &cst)
    {
        this->objectiveFunction.update_constant_term(cst);
    }

    void add_time_of_solver(const double &time)
    {
        this->time_solver += time;
    }

    const SolverParam &get_parameter() const
    {
        return this->param;
    }

    std::string to_string() const
    {
        std::string s;
        s += this->objectiveFunction.to_string();
        // s += "\n******** \n";
        // s += this->variables.to_string();

        return s;
    }

    ~Solver()
    {
        delete variables;
        delete variables_sdp;
        delete constraints_sdp;
        delete constraints_lp;
    }
};

#endif