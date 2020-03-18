#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "Constraint.hpp"
#include "Variables.hpp"
#include "../Utils/Exception.hpp"
#include "ObjectiveFunction.hpp"
#include <string>
#include <vector>
#include <set>
#include "SolverParam.hpp"

class Solver
{
protected:
    Variables variables;
    ObjectiveFunction objectiveFunction;
    double time_solver;
    int number_constraints;
    SolverParam param;

public:
    virtual void solve() = 0;
    virtual void create_environnement() = 0;
    virtual void reset_solver() = 0;
    virtual void initialize() = 0;
    virtual void finalize_optimization() = 0;

    //constraints
    virtual void add_constraint(const Constraint *constraint, bool is_to_append_new = true) = 0;
    virtual void add_constraints(const std::set<Constraint> *constraints) = 0;
    virtual void add_constraints(const Constraint *constraints, const int &size_consts) = 0;

    Solver(const SolverParam &solverParm) : objectiveFunction(ObjectiveFunction::create()),
                                           param(solverParm),
                                           number_constraints(0),
                                           time_solver(0.0)
    {
    }

    //add variables and return idx of variable;
    const Variable* add_variable(Variable *vars)
    {
        return this->variables.add_variable(vars);
    }

    Variables *get_variables()
    {
        return &this->variables;
    }

    const Variables *get_variables() const
    {
        return &this->variables;
    }

    //objective function
    void set_const_objective_function(const double &cst)
    {
        this->objectiveFunction.update_constant_term(cst);
    }

    int get_nb_constraints()
    {
        return this->number_constraints;
    }

    void add_time_of_solver(const double &time)
    {
        this->time_solver += time;
    }

    const SolverParam &get_parameter()
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
};

#endif