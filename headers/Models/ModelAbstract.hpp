#ifndef MODEL_ABSTRACT
#define MODEL_ABSTRACT

#include "../Solver/Parameter/TerminationParam.hpp"
#include "../Solver/Abstract/Solver.hpp"

class ModelAbstract
{
protected:
    Solver *solver;
public:
    ModelAbstract(Solver *solver_) : solver(solver_) {}

    virtual ModelAbstract *solve() = 0;
    virtual ModelAbstract *find_violated_constraints(const int &nb_max_ineq) = 0;
    virtual double get_optimal_solution_value()
    {
        return solver->get_optimal_solution_value();
    } 
    ModelAbstract *update_solver_termination_param(TerminationParam* param, const bool &is_early)
    {
        solver->update_termination_param(param, is_early);
        return this;
    }


    ~ModelAbstract() {}
};

#endif