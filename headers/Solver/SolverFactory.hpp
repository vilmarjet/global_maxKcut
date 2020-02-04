#ifndef SOLVER_FACTORY_HPP
#define SOLVER_FACTORY_HPP

#include "Solver.hpp"
#include "Solver.hpp"
#include "SolverMosekLp.hpp"
#include "SolverMosekEarlyTerminationLP.hpp"

enum class TypeSolver
{
    LP_MOSEK,
    LP_EARLY_MOSEK,
    SDP_MOSEK
};

class SolverFactory
{
private:
    /* data */
public:
    static Solver *create_solver(const TypeSolver &type, const SolverParam &solverParm)
    {
        switch (type)
        {
        case TypeSolver::LP_MOSEK:
            return new SolverMosekLp(solverParm);
        case TypeSolver::LP_EARLY_MOSEK:
            return new SolverMosekEarlyTerminationLP(solverParm);
        default:
            return new SolverMosekLp(solverParm);
        }
    }
    ~SolverFactory(){};
};

#endif