#ifndef SOLVER_FACTORY_HPP
#define SOLVER_FACTORY_HPP

#include "../Abstract/Solver.hpp"
#include "../Mosek/SolverMosekLp.hpp"
#include "../Mosek/SolverMosekEarlyTerminationLP.hpp"
#include "../Mosek/SolverMosekSDP.hpp"

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
        case TypeSolver::SDP_MOSEK:
            return new SolverMosekSDP(solverParm);
        default:
            return new SolverMosekLp(solverParm);
        }
    }
    ~SolverFactory(){};
};

#endif