#ifndef CUTTING_PLANE_ALGORITHM_HPP
#define CUTTING_PLANE_ALGORITHM_HPP

#include "./../Solver/Abstract/Solver.hpp"

/**
 * Steps :
 *  1) Solve
 *  2) Find violated inequalities 
 *  3) Add most violated in 
 */
class CuttingPlaneAlgorithm
{
private:
    Solver *solver;
    // Params parameters;

public:
    CuttingPlaneAlgorithm(Solver *solver_) : solver(solver_)
    {
    }

    ~CuttingPlaneAlgorithm()
    {
    }

    virtual void find_violated_inequality() = 0;

    void solve()
    {
        // this->solver->solve();
        std::cout<<"Chegou Solver brother";
    }

};

#endif
