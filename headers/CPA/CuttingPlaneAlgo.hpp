#ifndef CUTTING_PLANE_ALGORITHM_HPP
#define CUTTING_PLANE_ALGORITHM_HPP

#include "./../Solver/Abstract/Solver.hpp"
#include "../Models/ModelAbstract.hpp"
#include "./Parameter/CPAParam.hpp"
/**
 * Steps :
 *  1) Solve
 *  2) Find violated inequalities 
 *  3) Add most violated in 
 */
class CuttingPlaneAlgorithm
{
private:
    ModelAbstract *model;
    const CPAParam param;
    // Params parameters;

public:
    CuttingPlaneAlgorithm(ModelAbstract *model_, const CPAParam &param_) : model(model_),
                                                                              param(param_)

    {
    }

    ~CuttingPlaneAlgorithm()
    {
    }

    virtual void find_violated_inequality() = 0;

    void solve()
    {
        // this->solver->solve();
        std::cout << "Chegou Solver brother";
    }
};

#endif
