#ifndef CUTTING_PLANE_ALGORITHM_HPP2
#define CUTTING_PLANE_ALGORITHM_HPP2

#include "./../Solver/Abstract/Solver.hpp"
#include "../Models/ModelAbstract.hpp"
#include "./Parameter/CPAParam.hpp"
#include "../Solver/Parameter/TerminationParam.hpp"
/**
 * Steps :
 *  1) Solve
 *  2) Find violated inequalities 
 *  3) Add most violated in 
 */
class CuttingPlaneAlgorithm
{
protected:
    ModelAbstract *model;
    CPAParam *param;

public:
    CuttingPlaneAlgorithm(ModelAbstract *model_, CPAParam *param_) : model(model_),
                                                                     param(param_)

    {
    }

    virtual void solve_model() = 0;
    virtual void find_violate_constraints() = 0;

    virtual void execute()
    {
        for (int i = 0; !is_stopping_criteria(i); ++i)
        {
            solve_model();
            find_violate_constraints();
        }
    }

    virtual bool is_stopping_criteria(const int &iteration)
    {
        if (iteration == 0)
        {
            return false;
        }

        if (iteration >= param->get_number_max_iterations())
        {
            return true;
        }

        return false;
    }

    ~CuttingPlaneAlgorithm()
    {
        delete param;
    }
};

#endif
