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
private:
    ModelAbstract *model;
    CPAParam *param;
    int counter_iterations_non_optim;
    double current_solution_value;
    double previous_solution_value;

public:
    CuttingPlaneAlgorithm(ModelAbstract *model_, CPAParam *param_) : model(model_),
                                                                     param(param_)

    {
        counter_iterations_non_optim = param->get_number_iterations_between_optimality();
        current_solution_value = 0.0;
    }

    ~CuttingPlaneAlgorithm()
    {
    }

    void execute()
    {
        for (int i = 0;
             !is_stopping_criteria(i);
             ++i)
        {
            model->update_solver_termination_param(param->get_solver_termination(),
                                                   is_early_termination());
            model->solve();
            model->find_violated_constraints(param->get_number_max_violated_constraints());

            set_solution();
            

            std::cout << "i = " << i << "solution_value = " << current_solution_value << "\n";
        }
    }

    bool is_stopping_criteria(const int &iteration)
    {
        if (iteration == 0)
        {
            return false;
        }

        if (iteration >= param->get_number_max_iterations())
        {
            return true;
        }

        if (iteration > 2)
        {
            if (CONSTANTS::is_zero(previous_solution_value - current_solution_value))
            {
                return true;
            }
        }

        return false;
    }

    //TODO: CHECK IF IT IS OPTIM.... 
    void set_solution()
    {
    previous_solution_value = current_solution_value;
            current_solution_value = model->get_optimal_solution_value();
    }

    bool is_early_termination()
    {
        if (param->is_early_termination())
        {
            if (counter_iterations_non_optim < param->get_number_iterations_between_optimality())
            {
                counter_iterations_non_optim++;
                return true;
            }

            counter_iterations_non_optim = 0;
        }

        return false;
    }
};

#endif
