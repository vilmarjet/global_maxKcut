#ifndef MKC_CPA_HPP
#define MKC_CPA_HPP

#include "./CPA/CuttingPlaneAlgo.hpp"

namespace maxkcut
{
class MKC_CuttingPlane : public CuttingPlaneAlgorithm
{
private:
    int counter_non_optim_iterations;
    int number_violated_ineqs;
    int counter_optimal_iterations;
    double previous_solution_value;
    bool is_early;

public:
    MKC_CuttingPlane(ModelAbstract *model, CPAParam *param) : CuttingPlaneAlgorithm(model, param)
    {
        counter_non_optim_iterations = param->get_number_iterations_between_optimality();
        counter_optimal_iterations = 0;
    }

    void solve_model() override
    {
        is_early = is_early_termination();
        model->update_solver_termination_param(param->get_solver_termination(), is_early);
        model->solve();
    }

    void find_violate_constraints() override
    {
        number_violated_ineqs = model->find_violated_constraints(param->get_number_max_violated_constraints());
    }

    bool is_stopping_criteria(const int &iteration)
    {
        if (CuttingPlaneAlgorithm::is_stopping_criteria(iteration))
        {
            return true;
        }

        if (is_early)
        {
            if (number_violated_ineqs == 0)
            {
                counter_non_optim_iterations = param->get_number_iterations_between_optimality();
            }
        }
        else
        {
            double current_optimal_solution = model->get_solver()->get_optimal_solution_value();
            counter_optimal_iterations++;
            if (iteration > 0 && number_violated_ineqs == 0)
            {
                return true;
            }

            previous_solution_value = current_optimal_solution;
        }

        return false;
    }

    bool is_early_termination()
    {
        if (param->is_early_termination())
        {
            if (counter_non_optim_iterations < param->get_number_iterations_between_optimality())
            {
                counter_non_optim_iterations++;
                return true;
            }

            counter_non_optim_iterations = 0;
        }

        return false;
    }

    ~MKC_CuttingPlane()
    {
    }
};

} // namespace maxkcut

#endif