#ifndef SOLVER_PARAMETERS_HPP
#define SOLVER_PARAMETERS_HPP

class SolverParam
{
private:
    double ZERO = 1e-6;

    double gap_tolerance;
    double gap_primal;
    double gap_relative_tolerance;
    int number_iterations_before_optimality;
    int num_of_threads_solver;
    bool early_termination;

public:
    SolverParam()
    {
        gap_tolerance = ZERO;
        gap_primal = ZERO;
        gap_relative_tolerance = ZERO;
        num_of_threads_solver = 1;

        //if early termination
        number_iterations_before_optimality = 3;
    }
    ~SolverParam() {}

    SolverParam *set_number_threads(const int &nb_threads)
    {
        this->num_of_threads_solver = nb_threads;
        return this;
    }

    SolverParam *set_gap_tolerance(const double &gap)
    {
        this->gap_tolerance = gap;
        return this;
    }

    SolverParam *set_gap_primal(const double &gap)
    {
        this->gap_primal = gap;
        return this;
    }

    SolverParam *set_gap_relative_tolerance(const double &gap)
    {
        this->gap_relative_tolerance = gap;
        return this;
    }

    const int &get_number_threads()
    {
        return this->num_of_threads_solver;
    }

    const double &get_gap_tolerance() 
    {
        return this->gap_tolerance;
    }

    const double &get_gap_primal() 
    {
        return this->gap_primal;
    }

    const double &get_gap_relative_tolerance() 
    {
        return this->gap_relative_tolerance;
    }

    const int &get_number_iterations_before_optimality() 
    {
        return this->number_iterations_before_optimality;
    }
};

#endif
