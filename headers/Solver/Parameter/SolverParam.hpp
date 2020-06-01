#ifndef SOLVER_PARAMETERS_HPP
#define SOLVER_PARAMETERS_HPP

class SolverParam
{
private:
    double ZERO = 1e-6;

    int num_of_threads_solver;
    bool early_termination;

public:
    SolverParam()
    {
        num_of_threads_solver = 1;
    }
    ~SolverParam() {}

    SolverParam *set_number_threads(const int &nb_threads)
    {
        this->num_of_threads_solver = nb_threads;
        return this;
    }

    const int &get_number_threads()
    {
        return this->num_of_threads_solver;
    }
};

#endif
