#ifndef OBJECTIVE_FUNCTION_HPP
#define OBJECTIVE_FUNCTION_HPP

#include <string>

enum SenseOptimization
{
    MAXIMIZATION,
    MINIMIZATION,
};

class ObjectiveFunction
{

private:
    const SenseOptimization senseOptimization;
    double constant_term;
    double solution;

    ObjectiveFunction(const double &cst,
                      const SenseOptimization &sense) : senseOptimization(sense),
                                                        constant_term(cst),
                                                        solution(0.0) {}

public:
    static ObjectiveFunction create()
    {
        return ObjectiveFunction::create(0.0, MAXIMIZATION);
    }

    static ObjectiveFunction create(const double &cst,
                                    const SenseOptimization &sense)
    {
        return ObjectiveFunction(cst, sense);
    }

    void update_solution(const double &sol)
    {
        this->solution = sol;
    }

    const double &get_constant_term() const
    {
        return this->constant_term;
    }

    const double &get_solution_value() const
    {
        return this->solution;
    }

    SenseOptimization get_optimization_sense() const
    {
        return this->senseOptimization;
    }

    void update_constant_term(const double &cst)
    {
        this->constant_term = cst;
    }

    std::string to_string() const 
    {
        std::string s;
        s += "\nSolution:";
        s += std::to_string(this->solution);

        return s;
    }


    ~ObjectiveFunction() {}
};

#endif
