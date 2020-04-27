#ifndef CPA_PARAMETERS_HPP
#define CPA_PARAMETERS_HPP

#include "../../Solver/Parameter/TerminationParam.hpp"

class CPAParam
{
private:
    int number_max_iterations;
    TerminationParam *termination_param;
    bool is_early;
    int number_iterations_before_optimality;
    int number_max_violated_constraints;

    CPAParam(const int &number_max_iterations_,
             TerminationParam *early_param_,
             const bool &is_early_termination_,
             const int &nb,
             const int &nb_violate) : number_max_iterations(number_max_iterations),
                                      termination_param(early_param_),
                                      is_early(is_early_termination_),
                                      number_iterations_before_optimality(nb),
                                      number_max_violated_constraints(nb_violate)

    {
    }

public:
    static CPAParam *create(const int &number_max_iterations,
                            TerminationParam *early_param,
                            const bool &is_early_termination_,
                            const int &nb,
                            const int &nb_violate)
    {
        return new CPAParam(number_max_iterations, early_param, is_early_termination_, nb, nb_violate);
    }

    ~CPAParam() {}

    CPAParam *set_number_max_iterations(const int &number)
    {
        assert(number > 0);
        number_max_iterations = number;
        return this;
    }

    TerminationParam *get_solver_termination()
    {
        return termination_param;
    }

    const bool &is_early_termination() const
    {
        return this->is_early;
    }

    const int &get_number_max_iterations() const
    {
        return this->number_max_iterations;
    }

    const int &get_number_iterations_between_optimality() const
    {
        return this->number_iterations_before_optimality;
    }

    const int &get_number_max_violated_constraints() const
    {
        return this->number_max_violated_constraints;
    }
};

//Builder
template <typename P>
class CPAParamBuilder
{
private:
    int number_max_iterations;
    TerminationParamBuilder<CPAParamBuilder> *early_param_builder;
    bool is_early_termination;
    int number_iterations_before_optimality;
    int number_max_violated_inequalities;
    P *parent;

    CPAParamBuilder(P *parent_) : parent(parent_)
    {
        number_max_iterations = 1;
        early_param_builder = TerminationParamBuilder<CPAParamBuilder>::create(this);
        is_early_termination = false;
        number_iterations_before_optimality = 3;
        number_max_violated_inequalities = 100;
    }

public:
    ~CPAParamBuilder() {}

    static CPAParamBuilder<P> *create(P *parent)
    {
        return new CPAParamBuilder<P>(parent);
    }

    static CPAParamBuilder<P> *create()
    {
        std::nullptr_t np = nullptr;
        return CPAParamBuilder<std::nullptr_t>::create(np);
    }

    CPAParamBuilder *set_number_max_iterations(const int &number)
    {
        assert(number > 0);
        number_max_iterations = number;
        return this;
    }

    CPAParamBuilder *set_number_max_violated_constraints(const int &number)
    {
        assert(number > 0);
        number_max_violated_inequalities = number;
        return this;
    }

    TerminationParamBuilder<CPAParamBuilder> *set_early_termination(const int nb = 3)
    {
        number_iterations_before_optimality = nb;
        is_early_termination = true;
        return early_param_builder;
    }

    P *end()
    {
        return this->parent;
    }

    CPAParam *build()
    {
        return CPAParam::create(
            number_max_iterations,
            early_param_builder->build(),
            is_early_termination,
            number_iterations_before_optimality,
            number_max_violated_inequalities);
    }
};

#endif