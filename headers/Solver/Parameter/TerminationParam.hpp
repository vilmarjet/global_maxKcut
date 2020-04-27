#ifndef EARLY_TERMINATION_PARAM
#define EARLY_TERMINATION_PARAM

#include "../Utils/Constants.hpp"

//in a builder style
class TerminationParam
{
private:
    double gap_tolerance;
    double gap_primal;
    double gap_relative_tolerance;

    TerminationParam(double gap_tolerance_,
                     const double &gap_primal_,
                     const double &gap_relative_tolerance_) : gap_tolerance(gap_tolerance_),
                                                              gap_primal(gap_primal_),
                                                              gap_relative_tolerance(gap_relative_tolerance_)
    {
    }

public:
    static TerminationParam *create(double gap_tolerance_,
                                    const double &gap_primal_,
                                    const double &gap_relative_tolerance_)
    {
        return new TerminationParam(gap_tolerance_, gap_primal_, gap_relative_tolerance_);
    }

    ~TerminationParam() {}

    TerminationParam *update_gap_tolerance(const double &gap)
    {
        this->gap_tolerance = gap;
        return this;
    }

    TerminationParam *update_gap_primal(const double &gap)
    {
        this->gap_primal = gap;
        return this;
    }

    TerminationParam *update_gap_relative_tolerance(const double &gap)
    {
        this->gap_relative_tolerance = gap;
        return this;
    }

    const double &get_gap_tolerance() const
    {
        return this->gap_tolerance;
    }

    const double &get_gap_primal() const
    {
        return this->gap_primal;
    }

    const double &get_gap_relative_tolerance() const
    {
        return this->gap_relative_tolerance;
    }
};

// builder style
template <class P>
class TerminationParamBuilder
{
private:
    double gap_tolerance;
    double gap_primal;
    double gap_relative_tolerance;
    P *parent;

    TerminationParamBuilder(P *parent_) : parent(parent_)
    {
        gap_tolerance = CONSTANTS::ZERO;
        gap_primal = CONSTANTS::ZERO;
        gap_relative_tolerance = CONSTANTS::ZERO;
    }

public:
    static TerminationParamBuilder<P> *create(P *parent)
    {
        return new TerminationParamBuilder<P>(parent);
    }

    static TerminationParamBuilder<P> *create()
    {
        std::nullptr_t np = nullptr;
        return TerminationParamBuilder<std::nullptr_t>::create(np);
    }

    ~TerminationParamBuilder() {}

    TerminationParamBuilder<P> *set_gap_tolerance(const double &gap)
    {
        this->gap_tolerance = gap;
        return this;
    }

    TerminationParamBuilder<P> *set_gap_primal(const double &gap)
    {
        this->gap_primal = gap;
        return this;
    }

    TerminationParamBuilder<P> *set_gap_relative_tolerance(const double &gap)
    {
        this->gap_relative_tolerance = gap;
        return this;
    }

    P *end()
    {
        return this->parent;
    }

    TerminationParam *build()
    {
        return TerminationParam::create(gap_tolerance,
                                        gap_primal,
                                        gap_relative_tolerance);
    }
};

#endif