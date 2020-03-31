#ifndef CONSTRAINT_GENERIC_HPP
#define CONSTRAINT_GENERIC_HPP

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include "Variable.hpp"
#include "SDPVariable.hpp"
#include "LPVariables.hpp"
#include "Constraint.hpp"
#include "../MKCGraph.hpp"
#include <string>
#include <new>


template <typename T>
class ConstraintGeneric
{

private:
protected:
    const double lowerBound;
    const double upperBound;
    const ConstraintType type;

    std::map<const T, double> coefficients;

public:

    ConstraintGeneric(const double &lb,
                  const double &ub,
                  const ConstraintType &typ) : lowerBound(lb),
                                               upperBound(ub),
                                               type(typ){}

    void add_coefficient(const T* variable, const double &value)
    {
       coefficients[variable] += value ;
    }

    double get_lower_bound() const
    {
        return this->lowerBound;
    }

    size_t size() const
    {
        return this->coefficients.size();
    }

    double get_upper_bound() const
    {
        return this->upperBound;
    }

    ConstraintType get_type() const
    {
        return this->type;
    }

    const std::map<const T, double>* get_coefficients() const
    {
        return &coefficients;
    }

    ~ConstraintGeneric()
    {
    }
};

#endif
