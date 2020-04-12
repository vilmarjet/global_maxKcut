#ifndef CONSTRAINT_GENERIC_HPP
#define CONSTRAINT_GENERIC_HPP

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include "ConstraintType.hpp"
#include <string>
#include <new>

class ConstraintAbstract
{

private:
protected:
    const double lowerBound;
    const double upperBound;
    const ConstraintType type;

public:
    ConstraintAbstract(const double &lb,
                       const double &ub,
                       const ConstraintType &typ) : lowerBound(lb),
                                                    upperBound(ub),
                                                    type(typ) {}

    virtual int size() const = 0;

    double get_lower_bound() const
    {
        return this->lowerBound;
    }

    double get_upper_bound() const
    {
        return this->upperBound;
    }

    ConstraintType get_type() const
    {
        return this->type;
    }

    ~ConstraintAbstract()
    {
    }
};

#endif
