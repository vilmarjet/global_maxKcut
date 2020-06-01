#ifndef CONSTRAINT_GENERIC_HPP
#define CONSTRAINT_GENERIC_HPP

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include "ConstraintBoundKey.hpp"
#include "ConstraintType.hpp"
#include <string>
#include <new>

class ConstraintAbstract
{

private:
protected:
    const double lowerBound;
    const double upperBound;
    const ConstraintBoundKey bound_key;
    const ConstraintType type;

public:
    ConstraintAbstract(const double &lb,
                       const double &ub,
                       const ConstraintBoundKey &bk,
                       const ConstraintType &tp) : lowerBound(lb),
                                                    upperBound(ub),
                                                    bound_key(bk),
                                                    type(tp) {}

    virtual int size() const = 0;

    double get_lower_bound() const
    {
        return this->lowerBound;
    }

    double get_upper_bound() const
    {
        return this->upperBound;
    }

    const ConstraintBoundKey &get_bound_key() const
    {
        return this->bound_key;
    }

    const ConstraintType &get_type_constraint() const 
    {
        return this->type;
    }

    const double &get_rhs() const 
    {
        switch (bound_key)
        {
        case ConstraintBoundKey::EQUAL:
            return lowerBound;

        case ConstraintBoundKey::INFERIOR_EQUAL:
            return upperBound;

        case ConstraintBoundKey::SUPERIOR_EQUAL:
            return lowerBound;

        default:
            Exception("Bound not defined", ExceptionType::STOP_EXECUTION).execute();
            return lowerBound;
        }
    }

    ~ConstraintAbstract()
    {
    }
};

#endif
