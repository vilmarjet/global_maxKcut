#ifndef CONSTRAINTS_LINEAR_HPP
#define CONSTRAINTS_LINEAR_HPP

#include <vector>
#include "ConstraintLinear.hpp"
#include "../Utils/Exception.hpp"

class ConstraintsLinear
{
private:
    std::vector<ConstraintLinear *> constraints;
    void validate(const int &i) const
    {
        if (i < 0 || i > size())
        {
            Exception("Invalid index in ConstraintsLinear", ExceptionType::STOP_EXECUTION).execute();
        }
    }

public:
    ConstraintsLinear(/* args */) {}
    ~ConstraintsLinear()
    {
    }

    ConstraintSDP *add_constraint(const double &lb, const double &ub, const ConstraintType &typ)
    {
        constraints.push_back(ConstraintSDP::create(lb, ub, typ));

        return constraints[size() - 1];
    }

    const ConstraintSDP *get_constraint(const int &i)
    {
        validate(i);
        return constraints[i];
    }

    const size_t size() const
    {
        return constraints.size();
    }
};

#endif