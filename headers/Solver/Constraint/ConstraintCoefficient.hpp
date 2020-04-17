#ifndef CONSTRAINT_COEFFICIENT_HPP
#define CONSTRAINT_COEFFICIENT_HPP

#include <string>

template <typename V>
class ConstraintCoefficient
{
private:
    const V *variable;
    double value;

    ConstraintCoefficient(const V *var, const double &val) : variable(var),
                                                             value(val)
    {
    }

public:
    static ConstraintCoefficient *create(const V *var, const double &val)
    {
        return new ConstraintCoefficient(var, val);
    }

public:

    const V *get_variable() const 
    {
        return variable;
    }

    const double &get_value() const
    {
        return value;
    }

    void update_value(const double &new_value)
    {
        this->value = new_value;
    }

    bool operator==(const ConstraintCoefficient &other) const
    {
        return this->variable == other.variable &&
               this->value == other.value;
    }


    bool operator !=(const ConstraintCoefficient &other) const
    {
        return !(*(this) == other);
    }


    ~ConstraintCoefficient()
    {
    }
};

#endif