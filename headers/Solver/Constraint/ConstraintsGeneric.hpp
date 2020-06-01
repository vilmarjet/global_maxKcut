#ifndef CONSTRAINTS_ABSTRACT_HPP
#define CONSTRAINTS_ABSTRACT_HPP

#include <vector>
#include "../../Utils/Exception.hpp"

template <typename T>
class ConstraintsGeneric
{
private:
    std::vector<T *> constraints;
    int pos_last_applied_constraint;

protected:
    bool validate(const int &i) const
    {
        if (i < 0 || i >= size())
        {
            Exception("Index out of bound in LinearConstraints", ExceptionType::STOP_EXECUTION).execute();
        }

        return true;
    }

public:
    ConstraintsGeneric(/* args */) : pos_last_applied_constraint(-1) {}

    T *add_constraint(T *constraint)
    {
        constraints.push_back(constraint);
        return constraints[size() - 1];
    }

    int size() const
    {
        return constraints.size();
    }

    T *get_constraint(const int &i) const
    {
        validate(i);
        return constraints[i];
    }

    T *get_next_constraint_to_append()
    {
        if (pos_last_applied_constraint == size() - 1)
        {
            return nullptr;
        }

        pos_last_applied_constraint++;
        return get_constraint(pos_last_applied_constraint);
    }

    int get_number_non_appended_constraints() const
    {
        return (size() - pos_last_applied_constraint - 1);
    }

    void reset_position_append_constraint()
    {
        pos_last_applied_constraint = -1;
    }

    void clear_constraints()
    {
        reset_position_append_constraint();
        constraints.clear();
    }

    ~ConstraintsGeneric() {}
};

#endif