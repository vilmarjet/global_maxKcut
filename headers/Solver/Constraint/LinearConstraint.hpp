#ifndef CONSTRAINT_LINEAR_HPP
#define CONSTRAINT_LINEAR_HPP

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include "../Variable/Variable.hpp"
#include "ConstraintCoefficient.hpp"
#include "ConstraintAbstract.hpp"
#include "../../Utils/Exception.hpp"
#include <string>
#include <new>

class LinearConstraint : public ConstraintAbstract
{
private:
    std::vector<ConstraintCoefficient<Variable> *> vec_var;
    LinearConstraint(const double &lb,
                     const double &ub,
                     const ConstraintType &typ) : ConstraintAbstract(lb, ub, typ) {}

public:
    static LinearConstraint *create()
    {
        return create(0.0, 0.0, EQUAL);
    }

    static LinearConstraint *create(const double &lb, const double &ub, const ConstraintType &typ)
    {
        return new LinearConstraint(lb, ub, typ);
    }

    ConstraintCoefficient<Variable> *add_coefficient(const Variable *var, const double &coeff)
    {
        vec_var.push_back(ConstraintCoefficient<Variable>::create(var, coeff));
        return vec_var[vec_var.size() - 1];
    }

    LinearConstraint *add_coefficients(const Variable *const *vars, const double *coeffs, const int size)
    {
        for (int i = 0; i < size; i++)
        {
            add_coefficient(vars[i], coeffs[i]);
        }

        return this;
    }

    const std::vector<ConstraintCoefficient<Variable> *> &get_variables() const
    {
        return vec_var;
    }

    const ConstraintCoefficient<Variable> *get_coefficient_constraint_by_index(const int &idx) const
    {
        if (idx < 0 || idx > get_number_non_null_variables())
        {
            Exception("Invalid index in ConstraintSDP::get_sdp_variable_by_index ", ExceptionType::STOP_EXECUTION)
                .execute();
        }

        return vec_var[idx];
    }

    int get_number_non_null_variables() const
    {
        return vec_var.size();
    }

    int size() const 
    {
        return get_number_non_null_variables();
    }

    bool operator==(const LinearConstraint &other) const
    {
        if (this->get_number_non_null_variables() != other.get_number_non_null_variables())
        {
            return false;
        }

        int size = this->get_number_non_null_variables();

        for (int i = 0; i < size; ++i)
        {
            if (*(this->get_coefficient_constraint_by_index(i)) != *(other.get_coefficient_constraint_by_index(i)))
            {
                return false;
            }
        }

        std::cout << "They are similar";
        std::cin.get();

        return true;
    }

    std::string to_string()
    {
        return "Need to be implemented";
    }

    ~LinearConstraint()
    {
    }
};

#endif
