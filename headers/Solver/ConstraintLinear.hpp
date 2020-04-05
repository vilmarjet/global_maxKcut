#ifndef CONSTRAINT_LINEAR_HPP
#define CONSTRAINT_LINEAR_HPP

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include "Variable.hpp"
#include "LPVariables.hpp"
#include "../MKCGraph.hpp"
#include "ConstraintCoefficient.hpp"
#include "ConstraintAbstract.hpp"
#include "../Utils/Exception.hpp"
#include <string>
#include <new>

class ConstraintLinear : public ConstraintAbstract
{
private:
    std::vector<CoefficientConstraint<Variable> *> vec_var;
    ConstraintLinear(const double &lb,
                     const double &ub,
                     const ConstraintType &typ) : ConstraintAbstract(lb, ub, typ) {}

public:
    static ConstraintLinear *create()
    {
        return create(0.0, 0.0, EQUAL);
    }

    static ConstraintLinear *create(const double &lb,
                                    const double &ub,
                                    const ConstraintType &typ)
    {
        return new ConstraintLinear(lb, ub, typ);
    }

    void add_coefficient(const Variable *var, const double &coeff)
    {
        vec_var.push_back(CoefficientConstraint<Variable>::create(var, coeff));
    }

    void add_coefficients(const Variable *const *vars, const double *coeffs, const int size)
    {
        for (int i = 0; i < size; i++)
        {
            add_coefficient(vars[i], coeffs[i]);
        }
    }

    const std::vector<CoefficientConstraint<Variable> *> &get_variables() const
    {
        return vec_var;
    }

    const CoefficientConstraint<Variable> *get_coefficient_by_index(const int &idx) const
    {
        if (idx < 0 || idx > get_number_non_null_variables())
        {
            Exception("Invalid index in ConstraintSDP::get_sdp_variable_by_index ", ExceptionType::STOP_EXECUTION)
                .execute();
        }

        return vec_var[idx];
    }

    const int get_number_non_null_variables() const
    {
        return vec_var.size();
    }

    bool operator==(const ConstraintLinear &other) const
    {
        if (this->get_number_non_null_variables() != other.get_number_non_null_variables())
        {
            return false;
        }

        int size = this->get_number_non_null_variables();

        for (int i = 0; i < size; ++i)
        {
            if (*(this->get_coefficient_by_index(i)) != *(other.get_coefficient_by_index(i)))
            {
                return false;
            }
        }

        std::cout << "They are similar";
        std::cin.get();

        return true;
    }

    ~ConstraintLinear()
    {
    }
};

#endif
