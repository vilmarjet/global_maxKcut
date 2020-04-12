#ifndef CONSTRAINT_SDP_HPP
#define CONSTRAINT_SDP_HPP

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include "../Variable/Variable.hpp"
#include "../Variable/SDPVariable.hpp"
#include "ConstraintCoefficient.hpp"
#include "ConstraintAbstract.hpp"
#include "../../Utils/Exception.hpp"
#include <string>
#include <new>

class ConstraintSDP : public ConstraintAbstract
{

public:
    static ConstraintSDP *create()
    {
        return new ConstraintSDP(0.0, 0.0, EQUAL);
    }

    static ConstraintSDP *create(const double &lb,
                                 const double &ub,
                                 const ConstraintType &typ)
    {
        return new ConstraintSDP(lb, ub, typ);
    }

    static ConstraintSDP *from(ConstraintSDP *other)
    {
        return new ConstraintSDP(other);
    }

private:
    std::map<const SDPVariable<Variable> *, std::vector<ConstraintCoefficient<Variable> *>> map_sdp_var;
    std::vector<const SDPVariable<Variable> *> vec_sdp_var; //fix
    ConstraintSDP(ConstraintSDP *other) : ConstraintAbstract(other->lowerBound,
                                                             other->upperBound, other->type)
    {
        for (const SDPVariable<Variable> *sdp_var : other->vec_sdp_var)
        {
            for (ConstraintCoefficient<Variable> *constraintCoef : other->get_coefficeints_of_variable(sdp_var))
            {
                add_coefficient(sdp_var, constraintCoef->get_variable(), constraintCoef->get_value());
            }
        }
    }

    ConstraintSDP(const double &lb,
                  const double &ub,
                  const ConstraintType &typ) : ConstraintAbstract(lb, ub, typ) {}

public:
    ConstraintSDP *add_coefficient(const SDPVariable<Variable> *sdp_var, const Variable *var, const double &coeff)
    {
        std::map<const SDPVariable<Variable> *, std::vector<ConstraintCoefficient<Variable> *>>::iterator it;
        it = map_sdp_var.find(sdp_var);

        if (it == map_sdp_var.end())
        {
            vec_sdp_var.push_back(sdp_var);
            map_sdp_var[sdp_var].push_back(ConstraintCoefficient<Variable>::create(var, coeff));
        }
        else
        {
            it->second.push_back(ConstraintCoefficient<Variable>::create(var, coeff));
        }

        return this;
    }

    const std::map<const SDPVariable<Variable> *, std::vector<ConstraintCoefficient<Variable> *>> &get_variables() const
    {
        return map_sdp_var;
    }

    const size_t number_sdp_variables() const
    {
        return map_sdp_var.size();
    }

    const SDPVariable<Variable> *get_sdp_variable_by_index(const int &idx) const
    {
        if (idx < 0 || idx > number_sdp_variables())
        {
            Exception("Invalid index in ConstraintSDP::get_sdp_variable_by_index ", ExceptionType::STOP_EXECUTION)
                .execute();
        }

        return vec_sdp_var[idx];
    }

    const std::vector<const SDPVariable<Variable> *> &get_sdp_variables() const
    {
        return vec_sdp_var;
    }

    const std::vector<ConstraintCoefficient<Variable> *> &get_coefficeints_of_variable(
        const SDPVariable<Variable> *sdp_var) const
    {
        std::map<const SDPVariable<Variable> *, std::vector<ConstraintCoefficient<Variable> *>>::const_iterator it;
        it = map_sdp_var.find(sdp_var);

        if (it == map_sdp_var.end())
        {
            Exception("Invalid sdp_var in ConstraintSDP::get_coefficeints_of_variable ", ExceptionType::STOP_EXECUTION)
                .execute();
        }

        return it->second;
    }

    int size() const
    {
        return number_sdp_variables();
    }

    bool operator==(const ConstraintSDP &other) const
    {
        if (this->size() != other.size())
        {
            return false;
        }
        //FIXME
        // for (auto sdp_var : vec_sdp_var)
        // {
        //     get_coefficeints_of_variable(sdp_var);
        // }

        // for (int i = 0; i < size; ++i)
        // {
        //     if (*(this->get_coefficient_constraint_by_index(i)) != *(other.get_coefficient_constraint_by_index(i)))
        //     {
        //         return false;
        //     }
        // }

        return false; // it should be true
    }

    ~ConstraintSDP()
    {
    }
};

#endif
