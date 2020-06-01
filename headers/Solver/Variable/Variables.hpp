#ifndef VARIABLES1d2_CONTAINER_HPP
#define VARIABLES1d2_CONTAINER_HPP

#include "Variable.hpp"
#include "../../Utils/Exception.hpp"
#include <utility> // std::pair, std::make_pair
#include <vector>
#include <map>
#include <string>
#include <typeinfo>

template <typename V>
class Variables
{
protected:
    std::vector<V *> variables;
    std::map<const V *, int> index_variables;
    int pos_last_appended_variable;

    void validate_index(const int &idx) const
    {
        if (idx >= variables.size() || idx < 0)
        {
            throw Exception("Variables: Index of variable does not exist",
                            ExceptionType::STOP_EXECUTION);
        }
    }

public:
    Variables() : pos_last_appended_variable(-1) {}
    ~Variables() {}

    V *add_variable(V *var)
    {
        int idx = variables.size();
        return add_variable_with_index(var, idx);
    }

    V *add_variable_with_index(V *var, const int &idx)
    {
        if (variables.size() <= idx)
        {
            variables.resize(idx + 1);
        }

        variables[idx] = var;
        index_variables.insert(std::pair<const V *, int>(var, idx));

        return var;
    }

    V *get_variable(const int &idx) const
    {
        validate_index(idx);
        return variables[idx];
    }

    const std::vector<V *> &get_variables() const
    {
        return variables;
    }

    V *get_next_variable_to_append()
    {
        if (pos_last_appended_variable == size() - 1)
        {
            return nullptr;
        }

        pos_last_appended_variable++;

        return get_variable(pos_last_appended_variable);
    }

    const int get_index(const V *var) const
    {
        typename std::map<const V *, int>::const_iterator it_idx = index_variables.find(var);

        if (it_idx == index_variables.end())
        {
            throw Exception("Variables1D: Variable does not exist = " + var->to_string(),
                            ExceptionType::STOP_EXECUTION);
        }

        return it_idx->second;
    }

    int get_number_non_appended_variables() const
    {
        return size() - pos_last_appended_variable - 1;
    }

    void reset_position_append_variable()
    {
        pos_last_appended_variable = -1;
    }

    int size() const
    {
        return this->variables.size();
    }
};

#endif
